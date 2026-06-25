import os
import yaml
import copy

def load_yaml(path):
    if not os.path.exists(path):
        raise FileNotFoundError(f"YAML file not found: {path}")
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)

def deep_merge(dict1, dict2):
    """Recursively merge dict2 into dict1."""
    for key, value in dict2.items():
        if isinstance(value, dict) and key in dict1 and isinstance(dict1[key], dict):
            deep_merge(dict1[key], value)
        else:
            dict1[key] = copy.deepcopy(value)
    return dict1

def load_scenario(scenario_path):
    return load_yaml(scenario_path)

def generate_sim_params(scenario, overrides=None):
    """
    Generates the final sim_params dictionary from a scenario definition
    and applies any sweep overrides.
    """
    if 'scenario' in scenario:
        scenario = scenario['scenario']
        
    base_config_path = scenario.get('base_config', 'src/comms_sim_pkg/config/sim_params.yaml')
    
    # workspace absolute path processing (for docker)
    if not os.path.isabs(base_config_path):
        # Determine project root
        project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
        full_base_config_path = os.path.join(project_root, base_config_path)
    else:
        full_base_config_path = base_config_path
        
    base_config = load_yaml(full_base_config_path)
    
    # 1. Apply simulation overrides
    if 'simulation_overrides' in scenario:
        if 'simulation' not in base_config:
            base_config['simulation'] = {}
        deep_merge(base_config['simulation'], scenario['simulation_overrides'])
        
    # 2. Process Entities
    spawn_entities = {}
    vehicles = []
    
    entities = scenario.get('entities', [])
    model_catalog = scenario.get('model_catalog', {})
    
    for entity in entities:
        # Resolve model
        model_name = entity.get('model')
        model_info = model_catalog.get(model_name, {})
        
        # Build entity config
        entity_config = {
            'name': entity.get('name'),
            'model_uri': model_info.get('sdf_path', f"models://{model_name}"),
            'pose': entity.get('pose', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        }
        
        if 'waypoints' in entity:
            entity_config['waypoints'] = entity['waypoints']
            
        if 'static' in entity:
            entity_config['static'] = entity['static']
            
        # Build antennas
        if 'antennas' in entity:
            entity_config['antennas'] = entity['antennas']
        else:
            # Add default antenna if none specified
            entity_config['antennas'] = [{
                'name': f"{entity.get('name')}_ant",
                'offset': model_info.get('default_antenna_offset', [0.0, 0.0, 1.0]),
                'relative_rpy': [0.0, 0.0, 0.0]
            }]
            
        # Support single antenna format for spawn_entities for backward compatibility
        if entity.get('role') == 'rx':
            # For backward compatibility of base stations
            if 'antennas' in entity_config and len(entity_config['antennas']) > 0:
                entity_config['antenna_offset'] = entity_config['antennas'][0]['offset']
                entity_config['antenna_relative_rpy'] = entity_config['antennas'][0]['relative_rpy']
            spawn_entities[entity.get('name')] = entity_config
        else:
            vehicles.append(entity_config)
            
    # Apply sweep overrides
    if overrides:
        # Overrides format: [{'role': 'rx', 'field': 'pose[1]', 'value': 3.0}, ...]
        for ovr in overrides:
            role = ovr.get('role')
            name = ovr.get('entity_name')
            field = ovr.get('field')
            value = ovr.get('value')
            
            # Helper to set nested field
            def set_field(obj, field_path, val):
                # Simple parsing for field like "pose[1]" or "antennas.*.relative_rpy[2]"
                parts = field_path.split('.')
                curr = obj
                for i, part in enumerate(parts):
                    if i == len(parts) - 1:
                        # Leaf node
                        if '[' in part and ']' in part:
                            # List access
                            name_part = part[:part.index('[')]
                            idx = int(part[part.index('[')+1:part.index(']')])
                            if name_part == '':
                                curr[idx] = val
                            elif name_part == '*':
                                for item in curr:
                                    item[idx] = val
                            else:
                                curr[name_part][idx] = val
                        else:
                            curr[part] = val
                    else:
                        if part == '*':
                            # Apply to all elements in a list
                            for item in curr:
                                set_field(item, '.'.join(parts[i+1:]), val)
                            return
                        elif '[' in part and ']' in part:
                            name_part = part[:part.index('[')]
                            idx = int(part[part.index('[')+1:part.index(']')])
                            if name_part == '':
                                curr = curr[idx]
                            elif name_part == '*':
                                # Apply to specific index of all items? This is ambiguous for non-leaf.
                                # Usually * is used alone for non-leaf. 
                                pass
                            else:
                                curr = curr[name_part][idx]
                        else:
                            curr = curr[part]

            # Apply to targets
            targets = []
            if role == 'rx':
                targets = list(spawn_entities.values())
            elif role == 'tx':
                targets = vehicles
            elif name:
                # Find by name
                for v in vehicles:
                    if v.get('name') == name:
                        targets.append(v)
                for k, v in spawn_entities.items():
                    if k == name:
                        targets.append(v)
            
            for target in targets:
                try:
                    set_field(target, field, value)
                except Exception as e:
                    print(f"Warning: Failed to set field {field} on {target.get('name')}: {e}")

    # Set the generated entities back to base_config
    if spawn_entities:
        base_config['spawn_entities'] = spawn_entities
    if vehicles:
        base_config['vehicles'] = vehicles
        
    # Temporary fix for sim_launch.py if it expects old suv
    if 'spawn_entities' in base_config and 'suv' in base_config['spawn_entities']:
        del base_config['spawn_entities']['suv']
        
    return base_config

def write_sim_params(config_dict, output_path):
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    with open(output_path, 'w', encoding='utf-8') as f:
        yaml.dump(config_dict, f, default_flow_style=False, sort_keys=False, allow_unicode=True)

