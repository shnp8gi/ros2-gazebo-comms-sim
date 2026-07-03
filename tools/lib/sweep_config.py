import os
import yaml
import math

# =========================================================================
# 単位プリセット (Unit Registry)
# =========================================================================
UNIT_CONVERTERS = {
    'degree': lambda x: math.radians(x),
    'degrees': lambda x: math.radians(x),
    'km/h': lambda x: x / 3.6,
    'kmh': lambda x: x / 3.6,
}

# =========================================================================
# パラメータスイープ設定 (デフォルト値)
# =========================================================================
# パラメータスイープを繰り返す回数 (ラン数)
NUM_RUNS = 1

# 1タスクあたりの最大待機時間 [秒]
TASK_TIMEOUT_SEC = 300

# スイープ時の加速倍率 (ヘッドレス時のみ有効.1.0=リアルタイム)
SWEEP_REAL_TIME_FACTOR = 1.0

# =========================================================================
# 設定値から自動生成されるパラメータ・内部変数
# =========================================================================
PROGRESS_LOG = "tools/log/sweep_progress.log"
CONFIG_PATH = "src/comms_sim_pkg/config/sim_params.yaml"
BACKUP_PATH = "tools/sweep_build/sim_params.yaml.bak"

GENERIC_VARIABLES = []

def resolve_values(config_node):
    if 'values' in config_node:
        return config_node['values']
    elif 'range' in config_node:
        r = config_node['range']
        curr = float(r.get('start', 0.0))
        end = float(r.get('end', 1.0))
        step = float(r.get('step', 0.1))
        vals = []
        if step > 0:
            while curr <= end + 1e-5:
                vals.append(round(curr, 5))
                curr += step
        elif step < 0:
            while curr >= end - 1e-5:
                vals.append(round(curr, 5))
                curr += step
        else:
            vals.append(curr)
        return vals
    return [0.0]

def parse_target(target_node, val, fallback_unit=None):
    unit = target_node.get('unit', fallback_unit)
    converted_val = val
    if unit in UNIT_CONVERTERS:
        converted_val = UNIT_CONVERTERS[unit](val)
        
    out = {
        'field': target_node.get('field'),
        'value': converted_val,
        'raw_value': val,
        'unit': unit if unit else ""
    }
    if 'entity_role' in target_node:
        out['role'] = target_node['entity_role']
    elif 'role' in target_node:
        out['role'] = target_node['role']
        
    if 'entity_name' in target_node:
        out['entity_name'] = target_node['entity_name']
    elif 'name' in target_node:
        out['entity_name'] = target_node['name']
    return out

def load_sweep_config(yaml_path=None):
    global NUM_RUNS, TASK_TIMEOUT_SEC, SWEEP_REAL_TIME_FACTOR
    global GENERIC_VARIABLES
    
    sweep_data = None
    GENERIC_VARIABLES = []
    
    if yaml_path and os.path.exists(yaml_path):
        with open(yaml_path, 'r', encoding='utf-8') as f:
            sweep_data = yaml.safe_load(f).get('sweep', {})
            
    if sweep_data:
        exec_cfg = sweep_data.get('execution', {})
        NUM_RUNS = exec_cfg.get('num_runs', NUM_RUNS)
        TASK_TIMEOUT_SEC = exec_cfg.get('task_timeout_sec', TASK_TIMEOUT_SEC)
        SWEEP_REAL_TIME_FACTOR = exec_cfg.get('real_time_factor', SWEEP_REAL_TIME_FACTOR)
        
        for var in sweep_data.get('variables', []):
            var_data = {
                'name': var.get('name'),
                'states': []
            }
            
            unit = var.get('unit')
            
            if 'targets' in var:
                targets_defs = var['targets']
                target_value_lists = []
                for t in targets_defs:
                    target_value_lists.append(resolve_values(t))
                
                if not target_value_lists:
                    continue
                    
                min_len = min(len(l) for l in target_value_lists)
                states = []
                for i in range(min_len):
                    state_overrides = []
                    for t_idx, t in enumerate(targets_defs):
                        raw_val = target_value_lists[t_idx][i]
                        state_overrides.append(parse_target(t, raw_val, unit))
                    states.append(state_overrides)
                var_data['states'] = states
                
            elif 'target' in var:
                target_def = var['target']
                vals = resolve_values(var)
                states = []
                for val in vals:
                    states.append([parse_target(target_def, val, unit)])
                var_data['states'] = states
                
            if var_data['states']:
                GENERIC_VARIABLES.append(var_data)
                
    return sweep_data

# Initialize default
load_sweep_config()
