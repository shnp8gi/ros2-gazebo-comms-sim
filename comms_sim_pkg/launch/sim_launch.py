#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
# Simulation Launch File
# Launches Gazebo, ROS-GZ Bridge, and ROS 2 nodes
# =============================================================================
"""
Launch file for the communication simulation.

This launch file:
1. Loads parameters from sim_params.yaml
2. Starts Gazebo Harmonic with the world file
3. Spawns entities based on configuration
4. Starts ROS-Gazebo bridge for sensor topics
5. Launches comms_simulator_node and ugv_controller_node

All configurations are defined in sim_params.yaml including:
- Headless mode (GUI on/off)
- World file path
- Spawn entities
- Bridge topics
- Node parameters

Usage:
    ros2 launch comms_sim_pkg sim_launch.py
"""

import os

import yaml
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    TimerAction,
    LogInfo,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_yaml_config(yaml_path: str) -> dict:
    """
    Load YAML configuration file with error handling.
    
    Args:
        yaml_path: Path to YAML configuration file
        
    Returns:
        Dictionary containing configuration
        
    Raises:
        FileNotFoundError: If config file doesn't exist
        ValueError: If YAML is invalid or empty
    """
    if not os.path.exists(yaml_path):
        raise FileNotFoundError(
            f"\n{'='*70}\n"
            f"ERROR: Configuration file not found!\n"
            f"{'='*70}\n"
            f"Expected location: {yaml_path}\n"
            f"Please ensure sim_params.yaml exists in the config directory.\n"
            f"{'='*70}\n"
        )
    
    try:
        with open(yaml_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
            
        if config is None:
            raise ValueError(
                f"\n{'='*70}\n"
                f"ERROR: Empty YAML file!\n"
                f"{'='*70}\n"
                f"File: {yaml_path}\n"
                f"The configuration file is empty or contains only comments.\n"
                f"{'='*70}\n"
            )
            
        return config
        
    except yaml.YAMLError as e:
        raise ValueError(
            f"\n{'='*70}\n"
            f"ERROR: Invalid YAML syntax!\n"
            f"{'='*70}\n"
            f"File: {yaml_path}\n"
            f"Error: {e}\n"
            f"Please check the file for syntax errors (indentation, colons, etc.)\n"
            f"{'='*70}\n"
        )


def launch_setup(context, *args, **kwargs):
    """
    Setup function for launch (called with context).
    
    This allows us to evaluate LaunchConfiguration values.
    """
    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    headless_arg = LaunchConfiguration('headless').perform(context)
    use_sim_time_bool = (str(use_sim_time).lower() == 'true')
    headless_arg_bool = (str(headless_arg).lower() == 'true')
    
    # Paths
    pkg_share = get_package_share_directory('comms_sim_pkg')
    config_path = os.path.join('/workspace', 'config', 'sim_params.yaml')
    
    # Load configuration
    config = load_yaml_config(config_path)
    
    # Get simulation settings from YAML
    default_world_path = os.path.join(pkg_share, 'resource', 'minimal_world.sdf')
    sim_config = config.get('simulation', {})
    world_file = sim_config.get('world_file', default_world_path)
    
    if not os.path.exists(world_file):
        alt_world_file = os.path.join('/workspace', world_file)
        if os.path.exists(alt_world_file):
            world_file = alt_world_file
        else:
            raise FileNotFoundError(
                f"\n{'='*70}\n"
                f"ERROR: World file not found!\n"
                f"{'='*70}\n"
                f"Tried paths:\n"
                f"  1. {world_file}\n"
                f"  2. {alt_world_file}\n"
                f"Please check 'simulation.world_file' in sim_params.yaml\n"
                f"{'='*70}\n"
            )
    world_name = sim_config.get('world_name', 'comms_sim_world')
    verbosity = sim_config.get('verbosity', 3)
    headless = headless_arg_bool if headless_arg else sim_config.get('headless', False)
    model_prefix = sim_config.get('model_path_prefix', '/workspace/models')
    
    # Get timing configuration from YAML
    timing = sim_config.get('timing', {})
    gazebo_startup_delay = timing.get('gazebo_startup_delay', 3.0)
    entity_spawn_interval = timing.get('entity_spawn_interval', 1.0)
    bridge_startup_delay = timing.get('bridge_startup_delay', 2.0)
    comms_node_delay = timing.get('comms_node_delay', 5.0)
    ugv_controller_delay = timing.get('ugv_controller_delay', 6.0)
    
    # Get spawn entity configurations from YAML
    spawn_entities = config.get('spawn_entities', {})
    
    actions = []
    
    # =========================================================================
    # Gazebo Simulation
    # =========================================================================
    if headless:
        # Headless mode (server only) - from YAML
        gz_cmd = ['gz', 'sim', '-s', '-v', str(verbosity), '-r', world_file]
    else:
        # GUI mode - from YAML
        gz_cmd = ['gz', 'sim', '-v', str(verbosity), '-r', world_file]
    
    gz_sim = ExecuteProcess(
        cmd=gz_cmd,
        name='gazebo',
        output='screen',
        additional_env={'GZ_SIM_RESOURCE_PATH': '/workspace/models'}
    )
    actions.append(gz_sim)
    
    # =========================================================================
    # Spawn Entities (with delay to ensure Gazebo is ready)
    # =========================================================================
    spawn_delay = gazebo_startup_delay  # Use YAML configuration
    
    for entity_key, entity_config in spawn_entities.items():
        model_uri = entity_config.get('model_uri', '')
        name = entity_config.get('name', entity_key)
        pose = entity_config.get('pose', [0, 0, 0, 0, 0, 0])
        
        # Validate pose
        if not isinstance(pose, list) or len(pose) != 6:
            raise ValueError(
                f"Invalid pose for entity '{entity_key}': {pose}\n"
                f"Pose must be a list of 6 numbers: [x, y, z, roll, pitch, yaw]"
            )
        
        x, y, z = pose[0], pose[1], pose[2]
        roll, pitch, yaw = pose[3], pose[4], pose[5]
        
        # Convert model:// URI to actual SDF file path
        # model://antenna -> /workspace/models/antenna/model.sdf
        model_path = model_uri.replace('model://', f'{model_prefix}/') + '/model.sdf'
        
        # Use ros_gz_sim create node for spawning
        spawn_entity = TimerAction(
            period=spawn_delay,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    name=f'spawn_{name}',
                    output='screen',
                    arguments=[
                        '-world', str(world_name),
                        '-file', str(model_path),
                        '-name', str(name),
                        '-x', str(x),
                        '-y', str(y),
                        '-z', str(z),
                        '-R', str(roll),
                        '-P', str(pitch),
                        '-Y', str(yaw),
                    ]
                )
            ]
        )
        actions.append(spawn_entity)
        spawn_delay += entity_spawn_interval
    
    # =========================================================================
    # ROS-Gazebo Bridge
    # =========================================================================
    # Load bridge topics from YAML and replace {world_name} placeholder
    bridge_topics_raw = config.get('ros_gz_bridge', {}).get('ros__parameters', {}).get('bridge_topics', [])
    
    if not bridge_topics_raw:
        print("WARNING: No bridge topics configured. ROS-Gazebo communication may not work.")
    
    bridge_topic = [t.replace('{world_name}', world_name) for t in bridge_topics_raw]
    ros_gz_bridge = TimerAction(
        period=bridge_startup_delay,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='ros_gz_bridge',
                output='screen',
                arguments=bridge_topic,
                parameters=[{'use_sim_time': use_sim_time_bool}]
            )
        ]
    )
    actions.append(ros_gz_bridge)
    
    # =========================================================================
    # Communication Simulator Node
    # =========================================================================
    comms_params = config.get('comms_simulator_node', {}).get('ros__parameters', {})
    comms_node = TimerAction(
        period=comms_node_delay,
        actions=[
            Node(
                package='comms_sim_pkg',
                executable='comms_node.py',
                name='comms_simulator_node',
                output='screen',
                parameters=[
                    {
                        'sampling_rate': float(comms_params.get('sampling_rate', 1.0)),
                        'noise_variance': float(comms_params.get('noise_variance', 2.0)),
                        'e_plane_path': comms_params.get('e_plane_path', ''),
                        'h_plane_path': comms_params.get('h_plane_path', ''),
                        'mcs_table_path': comms_params.get('mcs_table_path', ''),
                        # Align keys with comms_node.py
                        'link_establishment_time_ms': float(comms_params.get('link_establishment_time_ms', comms_params.get('link_establishment_time', 2.0))),
                        'comm_data_limit_mb': float(comms_params.get('comm_data_limit_mb', comms_params.get('comm_data_limit', 100.0))),
                        'path_loss.d0': float(comms_params.get('path_loss', {}).get('d0', 1.0)),
                        'path_loss.pl0': float(comms_params.get('path_loss', {}).get('pl0', 40.0)),
                        'path_loss.exponent': float(comms_params.get('path_loss', {}).get('exponent', 2.0)),
                        'tx_power': float(comms_params.get('tx_power', -7.0)),
                        'use_sim_time': use_sim_time == 'true'
                    },
                ],
                remappings=[
                    (f'/world/{world_name}/model/suv/link/chassis/sensor/navsat_sensor/navsat', '/gps/fix'),
                    (f'/world/{world_name}/model/suv/link/chassis/sensor/imu_sensor/imu', '/imu/data'),
                ]
            )
        ]
    )
    actions.append(comms_node)

    # =========================================================================
    # UGV Controller Node
    # =========================================================================
    ugv_params = config.get('ugv_controller_node', {}).get('ros__parameters', {})

    # Generate ROS 2 params file for UGV to avoid launch parameter normalization issues
    ugv_param_file = os.path.join(tempfile.gettempdir(), 'ugv_controller_node.params.yaml')
    waypoints_raw = ugv_params.get('waypoints', [])
    waypoints_param = []
    if waypoints_raw:
        if isinstance(waypoints_raw[0], (list, tuple)):
            for waypoint in waypoints_raw:
                waypoints_param.extend(waypoint)
        else:
            waypoints_param = waypoints_raw

    ugv_param_yaml = {
        'ugv_controller_node': {
            'ros__parameters': {
                'waypoints': waypoints_param,
                'waypoint_tolerance': float(ugv_params.get('waypoint_tolerance', 2.0)),
                'control_rate': float(ugv_params.get('control_rate', 10.0)),
                'max_angular_velocity': float(ugv_params.get('max_angular_velocity', 1.0)),
                'heading_gain': float(ugv_params.get('heading_gain', 1.5)),
                'use_sim_time': use_sim_time == 'true',
            }
        }
    }

    with open(ugv_param_file, 'w', encoding='utf-8') as f:
        yaml.safe_dump(
            ugv_param_yaml,
            f,
            sort_keys=False,
            default_flow_style=False,
            allow_unicode=True,
        )

    try:
        with open(ugv_param_file, 'r', encoding='utf-8') as f:
            preview_lines = f.read().splitlines()[:30]
        actions.append(LogInfo(msg='[ugv_controller_node] params-file: ' + ugv_param_file + "\n" + "\n".join(preview_lines)))
    except Exception as e:
        actions.append(LogInfo(msg=f'[ugv_controller_node] failed to read generated params file: {e}'))

    ugv_node = TimerAction(
        period=ugv_controller_delay,
        actions=[
            Node(
                package='comms_sim_pkg',
                executable='ugv_controller_node.py',
                name='ugv_controller_node',
                output='screen',
                parameters=[ugv_param_file]
            )
        ]
    )
    actions.append(ugv_node)

    return actions


def generate_launch_description():
    """Generate launch description."""
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    declare_headless = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode'
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_headless,
        OpaqueFunction(function=launch_setup),
    ])
