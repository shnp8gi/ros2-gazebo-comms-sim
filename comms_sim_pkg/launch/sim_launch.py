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
3. Spawns entities (SUV, antenna) based on configuration
4. Starts ROS-Gazebo bridge for sensor topics
5. Launches comms_simulator_node and ugv_controller_node

Usage:
    ros2 launch comms_sim_pkg sim_launch.py
    ros2 launch comms_sim_pkg sim_launch.py headless:=true
"""

import os
from typing import List, Tuple

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node


def load_yaml_config(yaml_path: str) -> dict:
    """Load YAML configuration file."""
    with open(yaml_path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def generate_spawn_entity_processes(
    config: dict,
    world_name: str
) -> List[ExecuteProcess]:
    """
    Generate spawn entity processes from configuration.
    
    Args:
        config: Loaded sim_params.yaml configuration
        world_name: Name of the Gazebo world
        
    Returns:
        List of ExecuteProcess actions for spawning entities
    """
    spawn_actions = []
    spawn_entities = config.get('spawn_entities', {})
    
    for entity_key, entity_config in spawn_entities.items():
        model_uri = entity_config.get('model_uri', '')
        name = entity_config.get('name', entity_key)
        pose = entity_config.get('pose', [0, 0, 0, 0, 0, 0])
        
        # Extract pose components
        x, y, z = pose[0], pose[1], pose[2]
        roll, pitch, yaw = pose[3], pose[4], pose[5]
        
        # Create spawn command using gz service
        spawn_cmd = [
            'gz', 'service', '-s', '/world/' + world_name + '/create',
            '--reqtype', 'gz.msgs.EntityFactory',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '5000',
            '--req',
            f'sdf_filename: "{model_uri}", '
            f'name: "{name}", '
            f'pose: {{position: {{x: {x}, y: {y}, z: {z}}}, '
            f'orientation: {{x: {roll}, y: {pitch}, z: {yaw}}}}}'
        ]
        
        spawn_actions.append(
            ExecuteProcess(
                cmd=spawn_cmd,
                name=f'spawn_{name}',
                output='screen'
            )
        )
    
    return spawn_actions


def launch_setup(context, *args, **kwargs):
    """
    Setup function for launch (called with context).
    
    This allows us to evaluate LaunchConfiguration values.
    """
    # Get launch configurations
    headless = LaunchConfiguration('headless').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    
    # Paths
    pkg_share = get_package_share_directory('comms_sim_pkg')
    config_path = os.path.join('/workspace', 'config', 'sim_params.yaml')
    
    # Load configuration
    config = load_yaml_config(config_path)
    
    # Get simulation settings
    sim_config = config.get('simulation', {})
    world_file = sim_config.get('world_file', 
        os.path.join(pkg_share, 'resource', 'minimal_world.sdf'))
    verbosity = sim_config.get('verbosity', 3)
    
    # World name (extracted from SDF or default)
    world_name = 'comms_sim_world'
    
    # Get spawn entity configurations
    spawn_entities = config.get('spawn_entities', {})
    
    # Get node parameters
    comms_params = config.get('comms_simulator_node', {}).get('ros__parameters', {})
    ugv_params = config.get('ugv_controller_node', {}).get('ros__parameters', {})
    
    # Base station (antenna) position from spawn_entities
    antenna_config = spawn_entities.get('antenna', {})
    antenna_pose = antenna_config.get('pose', [0, 0, 0, 0, 0, 0])
    antenna_height_offset = antenna_config.get('antenna_height_offset', 1.9)
    
    # UGV antenna offset
    suv_config = spawn_entities.get('suv', {})
    suv_antenna_offset = suv_config.get('antenna_height_offset', 1.3)
    
    actions = []
    
    # =========================================================================
    # Gazebo Simulation
    # =========================================================================
    if headless == 'true':
        # Headless mode (server only)
        gz_cmd = ['gz', 'sim', '-s', '-v', str(verbosity), '-r', world_file]
    else:
        # GUI mode
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
    spawn_delay = 3.0  # seconds
    
    for entity_key, entity_config in spawn_entities.items():
        model_uri = entity_config.get('model_uri', '')
        name = entity_config.get('name', entity_key)
        pose = entity_config.get('pose', [0, 0, 0, 0, 0, 0])
        
        x, y, z = pose[0], pose[1], pose[2]
        roll, pitch, yaw = pose[3], pose[4], pose[5]
        
        # Convert model:// URI to actual SDF file path
        # model://antenna -> /workspace/models/antenna/model.sdf
        model_path = model_uri.replace('model://', '/workspace/models/') + '/model.sdf'
        
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
                        '-world', world_name,
                        '-file', model_path,
                        '-name', name,
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
        spawn_delay += 1.0  # Stagger spawns
    
    # =========================================================================
    # ROS-Gazebo Bridge
    # =========================================================================
    bridge_config = [
        # GPS/NavSat
        f'/world/{world_name}/model/suv/link/chassis/sensor/navsat_sensor/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
        # IMU
        f'/world/{world_name}/model/suv/link/chassis/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        # Odometry
        f'/model/suv/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        # Command velocity
        '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        # Clock
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
    ]
    
    ros_gz_bridge = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='ros_gz_bridge',
                output='screen',
                arguments=bridge_config,
                parameters=[{'use_sim_time': use_sim_time == 'true'}]
            )
        ]
    )
    actions.append(ros_gz_bridge)
    
    # Remap topics to standard names
    topic_remapper = TimerAction(
        period=2.5,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='topic_remapper',
                output='screen',
                arguments=[
                    f'/world/{world_name}/model/suv/link/chassis/sensor/navsat_sensor/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
                ],
                remappings=[
                    (f'/world/{world_name}/model/suv/link/chassis/sensor/navsat_sensor/navsat', '/gps/fix'),
                ]
            )
        ]
    )
    # Note: Topic remapping handled in node subscriptions
    
    # =========================================================================
    # Communication Simulator Node
    # =========================================================================
    comms_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='comms_sim_pkg',
                executable='comms_node.py',
                name='comms_simulator_node',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time == 'true'},
                    {'sampling_rate': comms_params.get('sampling_rate', 1.0)},
                    {'noise_variance': comms_params.get('noise_variance', 2.0)},
                    {'e_plane_path': comms_params.get('e_plane_path', '')},
                    {'h_plane_path': comms_params.get('h_plane_path', '')},
                    {'comm_data_limit_mb': comms_params.get('comm_data_limit_mb', -1.0)},
                    {'path_loss.d0': comms_params.get('path_loss', {}).get('d0', 1.0)},
                    {'path_loss.pl0': comms_params.get('path_loss', {}).get('pl0', 40.0)},
                    {'path_loss.exponent': comms_params.get('path_loss', {}).get('exponent', 2.0)},
                    {'tx_power': comms_params.get('tx_power', 20.0)},
                    # Base station (antenna) position from spawn_entities
                    {'base_station_position': antenna_pose[:3]},
                    {'base_station_antenna_offset': antenna_height_offset},
                    {'ugv_antenna_offset': suv_antenna_offset},
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
    ugv_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='comms_sim_pkg',
                executable='ugv_controller_node.py',
                name='ugv_controller_node',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time == 'true'},
                    {'waypoints': ugv_params.get('waypoints', [])},
                    {'waypoint_tolerance': ugv_params.get('waypoint_tolerance', 2.0)},
                    {'control_rate': ugv_params.get('control_rate', 10.0)},
                    {'max_angular_velocity': ugv_params.get('max_angular_velocity', 1.0)},
                    {'heading_gain': ugv_params.get('heading_gain', 1.5)},
                ],
                remappings=[
                    ('/odom', '/model/suv/odometry'),
                ]
            )
        ]
    )
    actions.append(ugv_node)
    
    return actions


def generate_launch_description():
    """Generate launch description."""
    
    # Declare launch arguments
    declare_headless = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode (no GUI)'
    )
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value='/workspace/config/sim_params.yaml',
        description='Path to simulation parameters YAML file'
    )
    
    return LaunchDescription([
        declare_headless,
        declare_use_sim_time,
        declare_config_file,
        OpaqueFunction(function=launch_setup),
    ])
