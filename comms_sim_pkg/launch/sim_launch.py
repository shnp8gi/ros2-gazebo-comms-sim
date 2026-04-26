#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
# シミュレーション起動ファイル
# Gazebo、ROS-GZブリッジ、ROS 2ノードを起動する
# =============================================================================
"""
通信シミュレーションのlaunchファイル。

このlaunchファイルは以下を実行する:
1. sim_params.yaml からパラメータを読み込む
2. ワールドファイルでGazebo Harmonicを起動する
3. 設定に基づきエンティティ（基地局等）をスポーンする
4. 複数車両をスポーンする（各車両固有のトピック名を持つSDF生成）
5. センサトピック用のROS-Gazeboブリッジを起動する
6. 各車両ごとに comms_simulator_node と ugv_controller_node を起動する

全設定は sim_params.yaml で定義:
- ヘッドレスモード（GUI有無）
- ワールドファイルパス
- スポーンエンティティ（基地局等）
- 車両リスト（vehicles セクション）
- ブリッジトピック
- ノードパラメータ

使用方法:
    ros2 launch comms_sim_pkg sim_launch.py
"""

import os
import re

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
    RegisterEventHandler,
    EmitEvent,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_yaml_config(yaml_path: str) -> dict:
    """
    YAMLコンフィグファイルをエラーハンドリング付きで読み込む。

    Args:
        yaml_path: YAMLコンフィグファイルのパス

    Returns:
        設定を含む辞書

    Raises:
        FileNotFoundError: コンフィグファイルが存在しない場合
        ValueError: YAMLが不正または空の場合
    """
    if not os.path.exists(yaml_path):
        raise FileNotFoundError(
            f"\n{'='*70}\n"
            f"エラー: コンフィグファイルが見つかりません\n"
            f"{'='*70}\n"
            f"パス: {yaml_path}\n"
            f"configディレクトリにsim_params.yamlが存在するか確認してください。\n"
            f"{'='*70}\n"
        )

    try:
        with open(yaml_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)

        if config is None:
            raise ValueError(
                f"\n{'='*70}\n"
                f"エラー: 空のYAMLファイルです\n"
                f"{'='*70}\n"
                f"ファイル: {yaml_path}\n"
                f"コンフィグファイルが空またはコメントのみです。\n"
                f"{'='*70}\n"
            )

        return config

    except yaml.YAMLError as e:
        raise ValueError(
            f"\n{'='*70}\n"
            f"エラー: YAMLの構文エラーです\n"
            f"{'='*70}\n"
            f"ファイル: {yaml_path}\n"
            f"エラー: {e}\n"
            f"インデント、コロン等の構文を確認してください。\n"
            f"{'='*70}\n"
        )


def _resolve_model_uri(model_uri: str, model_prefix: str) -> str:
    """
    モデルURIを実際のSDFファイルパスに変換する。

    models://SUV や model://SUV 形式のURIを
    /workspace/models/SUV/model.sdf のような実際のパスに変換する。

    Args:
        model_uri: モデルURI（例: "models://SUV"）
        model_prefix: モデルディレクトリパス（例: "/workspace/models"）

    Returns:
        SDFファイルの絶対パス
    """
    # models:// と model:// の両方に対応
    path = model_uri
    if path.startswith('models://'):
        path = path.replace('models://', f'{model_prefix}/', 1)
    elif path.startswith('model://'):
        path = path.replace('model://', f'{model_prefix}/', 1)
    return path + '/model.sdf'


def _generate_vehicle_sdf(original_sdf_path: str, vehicle_name: str) -> str:
    """
    車両固有のトピック名を持つSDFファイルを動的生成する。

    DiffDriveプラグインの <topic> と <odom_topic> タグを
    車両名付きのトピック名に書き換えた一時SDFファイルを生成する。

    Args:
        original_sdf_path: 元のモデルSDFファイルパス
        vehicle_name: 車両名（トピックプレフィックスに使用）

    Returns:
        生成された一時SDFファイルのパス
    """
    with open(original_sdf_path, 'r', encoding='utf-8') as f:
        sdf_content = f.read()

    # DiffDriveプラグインのトピック名を車両固有に書き換え
    # <topic>cmd_vel</topic> → <topic>/{vehicle_name}/cmd_vel</topic>
    sdf_content = re.sub(
        r'<topic>\s*cmd_vel\s*</topic>',
        f'<topic>/{vehicle_name}/cmd_vel</topic>',
        sdf_content
    )

    # <odom_topic>odom</odom_topic> → <odom_topic>/{vehicle_name}/odom</odom_topic>
    sdf_content = re.sub(
        r'<odom_topic>\s*odom\s*</odom_topic>',
        f'<odom_topic>/{vehicle_name}/odom</odom_topic>',
        sdf_content
    )

    # 一時ファイルに保存
    tmp_dir = os.path.join(tempfile.gettempdir(), 'comms_sim_vehicles')
    os.makedirs(tmp_dir, exist_ok=True)
    tmp_sdf_path = os.path.join(tmp_dir, f'{vehicle_name}_model.sdf')

    with open(tmp_sdf_path, 'w', encoding='utf-8') as f:
        f.write(sdf_content)

    return tmp_sdf_path


def _build_vehicles_from_legacy(config: dict) -> list:
    """
    旧形式の設定（spawn_entities.suv + ugv_controller_node）から
    vehiclesリスト形式に変換する。後方互換用。

    Args:
        config: YAMLコンフィグ全体

    Returns:
        vehiclesリスト（1車両分）
    """
    spawn_entities = config.get('spawn_entities', {})
    suv_cfg = spawn_entities.get('suv') or spawn_entities.get('SUV')
    if not isinstance(suv_cfg, dict):
        return []

    ugv_params = config.get('ugv_controller_node', {}).get('ros__parameters', {})

    vehicle = {
        'name': suv_cfg.get('name', 'suv'),
        'model_uri': suv_cfg.get('model_uri', 'models://SUV'),
        'pose': suv_cfg.get('pose', [-60.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        'antenna_offset': suv_cfg.get('antenna_offset', [0.0, 0.0, 2.23]),
        'antenna_relative_rpy': suv_cfg.get('antenna_relative_rpy', [0.0, 0.0, 0.0]),
        'waypoints': ugv_params.get('waypoints', []),
    }

    return [vehicle]


def launch_setup(context, *args, **kwargs):
    """
    launchセットアップ関数（コンテキスト付きで呼び出される）。

    LaunchConfigurationの値を評価するためにコンテキストが必要。
    """
    # launch引数の取得
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    headless_arg = LaunchConfiguration('headless').perform(context)
    use_sim_time_bool = (str(use_sim_time).lower() == 'true')
    headless_arg_bool = (str(headless_arg).lower() == 'true')

    # パス設定
    pkg_share = get_package_share_directory('comms_sim_pkg')
    config_path = os.path.join('/workspace', 'config', 'sim_params.yaml')

    # コンフィグ読み込み
    config = load_yaml_config(config_path)

    # YAMLからシミュレーション設定を取得
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
                f"エラー: ワールドファイルが見つかりません\n"
                f"{'='*70}\n"
                f"検索パス:\n"
                f"  1. {world_file}\n"
                f"  2. {alt_world_file}\n"
                f"sim_params.yaml の 'simulation.world_file' を確認してください。\n"
                f"{'='*70}\n"
            )
    world_name = sim_config.get('world_name', 'comms_sim_world')
    verbosity = sim_config.get('verbosity', 3)
    headless = headless_arg_bool if headless_arg else sim_config.get('headless', False)
    model_prefix = sim_config.get('model_path_prefix', '/workspace/models')

    # YAMLからタイミング設定を取得
    timing = sim_config.get('timing', {})
    gazebo_startup_delay = timing.get('gazebo_startup_delay', 3.0)
    entity_spawn_interval = timing.get('entity_spawn_interval', 1.0)
    bridge_startup_delay = timing.get('bridge_startup_delay', 2.0)
    comms_node_delay = timing.get('comms_node_delay', 5.0)
    ugv_controller_delay = timing.get('ugv_controller_delay', 6.0)

    # YAMLからスポーンエンティティ設定を取得（基地局等）
    spawn_entities = config.get('spawn_entities', {})

    # =====================================================================
    # 車両リストの取得（後方互換対応）
    # =====================================================================
    vehicles = config.get('vehicles', None)
    if vehicles is None or not isinstance(vehicles, list) or len(vehicles) == 0:
        # 旧形式フォールバック: spawn_entities.suv + ugv_controller_node から生成
        vehicles = _build_vehicles_from_legacy(config)
        if vehicles:
            # 旧形式の suv を spawn_entities から除外（車両は vehicles で処理する）
            spawn_entities = {k: v for k, v in spawn_entities.items()
                              if k.lower() != 'suv'}

    # UGVコントローラ共通パラメータ
    ugv_common = config.get('ugv_controller_common', {})
    # 旧形式フォールバック
    if not ugv_common:
        ugv_common = config.get('ugv_controller_node', {}).get('ros__parameters', {})
    waypoint_tolerance = float(ugv_common.get('waypoint_tolerance', 2.0))
    control_rate = float(ugv_common.get('control_rate', 10.0))
    max_angular_velocity = float(ugv_common.get('max_angular_velocity', 1.0))
    heading_gain = float(ugv_common.get('heading_gain', 1.5))

    actions = []

    # =========================================================================
    # Gazeboシミュレーション
    # =========================================================================
    if headless:
        # ヘッドレスモード（サーバーのみ）
        gz_cmd = ['gz', 'sim', '-s', '-v', str(verbosity), '-r', world_file]
    else:
        # GUIモード
        gz_cmd = ['gz', 'sim', '-v', str(verbosity), '-r', '--render-engine', 'ogre', world_file]

    gz_sim = ExecuteProcess(
        cmd=gz_cmd,
        name='gazebo',
        output='screen',
        additional_env={'GZ_SIM_RESOURCE_PATH': '/workspace/models'}
    )
    actions.append(gz_sim)

    # =========================================================================
    # エンティティスポーン: 基地局等の静的エンティティ（Gazebo起動待機後）
    # =========================================================================
    spawn_delay = gazebo_startup_delay

    for entity_key, entity_config in spawn_entities.items():
        model_uri = entity_config.get('model_uri', '')
        name = entity_config.get('name', entity_key)
        pose = entity_config.get('pose', [0, 0, 0, 0, 0, 0])

        # ポーズ検証
        if not isinstance(pose, list) or len(pose) != 6:
            raise ValueError(
                f"エンティティ '{entity_key}' のポーズが不正: {pose}\n"
                f"ポーズは6要素のリスト [x, y, z, roll, pitch, yaw] でなければなりません"
            )

        x, y, z = pose[0], pose[1], pose[2]
        roll, pitch, yaw = pose[3], pose[4], pose[5]

        # model:// / models:// URIを実際のSDFファイルパスに変換
        model_path = _resolve_model_uri(model_uri, model_prefix)

        # ros_gz_sim の create ノードでスポーン
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
    # 車両スポーン: 各車両固有のSDF生成とスポーン
    # =========================================================================
    for vehicle_cfg in vehicles:
        v_name = vehicle_cfg.get('name', 'suv')
        v_model_uri = vehicle_cfg.get('model_uri', 'models://SUV')
        v_pose = vehicle_cfg.get('pose', [0, 0, 0, 0, 0, 0])

        # ポーズ検証
        if not isinstance(v_pose, list) or len(v_pose) != 6:
            raise ValueError(
                f"車両 '{v_name}' のポーズが不正: {v_pose}\n"
                f"ポーズは6要素のリスト [x, y, z, roll, pitch, yaw] でなければなりません"
            )

        x, y, z = v_pose[0], v_pose[1], v_pose[2]
        roll, pitch, yaw = v_pose[3], v_pose[4], v_pose[5]

        # 元のSDFパスを解決
        original_sdf_path = _resolve_model_uri(v_model_uri, model_prefix)

        # 車両固有のSDFを生成（トピック名を書き換え）
        vehicle_sdf_path = _generate_vehicle_sdf(original_sdf_path, v_name)

        actions.append(LogInfo(
            msg=f'[vehicle] {v_name}: SDF生成完了 → {vehicle_sdf_path}'
        ))

        spawn_vehicle = TimerAction(
            period=spawn_delay,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    name=f'spawn_{v_name}',
                    output='screen',
                    arguments=[
                        '-world', str(world_name),
                        '-file', str(vehicle_sdf_path),
                        '-name', str(v_name),
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
        actions.append(spawn_vehicle)
        spawn_delay += entity_spawn_interval

    # =========================================================================
    # ROS-Gazeboブリッジ
    # =========================================================================
    # YAMLからブリッジトピックを読み込み（共通トピックのみ）
    bridge_topics_raw = config.get('ros_gz_bridge', {}).get('ros__parameters', {}).get('bridge_topics', [])
    bridge_topic = [t.replace('{world_name}', world_name) for t in bridge_topics_raw]

    # 各車両固有のブリッジトピックを動的に追加
    for vehicle_cfg in vehicles:
        v_name = vehicle_cfg.get('name', 'suv')

        # IMUセンサ: Gazeboモデル内のIMUトピック → ROS2
        bridge_topic.append(
            f"/world/{world_name}/model/{v_name}/link/chassis/sensor/imu_sensor/imu"
            f"@sensor_msgs/msg/Imu[gz.msgs.IMU"
        )

        # オドメトリ: DiffDriveプラグインが生成
        bridge_topic.append(
            f"/{v_name}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry"
        )

        # 速度指令: ROS → Gazebo DiffDriveプラグイン
        bridge_topic.append(
            f"/{v_name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist"
        )

    if not bridge_topic:
        print("警告: ブリッジトピックが設定されていません。ROS-Gazebo間通信が機能しない可能性があります。")

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
    # 基地局パラメータの抽出（通信ノード用）
    # =========================================================================
    antenna_base_position = [0.0, 0.0, 0.0]
    antenna_antenna_offset = [0.0, 0.0, 3.0]
    antenna_relative_rpy = [0.0, 0.0, 0.0]

    if isinstance(spawn_entities, dict):
        antenna_cfg = spawn_entities.get('antenna') or spawn_entities.get('Antenna')
        if isinstance(antenna_cfg, dict):
            pose = antenna_cfg.get('pose')
            if isinstance(pose, list) and len(pose) >= 3:
                antenna_base_position = [float(pose[0]), float(pose[1]), float(pose[2])]
            antenna_offset_raw = antenna_cfg.get('antenna_offset')
            if isinstance(antenna_offset_raw, list) and len(antenna_offset_raw) >= 3:
                antenna_antenna_offset = [float(v) for v in antenna_offset_raw[:3]]
            else:
                h = float(antenna_cfg.get('antenna_height_offset', 3.0))
                antenna_antenna_offset = [0.0, 0.0, h]
            rpy = antenna_cfg.get('antenna_relative_rpy')
            if isinstance(rpy, list) and len(rpy) >= 3:
                antenna_relative_rpy = [float(r) for r in rpy[:3]]

    # =========================================================================
    # 各車両ごとの通信ノード＆UGVコントローラノードの起動
    # =========================================================================
    comms_params = config.get('comms_simulator_node', {}).get('ros__parameters', {})

    for vehicle_cfg in vehicles:
        v_name = vehicle_cfg.get('name', 'suv')
        v_pose = vehicle_cfg.get('pose', [-20.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # 車両のアンテナパラメータ
        v_antenna_offset = [0.0, 0.0, 1.9]
        v_antenna_relative_rpy = [0.0, 0.0, 0.0]

        v_offset_raw = vehicle_cfg.get('antenna_offset')
        if isinstance(v_offset_raw, list) and len(v_offset_raw) >= 3:
            v_antenna_offset = [float(v) for v in v_offset_raw[:3]]

        v_rpy = vehicle_cfg.get('antenna_relative_rpy')
        if isinstance(v_rpy, list) and len(v_rpy) >= 3:
            v_antenna_relative_rpy = [float(r) for r in v_rpy[:3]]

        # スポーン位置
        suv_pose = [float(v_pose[0]), float(v_pose[1]), float(v_pose[2])]

        # -----------------------------------------------------------------
        # 通信シミュレータノード（車両ごと）
        # -----------------------------------------------------------------
        comms_node = TimerAction(
            period=comms_node_delay,
            actions=[
                Node(
                    package='comms_sim_pkg',
                    executable='comms_node.py',
                    name=f'comms_simulator_{v_name}',
                    output='screen',
                    parameters=[
                        {
                            'vehicle_name': v_name,
                            'sampling_rate': float(comms_params.get('sampling_rate', 1.0)),
                            'noise_variance': float(comms_params.get('noise_variance', 2.0)),
                            'e_plane_path': comms_params.get('e_plane_path', ''),
                            'h_plane_path': comms_params.get('h_plane_path', ''),
                            'mcs_table_path': comms_params.get('mcs_table_path', ''),
                            'link_establishment_time_ms': float(comms_params.get('link_establishment_time_ms', comms_params.get('link_establishment_time', 2.0))),
                            'comm_data_limit_mb': float(comms_params.get('comm_data_limit_mb', comms_params.get('comm_data_limit', 100.0))),
                            'path_loss.c': float(comms_params.get('path_loss', {}).get('c', 299792458)),
                            'path_loss.frequency': float(comms_params.get('path_loss', {}).get('frequency', 6.0e10)),
                            'path_loss.exponent': float(comms_params.get('path_loss', {}).get('exponent', 2.0)),
                            'path_loss.d0': float(comms_params.get('path_loss', {}).get('d0', 1.0)),
                            'path_loss.pl_d0': float(comms_params.get('path_loss', {}).get('pl_d0', -1.0)),
                            'tx_power': float(comms_params.get('tx_power', -7.0)),
                            'max_antenna_attenuation': float(comms_params.get('max_antenna_attenuation', 30.0)),
                            'logging_start_trigger': str(comms_params.get('logging_start_trigger', 'on_movement')),
                            'logging_start_topic': str(comms_params.get('logging_start_topic', '/logging/start')),
                            'ugv_spawn_pose': suv_pose,
                            'base_station_position': antenna_base_position,
                            'base_station_antenna_offset': antenna_antenna_offset,
                            'ugv_antenna_offset': v_antenna_offset,
                            'base_station_antenna_relative_rpy': antenna_relative_rpy,
                            'ugv_antenna_relative_rpy': v_antenna_relative_rpy,
                            'odom_topic': f'/{v_name}/odom',
                            'cmd_vel_topic': f'/{v_name}/cmd_vel',
                            'mission_complete_topic': f'/{v_name}/mission_complete',
                            'use_sim_time': use_sim_time == 'true'
                        },
                    ],
                    remappings=[
                        ('/imu/data', f'/world/{world_name}/model/{v_name}/link/chassis/sensor/imu_sensor/imu'),
                    ]
                )
            ]
        )
        actions.append(comms_node)

        # -----------------------------------------------------------------
        # UGVコントローラノード（車両ごと）
        # -----------------------------------------------------------------
        # ウェイポイントの解析
        waypoints_raw = vehicle_cfg.get('waypoints', [])
        waypoints_param = []
        if waypoints_raw:
            if isinstance(waypoints_raw[0], (list, tuple)):
                for waypoint in waypoints_raw:
                    waypoints_param.extend([float(v) for v in waypoint])
            else:
                waypoints_param = [float(v) for v in waypoints_raw]

        ugv_param_file = os.path.join(
            tempfile.gettempdir(), f'ugv_controller_{v_name}.params.yaml'
        )

        ugv_param_yaml = {
            f'ugv_controller_{v_name}': {
                'ros__parameters': {
                    'waypoints': waypoints_param,
                    'waypoint_tolerance': waypoint_tolerance,
                    'control_rate': control_rate,
                    'max_angular_velocity': max_angular_velocity,
                    'heading_gain': heading_gain,
                    'spawn_pose': suv_pose,
                    'odom_topic': f'/{v_name}/odom',
                    'cmd_vel_topic': f'/{v_name}/cmd_vel',
                    'mission_complete_topic': f'/{v_name}/mission_complete',
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
            actions.append(LogInfo(
                msg=f'[ugv_controller_{v_name}] パラメータファイル: '
                    + ugv_param_file + "\n" + "\n".join(preview_lines)
            ))
        except Exception as e:
            actions.append(LogInfo(
                msg=f'[ugv_controller_{v_name}] パラメータファイル読み込み失敗: {e}'
            ))

        ugv_node = TimerAction(
            period=ugv_controller_delay,
            actions=[
                Node(
                    package='comms_sim_pkg',
                    executable='ugv_controller_node.py',
                    name=f'ugv_controller_{v_name}',
                    output='screen',
                    parameters=[ugv_param_file]
                )
            ]
        )
        actions.append(ugv_node)

    # =========================================================================
    # link_controller_node & sim_logger_node の起動
    # =========================================================================
    vehicle_names = [v.get('name', 'suv') for v in vehicles]
    link_ctrl_params = config.get('link_controller_node', {}).get('ros__parameters', {})
    
    if len(vehicle_names) > 0:
        link_controller_node = TimerAction(
            period=comms_node_delay,
            actions=[
                Node(
                    package='comms_sim_pkg',
                    executable='link_controller_node.py',
                    name='link_controller_node',
                    output='screen',
                    parameters=[
                        {
                            'vehicle_names': vehicle_names,
                            'scheduling_policy': str(link_ctrl_params.get('scheduling_policy', 'sequential')),
                            'time_slot_duration_s': float(link_ctrl_params.get('time_slot_duration_s', 10.0)),
                            'rssi_threshold': float(link_ctrl_params.get('rssi_threshold', -75.0)),
                            'use_sim_time': use_sim_time_bool
                        }
                    ]
                )
            ]
        )
        actions.append(link_controller_node)
        
        sim_logger_node_action = Node(
            package='comms_sim_pkg',
            executable='sim_logger_node.py',
            name='sim_logger_node',
            output='screen',
            parameters=[
                {
                    'vehicle_names': vehicle_names,
                    'output_dir': '/workspace/sim_results/',
                    'use_sim_time': use_sim_time_bool
                }
            ]
        )

        sim_logger_node = TimerAction(
            period=comms_node_delay + 1.0,
            actions=[sim_logger_node_action]
        )
        actions.append(sim_logger_node)
        
        # sim_logger_node がミッション完了を検知して終了したら、全体の launch をシャットダウンする
        shutdown_event = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=sim_logger_node_action,
                on_exit=[EmitEvent(event=Shutdown())]
            )
        )
        actions.append(shutdown_event)

    return actions


def generate_launch_description():
    """launchディスクリプションを生成する。"""

    # launch引数の宣言
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='シミュレーション時刻を使用するかどうか'
    )

    declare_headless = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Gazeboをヘッドレスモードで起動するかどうか'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_headless,
        OpaqueFunction(function=launch_setup),
    ])
