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
6. 各車両ごとに comms_simulator_node と tx_controller_node を起動する

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


def get_workspace_root() -> str:
    """
    シミュレーションのワークスペースルートを動的に特定する。
    """
    if os.path.exists('/workspace'):
        return '/workspace'
    
    current_dir = os.path.abspath(os.path.dirname(__file__))
    temp_dir = current_dir
    while True:
        if os.path.exists(os.path.join(temp_dir, '.git')) or os.path.exists(os.path.join(temp_dir, 'src')):
            return temp_dir
        parent = os.path.dirname(temp_dir)
        if parent == temp_dir:
            break
        temp_dir = parent
    return os.getcwd()


def resolve_path(raw_path: str, pkg_share: str) -> str:
    """
    パスパラメータ内の /workspace/ 等の絶対パスを、実行環境の package share や
    ローカルワークスペース内の適切な相対パスに動的にマッピング・解決する。
    """
    if not raw_path:
        return raw_path

    normalized = raw_path.replace('\\', '/')

    if '/workspace/' in normalized or normalized.startswith('workspace/'):
        parts = normalized.split('/workspace/', 1)
        if len(parts) < 2:
            parts = normalized.split('workspace/', 1)
        rel_path = parts[1]

        # config/ ディレクトリの解決
        if rel_path.startswith('config/'):
            basename = os.path.basename(rel_path)
            resolved = os.path.join(pkg_share, 'config', basename)
            if os.path.exists(resolved):
                return resolved

        # models/ ディレクトリの解決
        if rel_path.startswith('models/') or rel_path == 'models':
            resolved = os.path.join(pkg_share, rel_path)
            if os.path.exists(resolved):
                return resolved
            return os.path.join(pkg_share, 'models')

        # worlds/ または resource/ ワールドファイルの解決
        if 'minimal_world.sdf' in rel_path:
            resolved = os.path.join(pkg_share, 'worlds', 'minimal_world.sdf')
            if os.path.exists(resolved):
                return resolved

        if rel_path.startswith('comms_sim_pkg/resource/'):
            sub_path = rel_path.replace('comms_sim_pkg/resource/', 'worlds/', 1)
            resolved = os.path.join(pkg_share, sub_path)
            if os.path.exists(resolved):
                return resolved

    if os.path.isabs(raw_path) and os.path.exists(raw_path):
        return raw_path

    pkg_relative = os.path.join(pkg_share, raw_path)
    if os.path.exists(pkg_relative):
        return pkg_relative

    return raw_path

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
            
        # Support new scenario format directly from launch
        if config and 'scenario' in config:
            import sys
            workspace_root = get_workspace_root()
            tools_lib_path = os.path.join(workspace_root, 'tools', 'lib')
            if tools_lib_path not in sys.path:
                sys.path.insert(0, tools_lib_path)
            try:
                from scenario_loader import generate_sim_params
                config = generate_sim_params(config)
            except ImportError as e:
                print(f"Warning: Could not import scenario_loader for scenario resolution: {e}")

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


def _generate_vehicle_sdf(original_sdf_path: str, vehicle_name: str, plugin_xml: str = "", suffix: str = "") -> str:
    """
    車両固有のトピック名を持つSDFファイルを動的生成する。

    DiffDriveプラグインの <topic> と <odom_topic> タグを
    車両名付きのトピック名に書き換えた一時SDFファイルを生成する。

    Args:
        original_sdf_path: 元のモデルSDFファイルパス
        vehicle_name: 車両名（トピックプレフィックスに使用）
        plugin_xml: 追加するプラグインXML
        suffix: 一時ファイル名をユニークにするためのサフィックス

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

    if plugin_xml:
        # 最後の </model> の直前にプラグインXMLを挿入
        sdf_content = re.sub(
            r'(</model>)',
            f'{plugin_xml}\n\\1',
            sdf_content,
            count=1
        )

    # 一時ファイルに保存
    tmp_dir = os.path.join(tempfile.gettempdir(), 'comms_sim_vehicles')
    os.makedirs(tmp_dir, exist_ok=True)
    
    filename_suffix = f"_{suffix}" if suffix else ""
    tmp_sdf_path = os.path.join(tmp_dir, f'{vehicle_name}_model{filename_suffix}.sdf')

    with open(tmp_sdf_path, 'w', encoding='utf-8') as f:
        f.write(sdf_content)

    return tmp_sdf_path


def _build_vehicles_from_legacy(config: dict) -> list:
    """
    旧形式の設定（spawn_entities.suv + tx_controller_node）から
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

    tx_params = config.get('tx_controller_node', {}).get('ros__parameters', {})

    vehicle = {
        'name': suv_cfg.get('name', 'suv'),
        'model_uri': suv_cfg.get('model_uri', 'models://SUV'),
        'pose': suv_cfg.get('pose', [-60.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        'antenna_offset': suv_cfg.get('antenna_offset', [0.0, 0.0, 2.23]),
        'antenna_relative_rpy': suv_cfg.get('antenna_relative_rpy', [0.0, 0.0, 0.0]),
        'waypoints': tx_params.get('waypoints', []),
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
    config_file_arg = LaunchConfiguration('config_file').perform(context)
    use_sim_time_bool = (str(use_sim_time).lower() == 'true')
    headless_arg_bool = (str(headless_arg).lower() == 'true')

    # パス設定
    pkg_share = get_package_share_directory('comms_sim_pkg')
    config_path = config_file_arg

    # コンフィグ読み込み
    config = load_yaml_config(config_path)

    # YAMLからシミュレーション設定を取得
    default_world_path = os.path.join(pkg_share, 'worlds', 'minimal_world.sdf')
    sim_config = config.get('simulation', {})
    world_file = resolve_path(sim_config.get('world_file', default_world_path), pkg_share)
    output_subdir = str(sim_config.get('output_subdir', ''))

    if not os.path.exists(world_file):
        raise FileNotFoundError(
            f"\n{'='*70}\n"
            f"エラー: ワールドファイルが見つかりません\n"
            f"{'='*70}\n"
            f"検索パス:\n"
            f"  - {world_file}\n"
            f"sim_params.yaml の 'simulation.world_file' を確認してください。\n"
            f"{'='*70}\n"
        )

    # Dynamic replacement of physics_max_step_size from sim_params.yaml
    physics_max_step_size = sim_config.get('physics_max_step_size', None)
    # 姿勢記録: 通信計算は運動に影響しないので、軌跡を残しておけば通信と
    # スケジューリングは後から単一プロセスで再計算できる (手法ごとに Gazebo を
    # 回す必要がなくなり、非同期由来の非決定性も消える)
    record_poses_path = str(sim_config.get('record_poses_path', '') or '')
    record_poses_period = float(sim_config.get('record_poses_period_s', 0.005))
    if physics_max_step_size is not None or record_poses_path:
        try:
            with open(world_file, 'r', encoding='utf-8') as f:
                world_content = f.read()

            if physics_max_step_size is not None:
                physics_max_step_size_val = float(physics_max_step_size)
                world_content = re.sub(
                    r'<max_step_size>\s*[0-9.eE+-]+\s*</max_step_size>',
                    f'<max_step_size>{physics_max_step_size_val}</max_step_size>',
                    world_content
                )

            if record_poses_path:
                os.makedirs(os.path.dirname(os.path.abspath(record_poses_path)),
                            exist_ok=True)
                recorder_xml = (
                    '<plugin filename="PoseRecorderPlugin.so" '
                    'name="comms_sim::PoseRecorderPlugin">'
                    f'<output_path>{record_poses_path}</output_path>'
                    f'<period_s>{record_poses_period}</period_s>'
                    '</plugin>\n'
                )
                # </world> の直前に差し込む (ワールド直下のシステムとして動く)
                idx = world_content.rfind('</world>')
                if idx < 0:
                    raise ValueError('ワールドSDFに </world> がありません')
                world_content = (world_content[:idx] + recorder_xml
                                 + world_content[idx:])
                # 再生を自己完結させるため、実効設定 (交通生成後の車両定義を
                # 含む) を記録の隣に残す。スケジューラはアンテナ配置と道路形状を
                # 要るので、これが無いと事後再計算ができない
                try:
                    cfg_copy = os.path.join(
                        os.path.dirname(os.path.abspath(record_poses_path)),
                        'effective_sim_params.yaml')
                    with open(cfg_copy, 'w', encoding='utf-8') as cf:
                        yaml.safe_dump(config, cf, sort_keys=False,
                                       allow_unicode=True)
                    print(f"[sim_launch] Effective config saved -> {cfg_copy}")
                except Exception as e:
                    print(f"[sim_launch] Warning: effective config dump failed: {e}")
                print(f"[sim_launch] Pose recording enabled -> {record_poses_path}")

            # Save to temp file
            tmp_dir = os.path.join(tempfile.gettempdir(), 'comms_sim_worlds')
            os.makedirs(tmp_dir, exist_ok=True)
            config_suffix = os.path.splitext(os.path.basename(config_path))[0]
            tmp_world_path = os.path.join(tmp_dir, f'world_{config_suffix}.sdf')

            with open(tmp_world_path, 'w', encoding='utf-8') as f:
                f.write(world_content)

            if physics_max_step_size is not None:
                print(f"[sim_launch] Dynamic world generation: max_step_size set to {physics_max_step_size_val} s")
            world_file = tmp_world_path
        except Exception as e:
            print(f"[sim_launch] Warning: failed to dynamically update world max_step_size: {e}")

    world_name = sim_config.get('world_name', 'comms_sim_world')
    verbosity = sim_config.get('verbosity', 3)
    headless = sim_config.get('headless', False)
    if headless_arg and str(headless_arg).lower() != 'auto':
        # コマンドライン引数が明示されていればそれを最優先
        headless = (str(headless_arg).lower() == 'true')
    elif str(headless_arg).lower() == 'auto':
        if headless:
            # YAMLで headless: true が明示されていればそれを尊重する
            # （sweep_sim.py 等がプログラム的に設定した値を DISPLAY 検出で上書きしない）
            print("[sim_launch] YAML設定 headless: true を使用します（headlessモード）")
        else:
            # YAML が false（またはデフォルト）の場合のみ DISPLAY 自動検出を行う
            display_env = os.environ.get('DISPLAY', '')
            if not display_env:
                print("[sim_launch] DISPLAY未設定のため、自動的にheadlessモードに切り替えます")
                headless = True
            else:
                print(f"[sim_launch] DISPLAY={display_env} が設定されています。GUIモードで起動します")
                headless = False
    model_prefix = resolve_path(sim_config.get('model_path_prefix', '/workspace/models'), pkg_share)

    # YAMLからタイミング設定を取得
    timing = sim_config.get('timing', {})
    gazebo_startup_delay = timing.get('gazebo_startup_delay', 3.0)
    entity_spawn_interval = timing.get('entity_spawn_interval', 1.0)
    bridge_startup_delay = timing.get('bridge_startup_delay', 2.0)
    comms_node_delay = timing.get('comms_node_delay', 5.0)
    tx_controller_delay = timing.get('tx_controller_delay', 6.0)

    # YAMLからスポーンエンティティ設定を取得（基地局等）
    spawn_entities = config.get('spawn_entities', {})

    # =====================================================================
    # 車両リストの取得（後方互換対応）
    # =====================================================================
    vehicles = config.get('vehicles', None)
    if vehicles is None or not isinstance(vehicles, list) or len(vehicles) == 0:
        # 旧形式フォールバック: spawn_entities.suv + tx_controller_node から生成
        vehicles = _build_vehicles_from_legacy(config)
        if vehicles:
            # 旧形式の suv を spawn_entities から除外（車両は vehicles で処理する）
            spawn_entities = {k: v for k, v in spawn_entities.items()
                              if k.lower() != 'suv'}

    # TXコントローラ共通パラメータ
    tx_common = config.get('tx_controller_common', {})
    # 旧形式フォールバック
    if not tx_common:
        tx_common = config.get('tx_controller_node', {}).get('ros__parameters', {})
    waypoint_tolerance = float(tx_common.get('waypoint_tolerance', 2.0))
    control_rate = float(tx_common.get('control_rate', 10.0))
    max_angular_velocity = float(tx_common.get('max_angular_velocity', 1.0))
    heading_gain = float(tx_common.get('heading_gain', 1.5))
    max_acceleration = float(tx_common.get('max_acceleration', 0.5))

    actions = []

    # =========================================================================
    # Gazeboシミュレーション
    # =========================================================================
    # GPU利用可否の判定
    has_nvidia = (
        os.path.exists('/dev/dri') or
        os.environ.get('NVIDIA_VISIBLE_DEVICES', '') not in ('', 'void')
    )

    # X11フォワーディング（リモート接続）かどうかを判定
    # DISPLAY が "localhost:N.M" や "hostname:N.M" 形式 → X11フォワーディング
    display_env = os.environ.get('DISPLAY', '')
    is_x11_forwarding = ':' in display_env and (
        '.' in display_env or          # localhost:10.0 形式
        display_env.startswith('localhost:') or
        (not display_env.startswith(':'))  # リモートホスト付き
    )

    # 基本の環境変数（GPU/CPU 共通）
    gz_env = os.environ.copy()
    gz_env['GZ_SIM_RESOURCE_PATH'] = model_prefix
    plugin_path = os.path.join(get_workspace_root(), 'install', 'comms_sim_pkg', 'lib', 'comms_sim_pkg', 'plugins')
    if 'GZ_SIM_SYSTEM_PLUGIN_PATH' in gz_env:
        gz_env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = f"{plugin_path}:{gz_env['GZ_SIM_SYSTEM_PLUGIN_PATH']}"
    else:
        gz_env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = plugin_path

    if headless:
        # ヘッドレスモード（GUIなし・レンダリングなし）
        # 物理エンジン(DART)はCPUベースのため、headlessではGPUは使用されない
        gz_cmd = ['gz', 'sim', '-s', '-v', str(verbosity), '-r', world_file]
        if has_nvidia:
            gz_env.update({
                'LIBGL_ALWAYS_SOFTWARE': '0',
                '__NV_PRIME_RENDER_OFFLOAD': '1',
                '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
                '__EGL_VENDOR_LIBRARY_FILENAMES': '/usr/share/glvnd/egl_vendor.d/10_nvidia.json',
            })
            print("[sim_launch] headless: GPUあり（レンダリングなしのためCPU物理演算）")
        else:
            print("[sim_launch] headless: CPU モードで実行します")
    elif has_nvidia and not is_x11_forwarding:
        # GUIモード + ローカルGPU接続: ogre2 でハードウェアレンダリング
        gz_cmd = ['gz', 'sim', '-v', str(verbosity), '-r', '--render-engine', 'ogre2', world_file]
        gz_env.update({
            'LIBGL_ALWAYS_SOFTWARE': '0',
            '__NV_PRIME_RENDER_OFFLOAD': '1',
            '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
        })
        print("[sim_launch] GUI: GPU (ogre2) レンダリングモードで起動します")
    else:
        # GUIモード + X11フォワーディング or GPU なし: ogre で CPU レンダリング
        gz_cmd = ['gz', 'sim', '-v', str(verbosity), '-r', '--render-engine', 'ogre', world_file]
        gz_env['LIBGL_ALWAYS_SOFTWARE'] = '1'
        if is_x11_forwarding:
            print("[sim_launch] GUI: X11フォワーディング経由 → CPU (ogre) レンダリングで起動します")
        else:
            print("[sim_launch] GUI: GPU未検出 → CPU (ogre) レンダリングで起動します")

    gz_sim = ExecuteProcess(
        cmd=gz_cmd,
        name='gazebo',
        output='screen',
        additional_env=gz_env
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

        # --- プラグインXMLの動的生成 ---
        waypoints_raw = vehicle_cfg.get('waypoints', [])
        waypoints_param = []
        if waypoints_raw:
            if isinstance(waypoints_raw[0], (list, tuple)):
                for waypoint in waypoints_raw:
                    waypoints_param.extend([float(v) for v in waypoint])
            else:
                waypoints_param = [float(v) for v in waypoints_raw]
        waypoints_str = " ".join(map(str, waypoints_param))

        is_shinkansen = 'shinkansen' in v_name.lower() or 'shinkansen' in original_sdf_path.lower()
        
        plugin_xml = f"""
        <plugin filename="TxControllerPlugin.so" name="tx_controller::TxControllerPlugin">
          <waypoints>{waypoints_str}</waypoints>
          <waypoint_tolerance>{waypoint_tolerance}</waypoint_tolerance>
          <heading_gain>{heading_gain}</heading_gain>
          <max_acceleration>{max_acceleration}</max_acceleration>
          <max_angular_velocity>{max_angular_velocity}</max_angular_velocity>
          <is_shinkansen>{"true" if is_shinkansen else "false"}</is_shinkansen>
          <mission_complete_topic>/{v_name}/mission_complete</mission_complete_topic>
          <ready_pub_topic>/tx_controller_{v_name}/ready</ready_pub_topic>
          <all_ready_topic>/sim/all_ready</all_ready_topic>
          <config_file_path>{config_path}</config_file_path>
        </plugin>
        """

        # 車両固有のSDFを生成（トピック名を書き換え + プラグイン追加）
        config_suffix = os.path.splitext(os.path.basename(config_path))[0]
        vehicle_sdf_path = _generate_vehicle_sdf(original_sdf_path, v_name, plugin_xml, suffix=config_suffix)

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
    # 制御プレーンノード: external_schedule + control_plane が
    # "kkf_mpc" (提案) / "ts_kf" (時系列KF) / "a3" (A3イベント型) の場合に起動
    # (sweep終了時のクリーンアップに巻き込まれないよう launch のライフサイクルに載せる)
    # =========================================================================
    link_params_for_cp = config.get('link_controller_node', {}).get('ros__parameters', {})
    if (link_params_for_cp.get('scheduling_policy') == 'external_schedule'
            and link_params_for_cp.get('control_plane') in ('kkf_mpc', 'ts_kf', 'a3',
                                                            'trend', 'oracle')):
        kkf_scheduler = TimerAction(
            period=gazebo_startup_delay,
            actions=[
                Node(
                    package='comms_sim_pkg',
                    executable='kkf_scheduler_node.py',
                    name='kkf_scheduler',
                    output='screen',
                    arguments=['--config', str(config_path)],
                )
            ]
        )
        actions.append(kkf_scheduler)

    # =========================================================================
    # 遮蔽体スポーン: role=blocker のエンティティ
    # (静的ならそのままスポーン、waypoints があれば WaypointMoverPlugin を付与)
    # =========================================================================
    blocker_entities = config.get('blocker_entities', {})
    for b_key, b_cfg in blocker_entities.items():
        b_name = b_cfg.get('name', b_key)
        b_pose = b_cfg.get('pose', [0, 0, 0, 0, 0, 0])
        if not isinstance(b_pose, list) or len(b_pose) != 6:
            raise ValueError(f"遮蔽体 '{b_key}' のポーズが不正: {b_pose}")

        b_model_path = _resolve_model_uri(b_cfg.get('model_uri', ''), model_prefix)
        b_waypoints_raw = b_cfg.get('waypoints', [])

        if b_waypoints_raw:
            b_waypoints_param = []
            for waypoint in b_waypoints_raw:
                b_waypoints_param.extend([float(v) for v in waypoint])
            mover_xml = f"""
        <plugin filename="WaypointMoverPlugin.so" name="tx_controller::WaypointMoverPlugin">
          <waypoints>{" ".join(map(str, b_waypoints_param))}</waypoints>
          <loop>{"true" if b_cfg.get('loop', False) else "false"}</loop>
          <all_ready_topic>/sim/all_ready</all_ready_topic>
        </plugin>
        """
            config_suffix = os.path.splitext(os.path.basename(config_path))[0]
            b_model_path = _generate_vehicle_sdf(b_model_path, b_name, mover_xml, suffix=config_suffix)

        spawn_blocker = TimerAction(
            period=spawn_delay,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    name=f'spawn_{b_name}',
                    output='screen',
                    arguments=[
                        '-world', str(world_name),
                        '-file', str(b_model_path),
                        '-name', str(b_name),
                        '-x', str(b_pose[0]), '-y', str(b_pose[1]), '-z', str(b_pose[2]),
                        '-R', str(b_pose[3]), '-P', str(b_pose[4]), '-Y', str(b_pose[5]),
                    ]
                )
            ]
        )
        actions.append(spawn_blocker)
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
        
        # C++プラグインからの完了通知
        bridge_topic.append(
            f"/{v_name}/mission_complete@std_msgs/msg/Bool[gz.msgs.Boolean"
        )
        
        # C++プラグインからのReadyシグナル
        bridge_topic.append(
            f"/tx_controller_{v_name}/ready@std_msgs/msg/Bool[gz.msgs.Boolean"
        )

        # C++プラグインからの進捗シグナル
        bridge_topic.append(
            f"/{v_name}/mission_progress@std_msgs/msg/Float64[gz.msgs.Double"
        )
        
        # 全体Readyシグナル (ROS → Gazebo)
        bridge_topic.append(
            f"/sim/all_ready@std_msgs/msg/Bool]gz.msgs.Boolean"
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
    # 複数基地局の対応
    bs_positions = []
    bs_antenna_offsets = []
    bs_rpys = []
    bs_relative_rpys = []

    if isinstance(spawn_entities, dict):
        # すべてのエンティティを基地局として取得 (role=rx のものが入っている前提)
        antenna_keys = sorted(list(spawn_entities.keys()))
        for key in antenna_keys:
            cfg = spawn_entities[key]
            if isinstance(cfg, dict):
                # position & rpy
                pose = cfg.get('pose', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                if isinstance(pose, list) and len(pose) >= 3:
                    pos = [float(pose[0]), float(pose[1]), float(pose[2])]
                else:
                    pos = [0.0, 0.0, 0.0]
                if isinstance(pose, list) and len(pose) >= 6:
                    rpy = [float(pose[3]), float(pose[4]), float(pose[5])]
                else:
                    rpy = [0.0, 0.0, 0.0]
                
                # offset
                offset_raw = cfg.get('antenna_offset')
                if isinstance(offset_raw, list) and len(offset_raw) >= 3:
                    offset = [float(v) for v in offset_raw[:3]]
                else:
                    h = float(cfg.get('antenna_height_offset', 3.0))
                    offset = [0.0, 0.0, h]
                
                # relative_rpy
                rel_rpy_raw = cfg.get('antenna_relative_rpy')
                if isinstance(rel_rpy_raw, list) and len(rel_rpy_raw) >= 3:
                    rel_rpy = [float(r) for r in rel_rpy_raw[:3]]
                else:
                    rel_rpy = [0.0, 0.0, 0.0]

                bs_positions.extend(pos)
                bs_antenna_offsets.extend(offset)
                bs_rpys.extend(rpy)
                bs_relative_rpys.extend(rel_rpy)

    # 従来の単一パラメータ用（下位互換性のため、1番目のアンテナを設定）
    if bs_positions:
        antenna_base_position = bs_positions[:3]
        antenna_antenna_offset = bs_antenna_offsets[:3]
        antenna_relative_rpy = bs_relative_rpys[:3]
        antenna_base_rpy = bs_rpys[:3]
    else:
        antenna_base_position = [0.0, 0.0, 0.0]
        antenna_antenna_offset = [0.0, 0.0, 3.0]
        antenna_relative_rpy = [0.0, 0.0, 0.0]
        antenna_base_rpy = [0.0, 0.0, 0.0]

    # =========================================================================
    # 各車両のアンテナ設定の処理 (必要に応じて将来利用)
    # =========================================================================
    for vehicle_cfg in vehicles:
        v_name = vehicle_cfg.get('name', 'suv')
        
        # C++ Gazebo Plugin (TxControllerPlugin.cc) への移行が完了したため、
        # 旧来のPython版通信ノードおよびコントローラノードは廃止・スキップされています。

        # 進捗を標準出力へフラッシュするためのアダプターノード
        progress_logger = Node(
            package='comms_sim_pkg',
            executable='progress_logger_node.py',
            name=f'progress_logger_{v_name}',
            output='screen',
            parameters=[{'vehicle_name': v_name}]
        )
        actions.append(progress_logger)

    # =========================================================================
    # Ready ゲートノード（全ノードの起動同期）
    # =========================================================================
    # C++プラグイン(TxControllerPlugin)からのReadyシグナルを待機する
    if len(vehicles) > 0:
        expected_ready_nodes = []
        for v in vehicles:
            v_name = v.get('name', 'suv')
            expected_ready_nodes.append(f'tx_controller_{v_name}')

        ready_gate_node = TimerAction(
            period=comms_node_delay,
            actions=[
                Node(
                    package='comms_sim_pkg',
                    executable='sim_ready_gate_node.py',
                    name='sim_ready_gate',
                    output='screen',
                    parameters=[{
                        'expected_nodes': expected_ready_nodes,
                        'use_sim_time': False
                    }]
                )
            ]
        )
        actions.append(ready_gate_node)

    # =========================================================================
    # Mission Coordinator（シミュレーション全体の完了・自動終了制御）
    # =========================================================================
    if len(vehicles) > 0:
        vehicle_names_list = [v.get('name', 'suv') for v in vehicles]
        mission_coordinator_node = Node(
            package='comms_sim_pkg',
            executable='mission_coordinator_node.py',
            name='mission_coordinator_node',
            output='screen',
            parameters=[{'vehicle_names': vehicle_names_list,
                         # シム時刻の上限。全車の完走を待つだけだと、1台でも
                         # 完走できない車両があると走行が終わらない
                         'max_sim_time_s': float(
                             sim_config.get('max_sim_time_s', 0.0))}]
        )
        actions.append(mission_coordinator_node)

        # 全車両がミッション完了を報告しコーディネーターが終了したら、
        # シミュレーションシステム全体（Launch）をシャットダウンして終了する
        shutdown_event = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=mission_coordinator_node,
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
        default_value='auto',
        description='Gazeboをヘッドレスモードで起動するかどうか (true/false/auto: yamlに従う)'
    )

    ws_root = get_workspace_root()
    ws_config = os.path.join(ws_root, 'src', 'comms_sim_pkg', 'config', 'sim_params.yaml')
    if os.path.exists(ws_config):
        default_config = ws_config
    else:
        try:
            default_config = os.path.join(get_package_share_directory('comms_sim_pkg'), 'config', 'sim_params.yaml')
        except Exception:
            default_config = ws_config

    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='シミュレーション設定ファイルのパス'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_headless,
        declare_config_file,
        OpaqueFunction(function=launch_setup),
    ])
