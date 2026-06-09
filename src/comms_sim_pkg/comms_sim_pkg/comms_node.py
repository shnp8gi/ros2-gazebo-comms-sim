#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
# 通信シミュレータノード
# TX-基地局間の無線通信品質をシミュレーションするROS 2ノード
# =============================================================================
"""
TXと基地局間の通信品質をシミュレーションするROS 2ノード。

サブスクライブトピック:
    - /imu/data (sensor_msgs/Imu): TXの姿勢情報
    - /odom (nav_msgs/Odometry): TXのローカル位置
    - /rx/pose (geometry_msgs/PoseStamped): 基地局の姿勢（オプション）

パブリッシュトピック:
    - /comms/quality (comms_sim_msgs/CommsQuality): 通信メトリクス

パラメータ:
    - sampling_rate: 計算周波数 [Hz]
    - noise_variance: AWGN分散 [dB]
    - e_plane_path: E面アンテナパターンCSVパス
    - h_plane_path: H面アンテナパターンCSVパス
    - comm_data_limit_mb: データ伝送上限 [MB]
    - path_loss.*: パスロスモデルパラメータ
    - tx_power: 送信電力 [dBm]
    - odom_topic: オドメトリトピック名
    - mission_complete_topic: ミッション完了トピック名
    - rx_pose_topic: 基地局姿勢トピック名
"""

from typing import Optional, List, Tuple
from enum import Enum

import os
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Header
from std_msgs.msg import Bool, Float64

# ros2 launch経由で実行可能なように絶対インポートを優先
try:
    from comms_sim_pkg.antenna_parser import AntennaPatternParser
    from comms_sim_pkg.comms_calculator import (
        CommsCalculator,
        LogDistancePathLossModel,
        TwoRayGroundModel,
    )
except ImportError:
    # python -m comms_sim_pkg.comms_node 実行時のフォールバック
    from .antenna_parser import AntennaPatternParser  # type: ignore
    from .comms_calculator import (  # type: ignore
        CommsCalculator,
        LogDistancePathLossModel,
        TwoRayGroundModel,
    )

# カスタムメッセージのインポート（ビルド後に利用可能）
try:
    from comms_sim_msgs.msg import CommsQuality
except ImportError:
    CommsQuality = None




class LinkState(Enum):
    """通信リンク状態の列挙型。"""
    DISCONNECTED = 0      # RSSIが閾値未満（切断）
    ESTABLISHING = 1      # リンク確立処理中
    CONNECTED = 2         # リンク確立済み（通信可能）


class CommsSimulatorNode(Node):
    """
    通信シミュレーション用ROS 2ノード。

    TXの位置・姿勢と基地局の配置に基づき、
    設定可能な伝搬路モデルを用いてRSSIとスループットを計算する。

    リンク確立遅延の実装:
    - RSSIが最低閾値を超えた時点でリンク確立待機開始
    - 確立完了後にデータ伝送開始
    """

    def __init__(self) -> None:
        super().__init__('comms_simulator_node')

        # =====================================================================
        # パラメータ宣言
        # =====================================================================
        self.declare_parameter('sampling_rate', 1.0)
        self.declare_parameter('noise_variance', 2.0)
        self.declare_parameter('e_plane_path', '')
        self.declare_parameter('h_plane_path', '')
        self.declare_parameter('comm_data_limit_mb', -1.0)
        self.declare_parameter('path_loss.c', 299792458.0)
        self.declare_parameter('path_loss.frequency', 6.0e10)
        self.declare_parameter('path_loss.exponent', 2.0)
        self.declare_parameter('path_loss.d0', 1.0)
        self.declare_parameter('path_loss.pl_d0', -1.0)
        self.declare_parameter('tx_power', -7.0)
        self.declare_parameter('rx_position', [0.0, 0.0, 0.0])
        self.declare_parameter('rx_antenna_offset', [0.0, 0.0, 10.5])
        self.declare_parameter('tx_antenna_offset', [0.0, 0.0, 1.3])
        self.declare_parameter('mcs_table_path', '')
        self.declare_parameter('link_establishment_time_ms', 2.0)
        self.declare_parameter('mission_complete_topic', '/mission_complete')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('tx_spawn_pose', [-20.0, 0.0, 0.0])
        self.declare_parameter('tx_antenna_relative_rpy', [0.0, 0.0, 0.0])
        self.declare_parameter('rx_antenna_relative_rpy', [0.0, 0.0, 0.0])
        self.declare_parameter('rx_pose_topic', '/rx/pose')
        self.declare_parameter('rx_rpy', [0.0, 0.0, 0.0])
        self.declare_parameter('rx_positions', [0.0, 0.0, 0.0])
        self.declare_parameter('rx_antenna_offsets', [0.0, 0.0, 10.5])
        self.declare_parameter('rx_rpys', [0.0, 0.0, 0.0])
        self.declare_parameter('rx_antenna_relative_rpys', [0.0, 0.0, 0.0])
        self.declare_parameter('logging_start_trigger', 'on_movement')
        self.declare_parameter('logging_start_topic', '/logging/start')
        self.declare_parameter('max_antenna_attenuation', 30.0)
        self.declare_parameter('vehicle_name', '')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        # パラメータ取得
        self.sampling_rate = self.get_parameter('sampling_rate').value * 1000.0
        self.noise_variance = self.get_parameter('noise_variance').value
        self.e_plane_path = self.get_parameter('e_plane_path').value
        self.h_plane_path = self.get_parameter('h_plane_path').value
        self.comm_data_limit_mb = self.get_parameter('comm_data_limit_mb').value
        self.tx_power = self.get_parameter('tx_power').value
        self.mcs_table_path = self.get_parameter('mcs_table_path').value
        self.link_establishment_time = self.get_parameter('link_establishment_time_ms').value / 1000.0  # ミリ秒→秒
        self._mission_complete_topic: str = str(self.get_parameter('mission_complete_topic').value)
        self.odom_topic: str = str(self.get_parameter('odom_topic').value)
        self.tx_spawn_pose: List[float] = list(self.get_parameter('tx_spawn_pose').value)
        self.tx_antenna_relative_rpy: List[float] = list(self.get_parameter('tx_antenna_relative_rpy').value)
        self.rx_antenna_relative_rpy: List[float] = list(self.get_parameter('rx_antenna_relative_rpy').value)
        self.rx_pose_topic: str = str(self.get_parameter('rx_pose_topic').value)
        self._saved_on_mission_complete: bool = False
        self.vehicle_name: str = str(self.get_parameter('vehicle_name').value)
        self.cmd_vel_topic: str = str(self.get_parameter('cmd_vel_topic').value)

        # ログ記録開始トリガー設定
        self._logging_trigger: str = str(self.get_parameter('logging_start_trigger').value).lower()
        self._logging_start_topic: str = str(self.get_parameter('logging_start_topic').value)
        if self._logging_trigger not in ('immediate', 'on_movement', 'on_topic'):
            self.get_logger().warn(
                f'不明なlogging_start_trigger: "{self._logging_trigger}"。'
                f'"immediate" を使用します。'
            )
            self._logging_trigger = 'immediate'

        # パスロスモデルパラメータ
        c = self.get_parameter('path_loss.c').value
        frequency = self.get_parameter('path_loss.frequency').value
        exponent = self.get_parameter('path_loss.exponent').value
        d0 = self.get_parameter('path_loss.d0').value
        pl_d0 = self.get_parameter('path_loss.pl_d0').value

        # 基地局エンティティの姿勢 (roll, pitch, yaw) [rad]
        self.rx_entity_rpy: np.ndarray = np.array(self.get_parameter('rx_rpy').value, dtype=float)

        # =====================================================================
        # コンポーネント初期化
        # =====================================================================
        # 伝搬路モデル（Strategyパターン）
        self.propagation_model = LogDistancePathLossModel(
            c=c, frequency=frequency, exponent=exponent, d0=d0, pl_d0=pl_d0
        )

        # 通信品質計算クラス
        self.comms_calculator = CommsCalculator(
            propagation_model=self.propagation_model,
            tx_power_dbm=self.tx_power,
            noise_variance=self.noise_variance,
            mcs_table_path=self.mcs_table_path
        )

        # MCSテーブルからRSSI閾値を取得（最低RSSI）
        self.rssi_threshold = self.comms_calculator.rssi_min

        # アンテナパターン解析
        max_antenna_attenuation = self.get_parameter('max_antenna_attenuation').value
        self.antenna_parser = AntennaPatternParser(
            max_antenna_attenuation=float(max_antenna_attenuation)
        )
        if self.e_plane_path:
            try:
                self.antenna_parser.load_e_plane(self.e_plane_path)
                self.get_logger().info(f'E面パターン読み込み完了: {self.e_plane_path}')
            except Exception as e:
                self.get_logger().warn(f'E面パターン読み込み失敗: {e}')

        if self.h_plane_path:
            try:
                self.antenna_parser.load_h_plane(self.h_plane_path)
                self.get_logger().info(f'H面パターン読み込み完了: {self.h_plane_path}')
            except Exception as e:
                self.get_logger().warn(f'H面パターン読み込み失敗: {e}')

        # =====================================================================
        # 状態変数
        # =====================================================================
        # 複数基地局のパラメータ取得
        bs_positions_raw = list(self.get_parameter('rx_positions').value)
        bs_offsets_raw = list(self.get_parameter('rx_antenna_offsets').value)
        bs_rpys_raw = list(self.get_parameter('rx_rpys').value)
        bs_relative_rpys_raw = list(self.get_parameter('rx_antenna_relative_rpys').value)

        self.rx_nodes = []
        if len(bs_positions_raw) >= 3 and len(bs_positions_raw) % 3 == 0:
            n_bs = len(bs_positions_raw) // 3
            for i in range(n_bs):
                pos = np.array(bs_positions_raw[i*3:(i+1)*3], dtype=float)
                offset = np.array(bs_offsets_raw[i*3:(i+1)*3], dtype=float) if len(bs_offsets_raw) >= (i+1)*3 else np.array([0.0, 0.0, 3.0], dtype=float)
                rpy = np.array(bs_rpys_raw[i*3:(i+1)*3], dtype=float) if len(bs_rpys_raw) >= (i+1)*3 else np.zeros(3, dtype=float)
                rel_rpy = np.array(bs_relative_rpys_raw[i*3:(i+1)*3], dtype=float) if len(bs_relative_rpys_raw) >= (i+1)*3 else np.zeros(3, dtype=float)
                
                # Precalculate antenna rotation matrix
                ant_rpy = rpy + rel_rpy
                rotmat = self.antenna_parser._rpy_to_rotmat(float(ant_rpy[0]), float(ant_rpy[1]), float(ant_rpy[2]))
                
                self.rx_nodes.append({
                    'position': pos,
                    'antenna_offset': offset,
                    'rpy': rpy,
                    'antenna_relative_rpy': rel_rpy,
                    'name': f'antenna_{i}',
                    'rotmat': rotmat
                })
        else:
            bs_pos = self.get_parameter('rx_position').value
            pos = np.array(bs_pos, dtype=float) if bs_pos else np.zeros(3, dtype=float)
            offset = np.array(self.get_parameter('rx_antenna_offset').value, dtype=float)
            rpy = np.array(self.get_parameter('rx_rpy').value, dtype=float)
            rel_rpy = np.array(self.get_parameter('rx_antenna_relative_rpy').value, dtype=float)
            
            ant_rpy = rpy + rel_rpy
            rotmat = self.antenna_parser._rpy_to_rotmat(float(ant_rpy[0]), float(ant_rpy[1]), float(ant_rpy[2]))
            
            self.rx_nodes.append({
                'position': pos,
                'antenna_offset': offset,
                'rpy': rpy,
                'antenna_relative_rpy': rel_rpy,
                'name': 'antenna_0',
                'rotmat': rotmat
            })

        self.rx_position: Optional[np.ndarray] = self.rx_nodes[0]['position']
        self.rx_antenna_offset: np.ndarray = self.rx_nodes[0]['antenna_offset']
        self.tx_antenna_offset: np.ndarray = np.array(self.get_parameter('tx_antenna_offset').value, dtype=float)

        self.tx_orientation: Optional[np.ndarray] = None  # [roll, pitch, yaw]
        self.tx_local_position: Optional[np.ndarray] = None
        self._odom_offset_set: bool = False
        self._odom_offset: np.ndarray = np.zeros(3, dtype=float)
        self._initial_position_verified: bool = False
        self._verification_attempt_count: int = 0

        self.total_data_transmitted: float = 0.0  # 累計伝送データ量 [MB]
        self.comm_active: bool = True
        self.simulation_start_time: Optional[float] = None
        self._last_calc_sim_time: Optional[float] = None

        # ログ記録準備完了フラグ（トリガー条件が満たされたらTrue）
        self._logging_ready: bool = (self._logging_trigger == 'immediate')
        # 排他制御（Link Grant）状態
        self.has_link_grant: bool = False

        # リンクグラントのローカル判定用
        self.vehicle_antennas: list = []
        self.scheduling_policy: str = 'sequential'

        # リンク確立状態管理
        self.link_state: LinkState = LinkState.DISCONNECTED
        self.link_establishment_start_time: Optional[float] = None
        self._last_rssi: Optional[float] = None  

        # =====================================================================
        # QoSプロファイル
        # =====================================================================
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # =====================================================================
        # サブスクライバ
        # =====================================================================
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            sensor_qos
        )

        # /cmd_vel サブスクライバ（on_movementモード時のみ）
        if self._logging_trigger == 'on_movement':
            self._cmd_vel_sub = self.create_subscription(
                Twist,
                self.cmd_vel_topic,
                self._on_cmd_vel,
                sensor_qos
            )

        # ログ開始トピック サブスクライバ（on_topicモード時のみ）
        if self._logging_trigger == 'on_topic':
            self._logging_start_sub = self.create_subscription(
                Bool,
                self._logging_start_topic,
                self._on_logging_start_topic,
                10
            )

        # /odom でローカル座標（Gazebo内）を取得（座標系の混在を避ける）
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            sensor_qos
        )

        # TXコントローラからのミッション完了通知
        self.mission_complete_sub = self.create_subscription(
            Bool,
            self._mission_complete_topic,
            self._on_mission_complete,
            10
        )

        # リンクグラント受信
        if self.vehicle_name:
            self.link_grant_sub = self.create_subscription(
                Bool,
                f'/{self.vehicle_name}/link_grant',
                self._on_link_grant,
                10
            )

        # 基地局姿勢サブスクライバ（オプション: 基地局の回転対応）
        self._bs_pose_sub = self.create_subscription(
            PoseStamped,
            self.rx_pose_topic,
            self._on_rx_pose,
            10,
        )

        # =====================================================================
        # パブリッシャ
        # =====================================================================
        if self.vehicle_name:
            quality_topic = f'/{self.vehicle_name}/comms/quality'
            self.link_request_pub = self.create_publisher(Float64, f'/{self.vehicle_name}/link_request', 10)
        else:
            quality_topic = '/comms/quality'
            self.link_request_pub = None

        if CommsQuality is not None:
            self.quality_pub = self.create_publisher(
                CommsQuality,
                quality_topic,
                10
            )
        else:
            self.quality_pub = None
            self.get_logger().warn('CommsQualityメッセージが利用できません')

        # =====================================================================
        # 定期計算タイマー
        # =====================================================================
        period = 1.0 / self.sampling_rate
        # YAMLからこのアンテナに属する車両のウェイポイントをロード
        self.waypoints: List[List[float]] = []
        try:
            import yaml
            from ament_index_python.packages import get_package_share_directory
            def get_workspace_root() -> str:
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

            yaml_path = ''
            try:
                pkg_share = get_package_share_directory('comms_sim_pkg')
                yaml_path = os.path.join(pkg_share, 'config', 'sim_params.yaml')
            except Exception:
                pass

            if not yaml_path or not os.path.exists(yaml_path):
                yaml_path = os.path.join(get_workspace_root(), 'src', 'comms_sim_pkg', 'config', 'sim_params.yaml')
            if os.path.exists(yaml_path):
                with open(yaml_path, 'r', encoding='utf-8') as f:
                    config = yaml.safe_load(f)
                    vehicles_cfg = config.get('vehicles', [])
                    for v in vehicles_cfg:
                        is_match = False
                        if v.get('name') == self.vehicle_name:
                            is_match = True
                        elif 'antennas' in v:
                            for a in v['antennas']:
                                if a.get('name') == self.vehicle_name:
                                    is_match = True
                                    break
                        if is_match:
                            wps_raw = v.get('waypoints', [])
                            if wps_raw:
                                if isinstance(wps_raw[0], (list, tuple)):
                                    self.waypoints = [list(wp) for wp in wps_raw]
                                else:
                                    self.waypoints = [wps_raw[i:i+4] for i in range(0, len(wps_raw), 4)]
                            self.vehicle_antennas = v.get('antennas', [])
                            lc_params = config.get('link_controller_node', {}).get('ros__parameters', {})
                            self.scheduling_policy = lc_params.get('scheduling_policy', 'sequential')
                            self.get_logger().info(f'[{self.vehicle_name}] ロードされたウェイポイント数: {len(self.waypoints)}, アンテナ数: {len(self.vehicle_antennas)}, ポリシー: {self.scheduling_policy}')
                            break
        except Exception as e:
            self.get_logger().warn(f'[{self.vehicle_name}] ウェイポイントのロード失敗: {e}')

        # self.timer_cb_group = MutuallyExclusiveCallbackGroup()
        # self.calc_timer = self.create_timer(
        #     period, 
        #     self.calculate_and_publish,
        #     callback_group=self.timer_cb_group
        # )

        self.get_logger().info(
            f'CommsSimulatorNode 初期化完了\n'
            f'  サンプリングレート: {self.sampling_rate} Hz\n'
            f'  送信電力: {self.tx_power} dBm\n'
            f'  雑音分散: {self.noise_variance} dB\n'
            f'  リンク確立時間: {self.link_establishment_time*1000:.1f} ms\n'
            f'  RSSI閾値: {self.rssi_threshold:.1f} dBm (MCSテーブルから)\n'
            f'  RSSI最小: {self.comms_calculator.rssi_min:.1f} dBm\n'
            f'  RSSI最大: {self.comms_calculator.rssi_max:.1f} dBm\n'
            f'  データ上限: {self.comm_data_limit_mb} MB\n'
            f'  モデル: {self.propagation_model.model_name}\n'
            f'  PL(d₀={self.propagation_model.d0}m): {self.propagation_model.pl_d0:.1f} dB, n={self.propagation_model.exponent}\n'
            f'  MCSテーブル: {self.mcs_table_path}\n'
            f'  ログ開始トリガー: {self._logging_trigger}\n'
            f'  車両名: {self.vehicle_name or "(未指定)"}\n'
            f'  odom: {self.odom_topic}\n'
            f'  cmd_vel: {self.cmd_vel_topic}\n'
            f'  mission_complete: {self._mission_complete_topic}'
        )

    def set_rx_position(
        self,
        position: List[float],
        antenna_offset: List[float]
    ) -> None:
        """
        ラウンチパラメータから基地局位置を設定する。

        Args:
            position: 基地局位置 [x, y, z]
            antenna_offset: モデル原点からのアンテナオフセット [x, y, z]
        """
        self.rx_position = np.array(position[:3])
        self.rx_antenna_offset = np.array(antenna_offset[:3], dtype=float)
        self.get_logger().info(
            f'基地局設定: {position}, アンテナオフセット: {antenna_offset}'
        )

    def set_tx_antenna_offset(self, offset: List[float]) -> None:
        """
        TXアンテナオフセットを設定する。

        Args:
            offset: TXモデル原点からのアンテナオフセット [x, y, z]
        """
        self.tx_antenna_offset = np.array(offset[:3], dtype=float)

    def odom_callback(self, msg: Odometry) -> None:
        """オドメトリ更新コールバック（ローカル位置 [m] として処理）。"""
        p = msg.pose.pose.position
        odom_pos = np.array([p.x, p.y, p.z], dtype=float)

        # 初回のオドメトリ受信時にオフセットを計算
        # 最初の受信位置がtx_spawn_poseに一致するようにする
        # 古いシミュレーションの残存メッセージを無視するため、スポーン位置付近（<= 10.0m）のメッセージのみ採用する
        if not self._odom_offset_set:
            if isinstance(self.tx_spawn_pose, (list, tuple)) and len(self.tx_spawn_pose) >= 3:
                spawn = np.array([
                    float(self.tx_spawn_pose[0]),
                    float(self.tx_spawn_pose[1]),
                    float(self.tx_spawn_pose[2]),
                ], dtype=float)
                dist_to_spawn = np.linalg.norm(odom_pos - spawn)
                if dist_to_spawn > 10.0:
                    self.get_logger().debug(
                        f'古いシミュレーションの残存オドメトリデータを検出 (スポーン位置からの距離 {dist_to_spawn:.1f}m)。無視します。'
                    )
                    return

                self._odom_offset = spawn - odom_pos
                self._odom_offset_set = True
                self.get_logger().info(
                    f'通信ノード odomオフセット計算完了: '
                    f'({self._odom_offset[0]:.3f}, {self._odom_offset[1]:.3f}, {self._odom_offset[2]:.3f})'
                )
            else:
                # 有効なスポーン位置がない場合は生のodomを使用
                self._odom_offset = np.zeros(3, dtype=float)
                self._odom_offset_set = True

        # オフセット適用済みの位置を保存（ワールド座標系）
        self.tx_local_position = odom_pos + self._odom_offset

        if 'shinkansen' in self.vehicle_name:
            # ウェイポイント間の直線上に完全に投影して固定する
            px, py, segment_yaw = self.get_current_segment_pose()
            if px is not None:
                self.tx_local_position[0] = px
                self.tx_local_position[1] = py

        # --- 初期位置の自己補正・検証システム ---
        if not getattr(self, '_initial_position_verified', True) and isinstance(self.tx_spawn_pose, (list, tuple)) and len(self.tx_spawn_pose) >= 3:
            expected = np.array([
                float(self.tx_spawn_pose[0]),
                float(self.tx_spawn_pose[1]),
                float(self.tx_spawn_pose[2]),
            ], dtype=float)
            error = np.linalg.norm((self.tx_local_position - expected)[:2])
            
            if error > 10.0:
                self._verification_attempt_count += 1
                self.get_logger().warn(
                    f'[通信ノード自己補正] 異常な初期位置を検出 (World: {self.tx_local_position[0]:.1f}, {self.tx_local_position[1]:.1f} '
                    f'| Spawn: {expected[0]:.1f}, {expected[1]:.1f})。オフセットを再計算します。'
                )
                self._odom_offset = expected - odom_pos
                self.tx_local_position = odom_pos + self._odom_offset
            elif error <= 0.5:
                self._initial_position_verified = True
                self.get_logger().info('通信ノード: 初期位置の検証が完了しました。')
            else:
                self._verification_attempt_count += 1
                if self._verification_attempt_count > 50:
                    self._initial_position_verified = True
                    self.get_logger().info('通信ノード: 初期位置の検証をタイムアウトで完了しました。')

        # シミュレーション開始時刻を記録
        if self.simulation_start_time is None:
            self.simulation_start_time = self.get_clock().now().nanoseconds / 1e9

        # オドメトリ更新に同期して通信品質の計算・パブリッシュを実行
        self.calculate_and_publish()

    def imu_callback(self, msg: Imu) -> None:
        """IMU姿勢コールバック。"""
        # クォータニオンをオイラー角に変換
        self.tx_orientation = self._quat_to_rpy(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )

        if 'shinkansen' in self.vehicle_name:
            px, py, segment_yaw = self.get_current_segment_pose()
            if segment_yaw is not None:
                self.tx_orientation = np.array([0.0, 0.0, segment_yaw], dtype=float)

    def get_current_segment_pose(self) -> tuple:
        """
        現在走行中のウェイポイント区間（直線）上の投影位置と方位を計算する。
        """
        if not self.waypoints or self.tx_local_position is None:
            return None, None, None

        # 現在位置
        ux = float(self.tx_local_position[0])
        uy = float(self.tx_local_position[1])

        # 現在位置に最も近い投影先セグメントを選択する
        best_px, best_py = ux, uy
        best_yaw = 0.0
        min_dist_sq = float('inf')

        # 前の地点 A
        ax = float(self.tx_spawn_pose[0])
        ay = float(self.tx_spawn_pose[1])

        for wp in self.waypoints:
            bx = float(wp[0])
            by = float(wp[1])

            # ベクトル AB
            vx = bx - ax
            vy = by - ay
            v_len_sq = vx*vx + vy*vy

            if v_len_sq < 1e-6:
                dist_sq = (ux - bx)**2 + (uy - by)**2
                if dist_sq < min_dist_sq:
                    min_dist_sq = dist_sq
                    best_px, best_py = bx, by
                    best_yaw = math.atan2(vy, vx)
            else:
                # 投影比率 t
                t = ((ux - ax) * vx + (uy - ay) * vy) / v_len_sq
                t = max(0.0, min(1.0, t))

                px = ax + t * vx
                py = ay + t * vy

                dist_sq = (ux - px)**2 + (uy - py)**2
                if dist_sq < min_dist_sq:
                    min_dist_sq = dist_sq
                    best_px, best_py = px, py
                    best_yaw = math.atan2(vy, vx)

            ax, ay = bx, by

        return best_px, best_py, best_yaw

    def set_tx_local_position(self, position: np.ndarray) -> None:
        """
        TXのローカル位置を直接設定する（オドメトリまたは直接ポーズ経由）。

        Args:
            position: ワールド座標系での位置 [x, y, z]
        """
        self.tx_local_position = position

    @staticmethod
    def _quat_to_rpy(x: float, y: float, z: float, w: float) -> np.ndarray:
        """クォータニオンから (roll, pitch, yaw) を計算する。"""
        # ロール
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = float(np.arctan2(sinr_cosp, cosr_cosp))

        # ピッチ
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = float(np.sign(sinp) * (np.pi / 2.0))
        else:
            pitch = float(np.arcsin(sinp))

        # ヨー
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = float(np.arctan2(siny_cosp, cosy_cosp))

        return np.array([roll, pitch, yaw], dtype=float)

    def _on_rx_pose(self, msg: PoseStamped) -> None:
        """基地局姿勢コールバック。"""
        q = msg.pose.orientation
        self.rx_entity_rpy = self._quat_to_rpy(float(q.x), float(q.y), float(q.z), float(q.w))

    def _on_cmd_vel(self, msg: Twist) -> None:
        """速度指令コールバック（on_movementトリガー用）。"""
        if not self._logging_ready:
            # 直進速度が非ゼロならTXが動き始めたと判定
            if abs(msg.linear.x) > 1e-3 or abs(msg.linear.y) > 1e-3:
                self._logging_ready = True
                self.get_logger().info(
                    f'移動検知。ログ記録を開始します (vx={msg.linear.x:.3f}, vy={msg.linear.y:.3f})'
                )

    def _on_logging_start_topic(self, msg: Bool) -> None:
        """外部トピックによるログ開始コールバック（on_topicトリガー用）。"""
        if msg.data and not self._logging_ready:
            self._logging_ready = True
            self.get_logger().info('外部トピックによりログ記録を開始します。')

    def _on_link_grant(self, msg: Bool) -> None:
        """リンク権付与通知コールバック"""
        self.has_link_grant = msg.data

    def _update_link_state(self, rssi: float, current_time: float, has_link_grant: Optional[bool] = None) -> bool:
        """
        RSSIとリンク権限(has_link_grant)に基づいてリンク確立状態を更新する。

        状態遷移:
        - DISCONNECTED → ESTABLISHING: RSSIが閾値を上回り、かつリンク権限がある
        - ESTABLISHING → CONNECTED: リンク確立時間が経過した
        - ESTABLISHING → DISCONNECTED: RSSIが閾値を下回った、またはリンク権限を失った
        - CONNECTED → DISCONNECTED: RSSIが閾値を下回った、またはリンク権限を失った

        Args:
            rssi: 現在のRSSI [dBm]
            current_time: 現在のシミュレーション時刻 [s]

        Returns:
            通信が許可される場合True、それ以外はFalse
        """
        grant = has_link_grant if has_link_grant is not None else self.has_link_grant
        if not grant or not self.comm_active:
            if self.link_state != LinkState.DISCONNECTED:
                self.link_state = LinkState.DISCONNECTED
                self.link_establishment_start_time = None
                if not self.comm_active:
                    self.get_logger().info('データ伝送終了のためリンクを切断しました')
            return False

        if self.link_state == LinkState.DISCONNECTED:
            # RSSIが閾値を超えたか確認
            if rssi > self.rssi_threshold:
                self.link_state = LinkState.ESTABLISHING
                self.link_establishment_start_time = current_time
                self.get_logger().info(
                    f'リンク確立開始 (RSSI: {rssi:.1f} dBm)'
                )
                # 即時確立チェック（シミュレーションのサンプリング周期が確立時間以上の場合は即時完了とする）
                if self.link_establishment_time <= (1.0 / self.sampling_rate):
                    self.link_state = LinkState.CONNECTED
                    self.get_logger().info(
                        f'リンク確立完了 (即時確立: {self.link_establishment_time*1000:.1f} ms)'
                    )
                    return True
            return False

        elif self.link_state == LinkState.ESTABLISHING:
            # RSSIが閾値を下回ったか確認
            if rssi <= self.rssi_threshold:
                self.link_state = LinkState.DISCONNECTED
                self.link_establishment_start_time = None
                self.get_logger().info('リンク確立中断 (RSSI低下)')
                return False

            # リンク確立時間が経過したか確認
            elapsed = current_time - self.link_establishment_start_time
            if elapsed >= self.link_establishment_time:
                self.link_state = LinkState.CONNECTED
                self.get_logger().info(
                    f'リンク確立完了 ({elapsed*1000:.1f} ms 経過)'
                )
                return True

            return False

        elif self.link_state == LinkState.CONNECTED:
            # RSSIが閾値を下回ったか確認
            if rssi <= self.rssi_threshold:
                self.link_state = LinkState.DISCONNECTED
                self.link_establishment_start_time = None
                self.get_logger().info('リンク切断 (RSSI低下)')
                return False

            return True

        return False

    def calculate_and_publish(self) -> None:
        """定期コールバック: 通信品質を計算しパブリッシュする。"""

        # 必要なデータが揃っているか確認
        if self.rx_position is None:
            self.get_logger().warn('基地局位置が未設定', throttle_duration_sec=5.0)
            return

        # ローカル位置（/odom由来）が必要: rx_positionと同一座標系を維持
        if self.tx_local_position is None or self.tx_orientation is None:
            self.get_logger().debug('/odom または /imu データ待機中...')
            return

        # 現在時刻の取得
        current_time = self.get_clock().now().nanoseconds / 1e9

        # サンプリング周期の決定
        dt_target = 1.0 / self.sampling_rate

        # サブステップの分割数を決定
        last_calc_sim_time = getattr(self, '_last_calc_sim_time', None)
        last_tx_pos = getattr(self, 'last_tx_pos', None)
        last_tx_orientation = getattr(self, 'last_tx_orientation', None)

        # 位置と姿勢の開始点と終了点をあらかじめ投影しておく
        if 'shinkansen' in self.vehicle_name:
            orig_pos = self.tx_local_position
            
            # last position projection
            if last_tx_pos is not None:
                self.tx_local_position = self.last_tx_pos
                px_last, py_last, yaw_last = self.get_current_segment_pose()
                if px_last is not None:
                    self.last_tx_pos[0] = px_last
                    self.last_tx_pos[1] = py_last
                if yaw_last is not None:
                    self.last_tx_orientation = np.array([0.0, 0.0, yaw_last], dtype=float)
            
            # current position projection
            self.tx_local_position = orig_pos
            px_curr, py_curr, yaw_curr = self.get_current_segment_pose()
            if px_curr is not None:
                self.tx_local_position[0] = px_curr
                self.tx_local_position[1] = py_curr
            if yaw_curr is not None:
                self.tx_orientation = np.array([0.0, 0.0, yaw_curr], dtype=float)

        if last_calc_sim_time is None or last_tx_pos is None or last_tx_orientation is None:
            # 初回は1ステップのみ
            self._last_calc_sim_time = current_time
            self.last_tx_pos = np.copy(self.tx_local_position)
            self.last_tx_orientation = np.copy(self.tx_orientation)
            num_steps = 1
            dt_actual = dt_target
        else:
            elapsed = current_time - last_calc_sim_time
            if elapsed <= 0.0:
                # 時刻が進んでいない場合はスキップ
                return
            num_steps = int(round(elapsed / dt_target))
            if num_steps <= 0:
                num_steps = 1
            elif num_steps > 5000:
                self.get_logger().warn(
                    f'大きいシミュレーション時間ギャップを検出 ({elapsed:.3f}s)。サンプリングレート {self.sampling_rate}Hz に対してサブステップ数を 5000 に制限します。',
                    throttle_duration_sec=5.0
                )
                num_steps = 5000
            dt_actual = elapsed / num_steps

        # 最終ステップのメトリクスを保持するための変数
        last_metrics = None
        last_tx_total = 0.0
        last_rx_total = 0.0
        last_bs_antenna_pos = None
        actual_throughput = 0.0

        # サブステップループ
        for step in range(1, num_steps + 1):
            frac = step / num_steps
            t_sub = self._last_calc_sim_time + step * dt_actual

            # 位置と姿勢の補間 (すでに投影済み)
            pos_sub = self.last_tx_pos + frac * (self.tx_local_position - self.last_tx_pos)
            rpy_sub = self.last_tx_orientation + frac * (self.tx_orientation - self.last_tx_orientation)

            # アンテナ位置の計算
            tx_antenna_pos = pos_sub + np.asarray(self.tx_antenna_offset, dtype=float)
            tx_ant_rpy = rpy_sub + np.asarray(self.tx_antenna_relative_rpy, dtype=float)

            # 共通の受信側回転行列を1回だけ計算
            rx_rotmat = self.antenna_parser._rpy_to_rotmat(float(tx_ant_rpy[0]), float(tx_ant_rpy[1]), float(tx_ant_rpy[2]))

            # 基地局の選定
            best_metrics = None
            best_bs_idx = 0
            best_tx_total = 0.0
            best_rx_total = 0.0
            best_bs_antenna_pos = None

            for idx, bs in enumerate(self.rx_nodes):
                bs_base = bs['position']
                bs_antenna_pos = bs_base + bs['antenna_offset']
                bs_ant_rpy = bs['rpy'] + bs['antenna_relative_rpy']

                # アンテナゲイン計算
                tx_e, tx_h, tx_total, rx_e, rx_h, rx_total = self.antenna_parser.get_tx_rx_gains(
                    tx_pos_world=bs_antenna_pos,
                    tx_rpy_world=bs_ant_rpy,
                    rx_pos_world=tx_antenna_pos,
                    rx_rpy_world=tx_ant_rpy,
                    tx_rotmat=bs.get('rotmat'),
                    rx_rotmat=rx_rotmat,
                )
                antenna_gain_db = float(tx_total + rx_total)

                # 通信メトリクス計算 (AWGNノイズあり)
                metrics = self.comms_calculator.calculate_all(
                    tx_antenna_pos,
                    bs_antenna_pos,
                    antenna_gain_db=antenna_gain_db,
                    add_noise=True,
                )

                if best_metrics is None or metrics['rssi'] > best_metrics['rssi']:
                    best_metrics = metrics
                    best_bs_idx = idx
                    best_tx_total = tx_total
                    best_rx_total = rx_total
                    best_bs_antenna_pos = bs_antenna_pos

            # 最善の基地局のメトリクスを採用
            metrics = best_metrics
            tx_total = best_tx_total
            rx_total = best_rx_total
            bs_antenna_pos = best_bs_antenna_pos

            # 選択された基地局の情報を更新
            self.rx_position = self.rx_nodes[best_bs_idx]['position']
            self.rx_antenna_offset = self.rx_nodes[best_bs_idx]['antenna_offset']
            self.rx_entity_rpy = self.rx_nodes[best_bs_idx]['rpy']
            self.rx_antenna_relative_rpy = list(self.rx_nodes[best_bs_idx]['antenna_relative_rpy'])

            # 最新RSSIをキャッシュ
            self._last_rssi = metrics['rssi']

            # ローカルでのリンク権判定（高精度スケジューリングの遅延回避）
            local_has_link_grant = self.has_link_grant
            if (self.scheduling_policy in ('feedforward_optimal', 'rssi_priority', 'physical_score_priority', 'geometric_beam_priority', 'geometric_weighted') and 
                self.vehicle_antennas):
                best_rssi_per_ant = {}
                for va in self.vehicle_antennas:
                    is_current_va = (va['name'] == self.vehicle_name)
                    if is_current_va and self.comms_calculator.noise_variance == 0.0:
                        best_va_rssi = metrics['rssi']
                    else:
                        va_offset = np.asarray(va['offset'], dtype=float)
                        va_pos_world = pos_sub + va_offset
                        
                        best_va_rssi = float('-inf')
                        for bs in self.rx_nodes:
                            bs_antenna_pos = bs['position'] + bs['antenna_offset']
                            bs_ant_rpy = bs['rpy'] + bs['antenna_relative_rpy']
                            
                            tx_e, tx_h, tx_total, rx_e, rx_h, rx_total = self.antenna_parser.get_tx_rx_gains(
                                tx_pos_world=bs_antenna_pos,
                                tx_rpy_world=bs_ant_rpy,
                                rx_pos_world=va_pos_world,
                                rx_rpy_world=tx_ant_rpy, # since relative RPY are identical, we can use tx_ant_rpy and rx_rotmat!
                                tx_rotmat=bs.get('rotmat'),
                                rx_rotmat=rx_rotmat,
                            )
                            antenna_gain_db = float(tx_total + rx_total)
                            
                            metrics_va = self.comms_calculator.calculate_all(
                                va_pos_world,
                                bs_antenna_pos,
                                antenna_gain_db=antenna_gain_db,
                                add_noise=False,
                            )
                            if metrics_va['rssi'] > best_va_rssi:
                                best_va_rssi = metrics_va['rssi']
                    
                    best_rssi_per_ant[va['name']] = best_va_rssi
                
                best_ant_name = max(best_rssi_per_ant, key=best_rssi_per_ant.get)
                local_has_link_grant = (self.vehicle_name == best_ant_name)

            # リンク状態の更新
            link_ready = self._update_link_state(metrics['rssi'], t_sub, has_link_grant=local_has_link_grant)

            # 累計伝送データ量の更新
            actual_throughput = 0.0
            if link_ready and self.comm_active and self._logging_ready:
                actual_throughput = metrics['throughput']
                if metrics['throughput'] > 0:
                    data_this_period = metrics['throughput'] * 1000 / 8 * dt_actual  # Mbps * s = MB
                    self.total_data_transmitted += data_this_period
                    if self.comm_data_limit_mb > 0 and self.total_data_transmitted >= self.comm_data_limit_mb:
                        self.comm_active = False

            # ループの最後で最終ステップの値をパブリッシュ用に保存
            if step == num_steps:
                last_metrics = metrics
                last_tx_total = tx_total
                last_rx_total = rx_total
                last_bs_antenna_pos = bs_antenna_pos

        # 状態の保存
        self._last_calc_sim_time = current_time
        self.last_tx_pos = np.copy(self.tx_local_position)
        self.last_tx_orientation = np.copy(self.tx_orientation)

        # リンク権限要求（最終ステップのRSSIをレポート）
        if hasattr(self, 'link_request_pub') and self.link_request_pub:
            req_msg = Float64()
            req_msg.data = float(last_metrics['rssi']) if self.comm_active else float('-inf')
            self.link_request_pub.publish(req_msg)

        # ROSメッセージとしてパブリッシュ (最終ステップの情報をパブリッシュ)
        if self.quality_pub is not None and CommsQuality is not None:
            msg = CommsQuality()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'world'
            msg.distance = last_metrics['distance']
            msg.rssi = last_metrics['rssi']
            msg.throughput = actual_throughput  # 最終ステップの実効スループット
            msg.total_data_transmitted = self.total_data_transmitted
            msg.tx_x = self.tx_local_position[0]
            msg.tx_y = self.tx_local_position[1]
            msg.tx_z = self.tx_local_position[2]
            msg.rx_x = self.rx_position[0]
            msg.rx_y = self.rx_position[1]
            msg.rx_z = last_bs_antenna_pos[2]
            msg.antenna_gain_e_plane = float(last_tx_total)
            msg.antenna_gain_h_plane = float(last_rx_total)
            msg.path_loss = last_metrics['path_loss']
            msg.comm_active = self.comm_active
            msg.link_state = self.link_state.name

            self.quality_pub.publish(msg)

        # ログ情報出力 (CONNECTED時のみ出力)
        if self.link_state == LinkState.CONNECTED:
            link_status = f"[{self.link_state.name}]"
            elapsed = current_time - (self.simulation_start_time or current_time)
            self.get_logger().info(
                f'[{elapsed:.1f}s] {link_status} D={last_metrics["distance"]:.1f}m, '
                f'RSSI={last_metrics["rssi"]:.1f}dBm, '
                f'TP={actual_throughput:.2f}Gbps, '
                f'Total={self.total_data_transmitted:.1f}MB',
                throttle_duration_sec=1.0
            )

    def _on_mission_complete(self, msg: Bool) -> None:
        """ミッション完了通知受信コールバック"""
        pass


def main(args=None):
    """メインエントリポイント。"""
    rclpy.init(args=args)

    node = CommsSimulatorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
