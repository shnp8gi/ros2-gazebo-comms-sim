#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
# 通信シミュレータノード
# UGV-基地局間の無線通信品質をシミュレーションするROS 2ノード
# =============================================================================
"""
UGVと基地局間の通信品質をシミュレーションするROS 2ノード。

サブスクライブトピック:
    - /imu/data (sensor_msgs/Imu): UGVの姿勢情報
    - /odom (nav_msgs/Odometry): UGVのローカル位置
    - /base_station/pose (geometry_msgs/PoseStamped): 基地局の姿勢（オプション）

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
    - base_station_pose_topic: 基地局姿勢トピック名
"""

import atexit
import csv
import os
from datetime import datetime
from typing import Optional, List, Tuple
from enum import Enum

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Header
from std_msgs.msg import Bool

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

    UGVの位置・姿勢と基地局の配置に基づき、
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
        self.declare_parameter('base_station_position', [0.0, 0.0, 0.0])
        self.declare_parameter('base_station_antenna_offset', [0.0, 0.0, 10.5])
        self.declare_parameter('ugv_antenna_offset', [0.0, 0.0, 1.3])
        self.declare_parameter('mcs_table_path', '')
        self.declare_parameter('link_establishment_time_ms', 2.0)
        self.declare_parameter('mission_complete_topic', '/mission_complete')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('ugv_spawn_pose', [-20.0, 0.0, 0.0])
        self.declare_parameter('ugv_antenna_relative_rpy', [0.0, 0.0, 0.0])
        self.declare_parameter('base_station_antenna_relative_rpy', [0.0, 0.0, 0.0])
        self.declare_parameter('base_station_pose_topic', '/base_station/pose')
        self.declare_parameter('logging_start_trigger', 'on_movement')
        self.declare_parameter('logging_start_topic', '/logging/start')
        self.declare_parameter('max_antenna_attenuation', 30.0)
        self.declare_parameter('vehicle_name', '')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        # パラメータ取得
        self.sampling_rate = self.get_parameter('sampling_rate').value
        self.noise_variance = self.get_parameter('noise_variance').value
        self.e_plane_path = self.get_parameter('e_plane_path').value
        self.h_plane_path = self.get_parameter('h_plane_path').value
        self.comm_data_limit_mb = self.get_parameter('comm_data_limit_mb').value
        self.tx_power = self.get_parameter('tx_power').value
        self.mcs_table_path = self.get_parameter('mcs_table_path').value
        self.link_establishment_time = self.get_parameter('link_establishment_time_ms').value / 1000.0  # ミリ秒→秒
        self._mission_complete_topic: str = str(self.get_parameter('mission_complete_topic').value)
        self.odom_topic: str = str(self.get_parameter('odom_topic').value)
        self.ugv_spawn_pose: List[float] = list(self.get_parameter('ugv_spawn_pose').value)
        self.ugv_antenna_relative_rpy: List[float] = list(self.get_parameter('ugv_antenna_relative_rpy').value)
        self.base_station_antenna_relative_rpy: List[float] = list(self.get_parameter('base_station_antenna_relative_rpy').value)
        self.base_station_pose_topic: str = str(self.get_parameter('base_station_pose_topic').value)
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
        self.base_station_entity_rpy: np.ndarray = np.array([0.0, 0.0, 0.0], dtype=float)

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
        # 基地局位置をパラメータから取得
        bs_pos = self.get_parameter('base_station_position').value
        self.base_station_position: Optional[np.ndarray] = np.array(bs_pos) if bs_pos else None
        self.base_station_antenna_offset: np.ndarray = np.array(self.get_parameter('base_station_antenna_offset').value, dtype=float)
        self.ugv_antenna_offset: np.ndarray = np.array(self.get_parameter('ugv_antenna_offset').value, dtype=float)

        self.ugv_orientation: Optional[np.ndarray] = None  # [roll, pitch, yaw]
        self.ugv_local_position: Optional[np.ndarray] = None
        self._odom_offset_set: bool = False
        self._odom_offset: np.ndarray = np.zeros(3, dtype=float)

        self.total_data_transmitted: float = 0.0  # 累計伝送データ量 [MB]
        self.comm_active: bool = True
        self.simulation_start_time: Optional[float] = None

        # ログ記録準備完了フラグ（トリガー条件が満たされたらTrue）
        self._logging_ready: bool = (self._logging_trigger == 'immediate')
        # リンク確立状態管理
        self.link_state: LinkState = LinkState.DISCONNECTED
        self.link_establishment_start_time: Optional[float] = None

        # CSV出力用データログ
        self.data_log: List[dict] = []
        # CSV保存済みフラグ（二重保存防止）
        self._csv_saved: bool = False

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

        # UGVコントローラからのミッション完了通知
        self.mission_complete_sub = self.create_subscription(
            Bool,
            self._mission_complete_topic,
            self._on_mission_complete,
            10
        )

        # 基地局姿勢サブスクライバ（オプション: 基地局の回転対応）
        self._bs_pose_sub = self.create_subscription(
            PoseStamped,
            self.base_station_pose_topic,
            self._on_base_station_pose,
            10,
        )

        # =====================================================================
        # パブリッシャ
        # =====================================================================
        if CommsQuality is not None:
            self.quality_pub = self.create_publisher(
                CommsQuality,
                '/comms/quality',
                10
            )
        else:
            self.quality_pub = None
            self.get_logger().warn('CommsQualityメッセージが利用できません')

        # =====================================================================
        # 定期計算タイマー
        # =====================================================================
        period = 1.0 / self.sampling_rate
        self.calc_timer = self.create_timer(period, self.calculate_and_publish)

        # =====================================================================
        # 終了時クリーンアップ登録
        # =====================================================================
        atexit.register(self.save_log_to_csv)

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

    def set_base_station_position(
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
        self.base_station_position = np.array(position[:3])
        self.base_station_antenna_offset = np.array(antenna_offset[:3], dtype=float)
        self.get_logger().info(
            f'基地局設定: {position}, アンテナオフセット: {antenna_offset}'
        )

    def set_ugv_antenna_offset(self, offset: List[float]) -> None:
        """
        UGVアンテナオフセットを設定する。

        Args:
            offset: UGVモデル原点からのアンテナオフセット [x, y, z]
        """
        self.ugv_antenna_offset = np.array(offset[:3], dtype=float)

    def odom_callback(self, msg: Odometry) -> None:
        """オドメトリ更新コールバック（ローカル位置 [m] として処理）。"""
        p = msg.pose.pose.position
        odom_pos = np.array([p.x, p.y, p.z], dtype=float)

        # 初回のオドメトリ受信時にオフセットを計算
        # 最初の受信位置がugv_spawn_poseに一致するようにする
        if not self._odom_offset_set:
            if isinstance(self.ugv_spawn_pose, (list, tuple)) and len(self.ugv_spawn_pose) >= 3:
                spawn = np.array([
                    float(self.ugv_spawn_pose[0]),
                    float(self.ugv_spawn_pose[1]),
                    float(self.ugv_spawn_pose[2]),
                ], dtype=float)
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
        self.ugv_local_position = odom_pos + self._odom_offset

        # シミュレーション開始時刻を記録
        if self.simulation_start_time is None:
            self.simulation_start_time = self.get_clock().now().nanoseconds / 1e9

    def imu_callback(self, msg: Imu) -> None:
        """IMU姿勢コールバック。"""
        # クォータニオンをオイラー角に変換
        self.ugv_orientation = self._quat_to_rpy(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )

    def set_ugv_local_position(self, position: np.ndarray) -> None:
        """
        UGVのローカル位置を直接設定する（オドメトリまたは直接ポーズ経由）。

        Args:
            position: ワールド座標系での位置 [x, y, z]
        """
        self.ugv_local_position = position

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

    def _on_base_station_pose(self, msg: PoseStamped) -> None:
        """基地局姿勢コールバック。"""
        q = msg.pose.orientation
        self.base_station_entity_rpy = self._quat_to_rpy(float(q.x), float(q.y), float(q.z), float(q.w))

    def _on_cmd_vel(self, msg: Twist) -> None:
        """速度指令コールバック（on_movementトリガー用）。"""
        if not self._logging_ready:
            # 直進速度が非ゼロならUGVが動き始めたと判定
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

    def _update_link_state(self, rssi: float, current_time: float) -> bool:
        """
        RSSIに基づいてリンク確立状態を更新する。

        状態遷移:
        - DISCONNECTED → ESTABLISHING: RSSIが閾値を上回った
        - ESTABLISHING → CONNECTED: リンク確立時間が経過した
        - ESTABLISHING → DISCONNECTED: RSSIが閾値を下回った
        - CONNECTED → DISCONNECTED: RSSIが閾値を下回った

        Args:
            rssi: 現在のRSSI [dBm]
            current_time: 現在のシミュレーション時刻 [s]

        Returns:
            通信が許可される場合True、それ以外はFalse
        """
        if self.link_state == LinkState.DISCONNECTED:
            # RSSIが閾値を超えたか確認
            if rssi > self.rssi_threshold:
                self.link_state = LinkState.ESTABLISHING
                self.link_establishment_start_time = current_time
                self.get_logger().info(
                    f'リンク確立開始 (RSSI: {rssi:.1f} dBm)'
                )
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
        if self.base_station_position is None:
            self.get_logger().warn('基地局位置が未設定', throttle_duration_sec=5.0)
            return

        # ローカル位置（/odom由来）が必要: base_station_positionと同一座標系を維持
        if self.ugv_local_position is None or self.ugv_orientation is None:
            self.get_logger().debug('/odom または /imu データ待機中...')
            return

        # UGVアンテナ位置 = UGVボディ位置 + アンテナオフセット
        ugv_pos = np.asarray(self.ugv_local_position, dtype=float)
        ugv_antenna_pos = ugv_pos + np.asarray(self.ugv_antenna_offset, dtype=float)

        # 基地局アンテナ位置 = 基地局原点 + アンテナオフセット
        bs_base = np.asarray(self.base_station_position, dtype=float)
        bs_antenna_pos = bs_base + np.asarray(self.base_station_antenna_offset, dtype=float)

        # アンテナ姿勢 = エンティティ姿勢 + 相対RPY
        ugv_ant_rpy = np.asarray(self.ugv_orientation, dtype=float) + np.asarray(self.ugv_antenna_relative_rpy, dtype=float)
        bs_ant_rpy = np.asarray(self.base_station_entity_rpy, dtype=float) + np.asarray(
            self.base_station_antenna_relative_rpy, dtype=float
        )

        # 送信(基地局) → 受信(UGV) のアンテナゲイン計算
        tx_e, tx_h, tx_total, rx_e, rx_h, rx_total = self.antenna_parser.get_tx_rx_gains(
            tx_pos_world=bs_antenna_pos,
            tx_rpy_world=bs_ant_rpy,
            rx_pos_world=ugv_antenna_pos,
            rx_rpy_world=ugv_ant_rpy,
        )

        antenna_gain_db = float(tx_total + rx_total)

        # 通信メトリクス計算
        metrics = self.comms_calculator.calculate_all(
            ugv_antenna_pos,
            bs_antenna_pos,
            antenna_gain_db=antenna_gain_db,
            add_noise=True,
        )

        # 現在時刻の取得
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed = current_time - (self.simulation_start_time or current_time)

        # RSSIに基づくリンク状態更新
        link_ready = self._update_link_state(metrics['rssi'], current_time)

        # 実効スループット計算（リンク状態とログ準備完了を考慮）
        actual_throughput = 0.0
        if link_ready and self.comm_active and self._logging_ready:
            actual_throughput = metrics['throughput']

            # 累計伝送データ量を更新
            if metrics['throughput'] > 0:
                # Gbps をサンプリング周期分の MB に変換
                period = 1.0 / self.sampling_rate
                data_this_period = metrics['throughput'] * 1000 / 8 * period  # Gbps → Mbps → MBps * s = MB
                self.total_data_transmitted += data_this_period

                # データ上限チェック
                if self.comm_data_limit_mb > 0:
                    if self.total_data_transmitted >= self.comm_data_limit_mb:
                        self.comm_active = False
                        self.get_logger().info(
                            f'データ上限到達: {self.total_data_transmitted:.2f} MB'
                        )

        # データログ記録
        log_entry = {
            'time_s': elapsed,
            'ugv_body_x_m': ugv_pos[0],
            'ugv_body_y_m': ugv_pos[1],
            'ugv_body_z_m': ugv_pos[2],
            'ugv_antenna_x_m': ugv_antenna_pos[0],
            'ugv_antenna_y_m': ugv_antenna_pos[1],
            'ugv_antenna_z_m': ugv_antenna_pos[2],
            'bs_origin_x_m': self.base_station_position[0],
            'bs_origin_y_m': self.base_station_position[1],
            'bs_origin_z_m': self.base_station_position[2],
            'bs_antenna_x_m': bs_antenna_pos[0],
            'bs_antenna_y_m': bs_antenna_pos[1],
            'bs_antenna_z_m': bs_antenna_pos[2],
            'distance_m': metrics['distance'],
            'rssi_dBm': metrics['rssi'],
            'throughput_Gbps': actual_throughput,
            'total_data_MB': self.total_data_transmitted,
            'path_loss_dB': metrics['path_loss'],
            'e_gain_dB': float(rx_e),
            'h_gain_dB': float(rx_h),
            'link_state': self.link_state.name,
        }
        if self._logging_ready:
            self.data_log.append(log_entry)

        # ROSメッセージとしてパブリッシュ
        if self.quality_pub is not None and CommsQuality is not None:
            msg = CommsQuality()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'world'
            msg.distance = metrics['distance']
            msg.rssi = metrics['rssi']
            msg.throughput = actual_throughput
            msg.total_data_transmitted = self.total_data_transmitted
            msg.ugv_x = ugv_pos[0]
            msg.ugv_y = ugv_pos[1]
            msg.ugv_z = ugv_pos[2]
            msg.base_station_x = self.base_station_position[0]
            msg.base_station_y = self.base_station_position[1]
            msg.base_station_z = bs_antenna_pos[2]
            msg.antenna_gain_e_plane = float(rx_e)
            msg.antenna_gain_h_plane = float(rx_h)
            msg.path_loss = metrics['path_loss']
            msg.comm_active = self.comm_active

            self.quality_pub.publish(msg)

        # ログ情報出力
        link_status = f"[{self.link_state.name}]"
        self.get_logger().info(
            f'[{elapsed:.1f}s] {link_status} D={metrics["distance"]:.1f}m, '
            f'RSSI={metrics["rssi"]:.1f}dBm, '
            f'TP={actual_throughput:.2f}Gbps, '
            f'Total={self.total_data_transmitted:.1f}MB',
            throttle_duration_sec=1.0
        )

    def save_log_to_csv(self) -> None:
        """収集したデータをCSVファイルに保存する。"""
        if self._csv_saved:
            self.get_logger().info('CSV保存済み')
            return
        if not self.data_log:
            self.get_logger().info('保存するデータがありません')
            return

        # ファイル名生成
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        if self.comm_data_limit_mb > 0:
            limit_str = f'LIMIT-{self.comm_data_limit_mb:.0f}MB'
        else:
            limit_str = 'LIMIT-UNLIMITED'

        if self.vehicle_name:
            filename = f'{timestamp}_{self.vehicle_name}_{limit_str}.csv'
        else:
            filename = f'{timestamp}_{limit_str}.csv'
        output_dir = '/workspace/sim_results/'

        # 出力ディレクトリ作成（存在しない場合）
        os.makedirs(output_dir, exist_ok=True)
        filepath = os.path.join(output_dir, filename)

        # CSV書き込み
        try:
            with open(filepath, 'w', newline='', encoding='utf-8') as f:
                # ヘッダコメント
                f.write(f'# 通信シミュレーション結果\n')
                f.write(f'# データ上限: {self.comm_data_limit_mb} MB\n')
                f.write(f'# 伝搬路モデル: {self.propagation_model.model_name}\n')
                f.write(f'# 送信電力: {self.tx_power} dBm\n')
                f.write(f'# 雑音分散: {self.noise_variance} dB\n')
                f.write('#\n')
                f.write('# 列の座標定義:\n')
                f.write('#   ugv_body    = UGV車体モデル原点（ホイールベース中心・地面レベル）\n')
                f.write('#   ugv_antenna = UGVアンテナ位置（ugv_body + antenna_offset）\n')
                f.write('#   bs_origin   = 基地局モデル原点\n')
                f.write('#   bs_antenna  = 基地局アンテナ位置（bs_origin + antenna_offset）\n')
                f.write('#   distance    = ugv_antenna と bs_antenna 間の3D距離\n')
                f.write('#\n')

                # データ書き込み
                fieldnames = [
                    'time_s',
                    'ugv_body_x_m', 'ugv_body_y_m', 'ugv_body_z_m',
                    'ugv_antenna_x_m', 'ugv_antenna_y_m', 'ugv_antenna_z_m',
                    'bs_origin_x_m', 'bs_origin_y_m', 'bs_origin_z_m',
                    'bs_antenna_x_m', 'bs_antenna_y_m', 'bs_antenna_z_m',
                    'distance_m', 'rssi_dBm', 'throughput_Gbps', 'total_data_MB',
                    'path_loss_dB', 'e_gain_dB', 'h_gain_dB', 'link_state'
                ]
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(self.data_log)

            self._csv_saved = True
            self.get_logger().info(f'データ保存完了: {filepath}')
        except Exception as e:
            self.get_logger().error(f'CSV保存失敗: {e}')

    def _on_mission_complete(self, msg: Bool) -> None:
        """ミッション完了通知受信時にCSVを保存する（1回のみ）。"""
        if not msg.data:
            return
        if self._saved_on_mission_complete:
            return

        self._saved_on_mission_complete = True
        self.get_logger().info('ミッション完了通知受信。CSV保存中...')
        self.save_log_to_csv()


def main(args=None):
    """メインエントリポイント。"""
    rclpy.init(args=args)

    node = CommsSimulatorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_log_to_csv()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass  # 既にシャットダウン済みの場合は無視


if __name__ == '__main__':
    main()
