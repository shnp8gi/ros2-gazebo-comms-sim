#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
# UGVコントローラノード
# ウェイポイント追従制御を行うROS 2ノード
# =============================================================================
"""
ウェイポイント追従によりUGVの移動を制御するROS 2ノード。

サブスクライブトピック:
    - /odom (nav_msgs/Odometry): UGVオドメトリ（位置フィードバック）

パブリッシュトピック:
    - /cmd_vel (geometry_msgs/Twist): 速度指令
    - /mission_complete (std_msgs/Bool): ミッション完了通知

パラメータ:
    - waypoints: ウェイポイントリスト [X, Y, Z, V]（ネスト形式またはフラットリスト）
    - waypoint_tolerance: ウェイポイント到達判定距離 [m]
    - control_rate: 制御ループ周波数 [Hz]
    - max_angular_velocity: 最大旋回速度 [rad/s]
    - heading_gain: ヘディング制御の比例ゲイン
"""

from typing import List, Optional, Callable
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """
    クォータニオンからヨー角を抽出する。

    Args:
        x, y, z, w: クォータニオンの各成分

    Returns:
        ヨー角 [ラジアン]
    """
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class Waypoint:
    """位置と目標速度を持つウェイポイントクラス。"""

    def __init__(self, x: float, y: float, z: float, velocity: float) -> None:
        self.x = x
        self.y = y
        self.z = z
        self.velocity = velocity

    @classmethod
    def from_list(cls, data: List[float]) -> 'Waypoint':
        """[X, Y, Z, V] リストからウェイポイントを生成する。"""
        if len(data) < 4:
            raise ValueError(f"ウェイポイントには4要素必要ですが、{len(data)}要素しかありません")
        return cls(data[0], data[1], data[2], data[3])

    def distance_to(self, x: float, y: float) -> float:
        """指定座標との2D距離を計算する。"""
        return math.sqrt((self.x - x) ** 2 + (self.y - y) ** 2)

    def heading_to(self, x: float, y: float) -> float:
        """指定座標からこのウェイポイントへのヘディング角を計算する。"""
        return math.atan2(self.y - y, self.x - x)

    def __repr__(self) -> str:
        return f"Waypoint(x={self.x}, y={self.y}, z={self.z}, v={self.velocity})"


class UGVControllerNode(Node):
    """
    ウェイポイント追従制御を行うROS 2ノード。

    ヘディング制御に比例制御器を使用し、
    ウェイポイント間は一定速度で走行する。
    """

    def __init__(self) -> None:
        super().__init__('ugv_controller_node')

        # =====================================================================
        # パラメータ宣言
        # =====================================================================
        waypoints_descriptor = ParameterDescriptor(
            description='ウェイポイントのフラットリスト [x, y, z, v, ...]',
        )
        self.declare_parameters(
            '',
            [('waypoints', Parameter.Type.DOUBLE_ARRAY, waypoints_descriptor)],
        )
        self.declare_parameter('waypoint_tolerance', 2.0)
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('heading_gain', 1.5)
        self.declare_parameter('spawn_pose', [0.0, 0.0, 0.0])
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('mission_complete_topic', '/mission_complete')

        # パラメータ取得
        waypoints_raw = self.get_parameter('waypoints').value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        self.control_rate = self.get_parameter('control_rate').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.heading_gain = self.get_parameter('heading_gain').value
        self.spawn_pose = self.get_parameter('spawn_pose').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.mission_complete_topic = self.get_parameter('mission_complete_topic').value

        # ウェイポイント解析
        self.waypoints: List[Waypoint] = []
        if waypoints_raw:
            waypoint_groups: List[List[float]] = []
            if isinstance(waypoints_raw, list) and waypoints_raw:
                if isinstance(waypoints_raw[0], (list, tuple)):
                    # ネスト形式: [[x, y, z, v], ...]
                    waypoint_groups = [list(wp) for wp in waypoints_raw]
                else:
                    # フラット形式: [x, y, z, v, x, y, z, v, ...]
                    if len(waypoints_raw) % 4 != 0:
                        self.get_logger().warn(
                            f'フラットリストの要素数は4の倍数であるべきですが、{len(waypoints_raw)}要素です'
                        )
                    waypoint_groups = [
                        waypoints_raw[i:i + 4] for i in range(0, len(waypoints_raw), 4)
                    ]
            else:
                self.get_logger().warn(f'ウェイポイントパラメータの型が想定外: {type(waypoints_raw)}')

            for wp_data in waypoint_groups:
                try:
                    self.waypoints.append(Waypoint.from_list(wp_data))
                except (ValueError, TypeError) as e:
                    self.get_logger().warn(f'無効なウェイポイントデータ: {wp_data}, エラー: {e}')

        if not self.waypoints:
            self.get_logger().warn('ウェイポイントが設定されていません！')

        # =====================================================================
        # 状態変数
        # =====================================================================
        self.current_waypoint_idx: int = 0
        self.current_x: float = 0.0
        self.current_y: float = 0.0
        self.current_z: float = 0.0
        self.current_yaw: float = 0.0
        self.world_x: float = 0.0
        self.world_y: float = 0.0
        self.world_z: float = 0.0
        self.odom_received: bool = False
        self.mission_complete: bool = False
        self.odom_offset_set: bool = False
        self.odom_offset_x: float = 0.0
        self.odom_offset_y: float = 0.0
        self.odom_offset_z: float = 0.0
        self.current_cmd_vel: float = 0.0
        self.max_acceleration: float = 0.5
        self.last_log_time: float = 0.0

        # 位置更新コールバック（通信ノード連携用）
        self.position_callback: Optional[Callable[[np.ndarray], None]] = None

        # ミッション完了コールバック
        self.on_mission_complete: Optional[Callable[[], None]] = None

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
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            sensor_qos
        )

        # =====================================================================
        # パブリッシャ
        # =====================================================================
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10
        )

        # ミッション完了通知（他ノード向け、例: comms_node）
        self.mission_complete_pub = self.create_publisher(
            Bool,
            self.mission_complete_topic,
            10
        )

        # =====================================================================
        # 制御タイマー
        # =====================================================================
        period = 1.0 / self.control_rate
        self.control_timer = self.create_timer(period, self.control_loop)

        self.get_logger().info(
            f'UGVControllerNode 初期化完了\n'
            f'  ウェイポイント数: {len(self.waypoints)}\n'
            f'  到達判定距離: {self.waypoint_tolerance} m\n'
            f'  制御レート: {self.control_rate} Hz\n'
            f'  最大角速度: {self.max_angular_vel} rad/s\n'
            f'  odom: {self.odom_topic}\n'
            f'  cmd_vel: {self.cmd_vel_topic}\n'
            f'  mission_complete: {self.mission_complete_topic}'
        )
        self.get_logger().info(f'  スポーン位置: {self.spawn_pose}')

        for i, wp in enumerate(self.waypoints):
            self.get_logger().info(f'  WP{i}: {wp}')

    def set_position_callback(self, callback: Callable[[np.ndarray], None]) -> None:
        """
        位置更新コールバックを設定する。

        Args:
            callback: [x, y, z] 位置を受け取る関数
        """
        self.position_callback = callback

    def set_mission_complete_callback(self, callback: Callable[[], None]) -> None:
        """
        ミッション完了コールバックを設定する。

        Args:
            callback: 全ウェイポイント到達時に呼ばれる関数
        """
        self.on_mission_complete = callback

    def odom_callback(self, msg: Odometry) -> None:
        """オドメトリ更新コールバック。"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z
        self.current_yaw = quaternion_to_yaw(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        self.odom_received = True

        # スポーン位置を使ってodom→ワールドオフセットを1回だけ計算
        if (not self.odom_offset_set and isinstance(self.spawn_pose, (list, tuple))
                and len(self.spawn_pose) >= 3):
            self.odom_offset_x = float(self.spawn_pose[0]) - self.current_x
            self.odom_offset_y = float(self.spawn_pose[1]) - self.current_y
            self.odom_offset_z = float(self.spawn_pose[2]) - self.current_z
            self.odom_offset_set = True
            self.get_logger().info(
                f'odomオフセット計算完了: '
                f'({self.odom_offset_x:.3f}, {self.odom_offset_y:.3f}, {self.odom_offset_z:.3f})'
            )

        self.world_x = self.current_x + self.odom_offset_x
        self.world_y = self.current_y + self.odom_offset_y
        self.world_z = self.current_z + self.odom_offset_z

        # 位置更新コールバック呼び出し
        if self.position_callback:
            position = np.array([self.world_x, self.world_y, self.world_z])
            self.position_callback(position)

    def control_loop(self) -> None:
        """メイン制御ループ。"""
        if not self.odom_received:
            self.get_logger().debug('オドメトリ待機中...', throttle_duration_sec=2.0)
            return

        if self.mission_complete:
            return

        if not self.waypoints or self.current_waypoint_idx >= len(self.waypoints):
            self.complete_mission()
            return

        # 現在のターゲットウェイポイント
        target = self.waypoints[self.current_waypoint_idx]

        # ウェイポイントまでの距離とヘディングを計算
        distance = target.distance_to(self.world_x, self.world_y)
        target_heading = target.heading_to(self.world_x, self.world_y)

        # ウェイポイント到達チェック
        if distance < self.waypoint_tolerance:
            self.get_logger().info(
                f'ウェイポイント {self.current_waypoint_idx} 到達！ '
                f'({target.x}, {target.y})'
            )
            self.current_waypoint_idx += 1

            if self.current_waypoint_idx >= len(self.waypoints):
                self.complete_mission()
                return

            # 次のターゲットに更新
            target = self.waypoints[self.current_waypoint_idx]
            target_heading = target.heading_to(self.world_x, self.world_y)

        # ヘディング偏差計算
        heading_error = self.normalize_angle(target_heading - self.current_yaw)

        # 速度指令生成
        cmd = Twist()

        # 角速度（比例制御）
        angular_vel = self.heading_gain * heading_error
        angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, angular_vel))
        cmd.angular.z = angular_vel

        # 直進速度（急旋回時は減速）
        turn_factor = 1.0 - min(1.0, abs(heading_error) / (math.pi / 2))
        target_linear = target.velocity * max(0.3, turn_factor)

        # 急加速によるタイヤの空転(スリップ)とオドメトリ誤差を防ぐための加速度制限
        accel_step = self.max_acceleration / self.control_rate
        if target_linear > self.current_cmd_vel:
            self.current_cmd_vel = min(self.current_cmd_vel + accel_step, target_linear)
        else:
            self.current_cmd_vel = max(self.current_cmd_vel - accel_step, target_linear)
            
        cmd.linear.x = self.current_cmd_vel

        # 指令値パブリッシュ
        self.cmd_vel_pub.publish(cmd)

        # デバッグ: 1秒ごとに現在の指令速度とオドメトリの変化を表示して空転を監視
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - getattr(self, 'last_log_time', 0.0) >= 1.0:
            self.last_log_time = current_time
            self.get_logger().info(f"[スリップ監視] 指令速度: {self.current_cmd_vel:.2f} m/s, オドメトリ距離: {self.current_x:.2f}")

        # デバッグログ
        self.get_logger().debug(
            f'WP{self.current_waypoint_idx}: dist={distance:.2f}m, '
            f'heading_err={math.degrees(heading_error):.1f}°, '
            f'v={cmd.linear.x:.2f}m/s, w={cmd.angular.z:.2f}rad/s'
        )

    def complete_mission(self) -> None:
        """ミッション完了処理。"""
        if self.mission_complete:
            return

        self.mission_complete = True

        # 車両停止
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

        self.get_logger().info('=== ミッション完了 ===')
        self.get_logger().info('全ウェイポイント到達。車両を停止します。')

        # ミッション完了通知パブリッシュ
        msg = Bool()
        msg.data = True
        self.mission_complete_pub.publish(msg)

        # 完了コールバック呼び出し
        if self.on_mission_complete:
            self.on_mission_complete()

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """
        角度を [-π, π] に正規化する。

        Args:
            angle: 角度 [ラジアン]

        Returns:
            正規化された角度
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def is_mission_complete(self) -> bool:
        """ミッション完了状態を確認する。"""
        return self.mission_complete


def main(args=None):
    """メインエントリポイント。"""
    rclpy.init(args=args)

    node = UGVControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # シャットダウン前に車両停止
        try:
            stop_cmd = Twist()
            node.cmd_vel_pub.publish(stop_cmd)
        except Exception:
            pass  # コンテキスト無効化後はパブリッシュ不可
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass  # 既にシャットダウン済みの場合は無視


if __name__ == '__main__':
    main()
