#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
# リンク制御ノード
# 基地局と車両間の一対一通信を保証する中央スケジューラ
# =============================================================================
"""
基地局が同時に通信できる車両を1台に制限する中央制御ノード。

各 comms_node から RSSI を受信し、スケジューリングポリシーに基づいて
1台の車両にのみリンク権（link_grant）を付与する。

サブスクライブトピック:
    - /{vehicle_name}/link_request (std_msgs/Float64): 各車両のRSSI
    - /{vehicle_name}/mission_complete (std_msgs/Bool): 各車両のミッション完了通知

パブリッシュトピック:
    - /{vehicle_name}/link_grant (std_msgs/Bool): リンク権付与

パラメータ:
    - vehicle_names: 車両名リスト
    - scheduling_policy: スケジューリングポリシー
        - sequential: 車両順に通信、ミッション完了で次へ切替
        - round_robin: タイムスロットで順番に切替
        - rssi_priority: RSSI最大の車両を優先
    - time_slot_duration_s: round_robin 時のスロット長 [s]
"""

from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64


class LinkControllerNode(Node):
    """
    基地局の一対一通信を管理するリンク制御ノード。

    基地局の指向性アンテナは同時に1台の車両としか通信できないため、
    このノードがどの車両にリンク権を与えるかを決定する。
    """

    def __init__(self) -> None:
        super().__init__('link_controller_node')

        # =====================================================================
        # パラメータ宣言・取得
        # =====================================================================
        self.declare_parameter('vehicle_names', [''])
        self.declare_parameter('scheduling_policy', 'sequential')
        self.declare_parameter('time_slot_duration_s', 10.0)

        vehicle_names_raw = self.get_parameter('vehicle_names').value
        self.scheduling_policy: str = str(
            self.get_parameter('scheduling_policy').value
        ).lower()
        self.time_slot_duration: float = float(
            self.get_parameter('time_slot_duration_s').value
        )

        # 車両名リストのパース
        if isinstance(vehicle_names_raw, list):
            self.vehicle_names: List[str] = [
                str(v) for v in vehicle_names_raw if v
            ]
        else:
            self.vehicle_names = []

        if not self.vehicle_names:
            self.get_logger().error('vehicle_names が空です。リンク制御を開始できません。')
            return

        # ポリシー検証
        valid_policies = ('sequential', 'round_robin', 'rssi_priority')
        if self.scheduling_policy not in valid_policies:
            self.get_logger().warn(
                f'不明なポリシー: "{self.scheduling_policy}"。'
                f'"sequential" を使用します。'
            )
            self.scheduling_policy = 'sequential'

        # =====================================================================
        # 状態変数
        # =====================================================================
        # 現在リンク権を持つ車両のインデックス
        self._active_idx: int = 0

        # 各車両の最新RSSI
        self._rssi: Dict[str, float] = {
            name: float('-inf') for name in self.vehicle_names
        }

        # 各車両のミッション完了フラグ
        self._mission_complete: Dict[str, bool] = {
            name: False for name in self.vehicle_names
        }

        # round_robin 用: 最後にスロットを切り替えた時刻
        self._last_slot_switch_time: Optional[float] = None

        # =====================================================================
        # サブスクライバ・パブリッシャの動的生成
        # =====================================================================
        self._grant_pubs: Dict[str, object] = {}
        self._request_subs: List[object] = []
        self._mission_subs: List[object] = []

        for name in self.vehicle_names:
            # リンクグラント パブリッシャ
            pub = self.create_publisher(Bool, f'/{name}/link_grant', 10)
            self._grant_pubs[name] = pub

            # リンクリクエスト サブスクライバ（RSSI報告）
            sub = self.create_subscription(
                Float64,
                f'/{name}/link_request',
                lambda msg, vn=name: self._on_link_request(vn, msg),
                10
            )
            self._request_subs.append(sub)

            # ミッション完了 サブスクライバ
            sub_mc = self.create_subscription(
                Bool,
                f'/{name}/mission_complete',
                lambda msg, vn=name: self._on_mission_complete(vn, msg),
                10
            )
            self._mission_subs.append(sub_mc)

        # =====================================================================
        # スケジューリングタイマー (10Hz)
        # =====================================================================
        self._schedule_timer = self.create_timer(0.1, self._schedule_tick)

        self.get_logger().info(
            f'LinkControllerNode 初期化完了\n'
            f'  車両数: {len(self.vehicle_names)}\n'
            f'  車両名: {self.vehicle_names}\n'
            f'  ポリシー: {self.scheduling_policy}\n'
            f'  タイムスロット: {self.time_slot_duration} s'
        )

    # =========================================================================
    # コールバック
    # =========================================================================

    def _on_link_request(self, vehicle_name: str, msg: Float64) -> None:
        """各車両からのRSSI報告を受信。"""
        self._rssi[vehicle_name] = msg.data

    def _on_mission_complete(self, vehicle_name: str, msg: Bool) -> None:
        """各車両のミッション完了通知を受信。"""
        if not msg.data:
            return
        if not self._mission_complete.get(vehicle_name, False):
            self._mission_complete[vehicle_name] = True
            self.get_logger().info(
                f'ミッション完了通知受信: {vehicle_name}'
            )

    # =========================================================================
    # スケジューリング
    # =========================================================================

    def _schedule_tick(self) -> None:
        """定期的にリンク権を決定し、各車両にブロードキャストする。"""
        if self.scheduling_policy == 'sequential':
            self._policy_sequential()
        elif self.scheduling_policy == 'round_robin':
            self._policy_round_robin()
        elif self.scheduling_policy == 'rssi_priority':
            self._policy_rssi_priority()

        # リンクグラントをパブリッシュ
        active_name = self.vehicle_names[self._active_idx]
        for name, pub in self._grant_pubs.items():
            msg = Bool()
            msg.data = (name == active_name)
            pub.publish(msg)

    def _policy_sequential(self) -> None:
        """
        sequential ポリシー:
        現在の車両のミッションが完了したら次の車両に切替。
        """
        current_name = self.vehicle_names[self._active_idx]
        if self._mission_complete.get(current_name, False):
            next_idx = self._active_idx + 1
            if next_idx < len(self.vehicle_names):
                self._active_idx = next_idx
                self.get_logger().info(
                    f'リンク切替: {current_name} → '
                    f'{self.vehicle_names[self._active_idx]} '
                    f'(sequential: ミッション完了)'
                )

    def _policy_round_robin(self) -> None:
        """
        round_robin ポリシー:
        一定時間ごとに次の車両に切替。
        """
        current_time = self.get_clock().now().nanoseconds / 1e9

        if self._last_slot_switch_time is None:
            self._last_slot_switch_time = current_time
            return

        elapsed = current_time - self._last_slot_switch_time
        if elapsed >= self.time_slot_duration:
            old_name = self.vehicle_names[self._active_idx]
            self._active_idx = (self._active_idx + 1) % len(self.vehicle_names)
            self._last_slot_switch_time = current_time
            self.get_logger().info(
                f'リンク切替: {old_name} → '
                f'{self.vehicle_names[self._active_idx]} '
                f'(round_robin: {elapsed:.1f}s 経過)'
            )

    def _policy_rssi_priority(self) -> None:
        """
        rssi_priority ポリシー:
        RSSIが最も高い車両にリンク権を付与。
        """
        best_name = max(self._rssi, key=self._rssi.get)
        best_idx = self.vehicle_names.index(best_name)
        if best_idx != self._active_idx:
            old_name = self.vehicle_names[self._active_idx]
            self._active_idx = best_idx
            self.get_logger().info(
                f'リンク切替: {old_name} → {best_name} '
                f'(rssi_priority: RSSI={self._rssi[best_name]:.1f} dBm)',
                throttle_duration_sec=2.0
            )


def main(args=None):
    """メインエントリポイント。"""
    rclpy.init(args=args)

    node = LinkControllerNode()

    try:
        rclpy.spin(node)
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
