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
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import Bool, Float64

from comms_sim_msgs.msg import CommsQuality


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
        self.declare_parameter('beam_gain_threshold', 5.0)
        self.declare_parameter('weight_distance', 0.7)
        self.declare_parameter('weight_angle', 0.3)
        self.declare_parameter('scheduling_rate_hz', 1000.0)

        vehicle_names_raw = self.get_parameter('vehicle_names').value
        self.scheduling_policy: str = str(
            self.get_parameter('scheduling_policy').value
        ).lower()
        self.time_slot_duration: float = float(
            self.get_parameter('time_slot_duration_s').value
        )
        self.beam_gain_threshold: float = float(
            self.get_parameter('beam_gain_threshold').value
        )
        self.weight_distance: float = float(
            self.get_parameter('weight_distance').value
        )
        self.weight_angle: float = float(
            self.get_parameter('weight_angle').value
        )
        self.scheduling_rate_hz: float = float(
            self.get_parameter('scheduling_rate_hz').value
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
        valid_policies = (
            'sequential', 'round_robin', 'rssi_priority',
            'geometric_beam_priority', 'physical_score_priority', 'geometric_weighted'
        )
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
        self._last_grant_change_time: float = 0.0

        # 各車両の最新RSSI
        self._rssi: Dict[str, float] = {
            name: float('-inf') for name in self.vehicle_names
        }

        # 各車両のジオメトリ情報 (距離, アンテナゲイン, 有効フラグ等)
        self._geometry_info: Dict[str, dict] = {
            name: {} for name in self.vehicle_names
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
        self._quality_subs: List[object] = []
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

            # 幾何学・品質 サブスクライバ（距離・ゲイン情報を取得）
            sub_q = self.create_subscription(
                CommsQuality,
                f'/{name}/comms/quality',
                lambda msg, vn=name: self._on_comms_quality(vn, msg),
                10
            )
            self._quality_subs.append(sub_q)

            # ミッション完了 サブスクライバ
            sub_mc = self.create_subscription(
                Bool,
                f'/{name}/mission_complete',
                lambda msg, vn=name: self._on_mission_complete(vn, msg),
                10
            )
            self._mission_subs.append(sub_mc)

        # ストラテジーの初期化
        try:
            from .link_scheduling_strategy import (
                SequentialStrategy, RoundRobinStrategy, RssiPriorityStrategy,
                GeometricBeamPriorityStrategy, GeometricWeightedStrategy,
                PhysicalScorePriorityStrategy
            )
        except ImportError:
            from link_scheduling_strategy import (
                SequentialStrategy, RoundRobinStrategy, RssiPriorityStrategy,
                GeometricBeamPriorityStrategy, GeometricWeightedStrategy,
                PhysicalScorePriorityStrategy
            )

        if self.scheduling_policy == 'sequential':
            self.strategy = SequentialStrategy()
        elif self.scheduling_policy == 'round_robin':
            self.strategy = RoundRobinStrategy(self.time_slot_duration)
        elif self.scheduling_policy == 'rssi_priority':
            self.strategy = RssiPriorityStrategy()
        elif self.scheduling_policy == 'geometric_beam_priority':
            self.strategy = GeometricBeamPriorityStrategy(self.beam_gain_threshold)
        elif self.scheduling_policy == 'physical_score_priority':
            self.strategy = PhysicalScorePriorityStrategy(self.beam_gain_threshold)
        elif self.scheduling_policy == 'geometric_weighted':
            self.strategy = GeometricWeightedStrategy(self.weight_distance, self.weight_angle)
        else:
            self.strategy = SequentialStrategy()

        # =====================================================================
        # スケジューリングタイマー
        # =====================================================================
        self.timer_cb_group = MutuallyExclusiveCallbackGroup()
        
        timer_period = 1.0 / self.scheduling_rate_hz if self.scheduling_rate_hz > 0 else 0.001
        self._schedule_timer = self.create_timer(
            timer_period, 
            self._schedule_tick,
            callback_group=self.timer_cb_group
        )

        self.get_logger().info(
            f'LinkControllerNode 初期化完了\n'
            f'  車両数: {len(self.vehicle_names)}\n'
            f'  車両名: {self.vehicle_names}\n'
            f'  ポリシー: {self.scheduling_policy} (Strategy initialized)\n'
            f'  タイムスロット: {self.time_slot_duration} s'
        )

    # =========================================================================
    # コールバック
    # =========================================================================

    def _on_link_request(self, vehicle_name: str, msg: Float64) -> None:
        """各車両からのRSSI報告を受信。"""
        self._rssi[vehicle_name] = msg.data

    def _on_comms_quality(self, vehicle_name: str, msg: CommsQuality) -> None:
        """各車両からの通信品質・幾何学情報を受信。"""
        self._geometry_info[vehicle_name] = {
            'distance': msg.distance,
            'antenna_gain_e_plane': msg.antenna_gain_e_plane,
            'antenna_gain_h_plane': msg.antenna_gain_h_plane,
            'path_loss': msg.path_loss,
            'comm_active': msg.comm_active,
            'link_state': msg.link_state,
            'rssi': msg.rssi
        }

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
        """
        定期的にリンク権を決定し、各車両にブロードキャストする。
        
        スケジューリングは以下の2フェーズで行われます:
        【フェーズ1: ポリシーベースのスケジュール決定】
          設定された scheduling_policy (例: physical_score_priority, sequential 等)
          に従って、Strategyクラスが「次にリンク権を持つべき車両」を決定します。
        
        【フェーズ2: プロアクティブ・ハンドオーバー（通信断絶の回避）】
          フェーズ1で選ばれた車両が物理的な要因（距離やアンテナ角度など）により
          実際には通信を確立できていない（DISCONNECTED）場合、通信可能な別の車両を
          探して強制的にリンク権を切り替える安全装置です。
        """
        current_time = self.get_clock().now().nanoseconds / 1e9

        # --- フェーズ1: 基本的なスケジューリング実行 ---
        new_idx, log_msg = self.strategy.determine_active_link(
            self._rssi,
            self._mission_complete,
            self._geometry_info,
            self._active_idx,
            self.vehicle_names,
            current_time
        )

        if log_msg:
            self.get_logger().info(log_msg)
            self._active_idx = new_idx
            self._last_grant_change_time = current_time

        # --- フェーズ2: プロアクティブ・ハンドオーバー実行 ---
        # 現在のgrant保持者のlink_stateを確認
        # DISCONNECTED状態の場合、物理スコア（受信電力の推定値）が最良の車両に即座にgrantを移す
        active_name = self.vehicle_names[self._active_idx]
        active_info = self._geometry_info.get(active_name, {})
        active_link_state = active_info.get('link_state', 'DISCONNECTED')

        # リンク権付与直後は状態が伝播・確立するまでの猶予期間（0.1秒）を設ける
        grace_period_passed = (current_time - self._last_grant_change_time) > 0.1

        if active_link_state == 'DISCONNECTED' and active_info and grace_period_passed:
            best_score = float('-inf')
            best_idx = None
            for i, name in enumerate(self.vehicle_names):
                if i == self._active_idx:
                    continue
                info = self._geometry_info.get(name, {})
                if not info.get('comm_active', False):
                    continue
                # 物理スコア = 送信側総ゲイン(E) + 受信側総ゲイン(H) - パスロス
                # これは実質的にノイズを含まない理想的なRSSI(受信電力)に比例します
                e_gain = info.get('antenna_gain_e_plane', -999.0)
                h_gain = info.get('antenna_gain_h_plane', -999.0)
                path_loss = info.get('path_loss', 999.0)
                score = e_gain + h_gain - path_loss
                if score > best_score:
                    best_score = score
                    best_idx = i
            if best_idx is not None:
                active_score = (
                    active_info.get('antenna_gain_e_plane', -999.0) +
                    active_info.get('antenna_gain_h_plane', -999.0) -
                    active_info.get('path_loss', 999.0)
                )
                if best_score > active_score:
                    old_name = active_name
                    self._active_idx = best_idx
                    active_name = self.vehicle_names[self._active_idx]
                    self.get_logger().info(
                        f'強制リンク切替: {old_name}(DISCONNECTED) → {active_name} '
                        f'(geo_score={best_score:.1f})'
                    )
                    self._last_grant_change_time = current_time

        # リンクグラントをパブリッシュ
        for name, pub in self._grant_pubs.items():
            msg = Bool()
            has_grant = (name == active_name)
            msg.data = has_grant
            pub.publish(msg)


def main(args=None):
    """メインエントリポイント。"""
    rclpy.init(args=args)

    node = LinkControllerNode()
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
