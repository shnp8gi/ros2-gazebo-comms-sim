#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
# シミュレーション Ready ゲートノード
# 全ノードの初期化完了を確認し、一斉スタート信号を発行する
# =============================================================================
"""
全ノードの初期化完了を確認し、一斉スタート信号を発行するゲートノード。

現実世界での「全ECU起動確認 → システム Ready」に相当する。

サブスクライブトピック:
    - /{node_name}/ready (std_msgs/Bool): 各ノードの Ready 通知

パブリッシュトピック:
    - /sim/all_ready (std_msgs/Bool): 全ノード Ready 通知 (VOLATILE)

パラメータ:
    - expected_nodes: Ready を待つノード名のリスト
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool

_VOLATILE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)


class SimReadyGateNode(Node):
    """全ノードの初期化完了を確認し、一斉スタート信号を発行する。"""

    def __init__(self):
        super().__init__('sim_ready_gate')

        self.declare_parameter('expected_nodes', [''])
        expected_raw = self.get_parameter('expected_nodes').value

        if isinstance(expected_raw, list):
            self.expected_nodes = set(n for n in expected_raw if n)
        else:
            self.expected_nodes = set()

        self.ready_nodes = set()
        self._all_ready_published = False

        # 各ノードの Ready トピックを購読 (VOLATILE: 遅延起動でも受信可能)
        self._subs = []
        for node_name in self.expected_nodes:
            sub = self.create_subscription(
                Bool,
                f'/{node_name}/ready',
                lambda msg, n=node_name: self._on_ready(n, msg),
                _VOLATILE_QOS
            )
            self._subs.append(sub)

        # /sim/all_ready utilizes a VOLATILE QoS to avoid caching startup signals from previous runs
        self.all_ready_pub = self.create_publisher(
            Bool, '/sim/all_ready', _VOLATILE_QOS)

        self.get_logger().info(
            f'SimReadyGateNode 初期化完了。'
            f'{len(self.expected_nodes)} ノードの Ready を待機します: '
            f'{sorted(self.expected_nodes)}'
        )

    def _on_ready(self, node_name: str, msg: Bool):
        if not msg.data:
            return
        if node_name in self.ready_nodes:
            return

        self.ready_nodes.add(node_name)
        self.get_logger().info(
            f'Ready 受信: {node_name} '
            f'({len(self.ready_nodes)}/{len(self.expected_nodes)})'
        )

        if self.ready_nodes >= self.expected_nodes and not self._all_ready_published:
            self._all_ready_published = True
            self.get_logger().info(
                '=== 全ノード Ready。シミュレーション開始シグナルを発行します ==='
            )
            self._publish_all_ready()
            # 開始シグナルは繰り返し発行する。VOLATILE QoS では発行の瞬間に
            # 購読が確立していない相手に届かず、その車両は永久に動き出さない。
            # 実測で、28台すべてが開始シグナルを取り逃して 1 台も動かず、
            # 走行が終わらないまま打ち切られた。
            # (TRANSIENT_LOCAL にすると前の走行の信号を拾う恐れがあるため、
            #  QoS は VOLATILE のまま再送で担保する)
            self._repeat_timer = self.create_timer(0.5, self._publish_all_ready)

    def _publish_all_ready(self):
        msg = Bool()
        msg.data = True
        self.all_ready_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimReadyGateNode()
    try:
        while rclpy.ok():
            try:
                rclpy.spin_once(node, timeout_sec=0.1)
            except KeyboardInterrupt:
                break
            except Exception as e:
                node.get_logger().error(f"Error during spin (ignored to prevent crash): {e}")
                import time
                time.sleep(0.01)
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
