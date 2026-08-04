#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool

class MissionCoordinatorNode(Node):
    """
    A lightweight centralized node that tracks the mission completion status
    of all vehicles in the simulation. When all vehicles report their mission
    as complete, this node shuts down, causing the launch system to terminate
    the simulation safely.
    """
    def __init__(self):
        super().__init__('mission_coordinator_node')
        
        self.declare_parameter('vehicle_names', ['suv'])
        vehicle_names_param = self.get_parameter('vehicle_names').get_parameter_value().string_array_value
        
        # If the parameter list is empty, fallback to single default
        if not vehicle_names_param:
            vehicle_names_param = ['suv']

        # シム時刻による安全停止。全車の完走を待つだけだと、1台でも完走できない
        # 車両があると走行が終わらない (実測: 交通生成を使う構成で 20 走行中 9 走行が
        # 停止せず、シム時刻 3245 秒まで走り続けた)。
        # 実時間ではなくシム時刻で判定するため、計算機の速度によらず同じ地点で止まる
        # 完了報告が途絶えてからの猶予 [シム秒]。0 = 無効
        self.declare_parameter('idle_stop_s', 10.0)
        self.idle_stop_s = float(
            self.get_parameter('idle_stop_s').get_parameter_value().double_value)
        self._last_report_t = -1.0
        self._sim_t = 0.0

        self.declare_parameter('max_sim_time_s', 0.0)
        self.max_sim_time_s = float(
            self.get_parameter('max_sim_time_s').get_parameter_value().double_value)

        self.vehicles = list(vehicle_names_param)
        self.completion_status = {v: False for v in self.vehicles}
        self.subscriptions_list = []
        self._stopping = False

        if self.max_sim_time_s > 0.0:
            self.clock_sub = self.create_subscription(
                Clock, '/clock', self.clock_callback, 10)
            self.get_logger().info(
                f"Safety stop enabled: sim time limit {self.max_sim_time_s} s")

        self.get_logger().info(f"Mission Coordinator started. Waiting for {len(self.vehicles)} vehicles: {self.vehicles}")

        for v_name in self.vehicles:
            topic_name = f'/{v_name}/mission_complete'
            sub = self.create_subscription(
                Bool,
                topic_name,
                lambda msg, v=v_name: self.mission_complete_callback(msg, v),
                10
            )
            self.subscriptions_list.append(sub)

    def clock_callback(self, msg: Clock):
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        self._sim_t = t
        # 完了報告も plugin → gz-transport → bridge → ROS の多段経路を通るため
        # 取りこぼしうる。実測では、全車が終点に到達しているのに報告が揃わず
        # 上限まで空回りした走行が多数あった。報告が一定時間途絶えたら、
        # 走行は実質終わっているとみなして打ち切る (データは既に揃っている)
        if (self._last_report_t > 0.0 and self.idle_stop_s > 0.0
                and t - self._last_report_t >= self.idle_stop_s
                and any(self.completion_status.values())):
            done = sum(1 for v in self.completion_status.values() if v)
            self.get_logger().warn(
                f"完了報告が {self.idle_stop_s}s 途絶えました "
                f"({done}/{len(self.vehicles)} 受信)。走行を終了します")
            self._shutdown_after_flush()
            return
        if t >= self.max_sim_time_s:
            done = sum(1 for v in self.completion_status.values() if v)
            self.get_logger().warn(
                f"Sim time {t:.1f}s reached the limit ({self.max_sim_time_s}s). "
                f"{done}/{len(self.vehicles)} vehicles completed. Shutting down.")
            self._shutdown_after_flush()

    def _shutdown_after_flush(self):
        """ログの書き出しを待ってから終了する。

        コールバック内で rclpy.shutdown() を呼ぶとデッドロックするため、
        別スレッドで os._exit する。
        """
        if self._stopping:
            return
        self._stopping = True
        import os
        import threading
        import time

        def delayed_exit():
            time.sleep(3.0)
            os._exit(0)

        threading.Thread(target=delayed_exit, daemon=True).start()

    def mission_complete_callback(self, msg: Bool, vehicle_name: str):
        if msg.data and not self.completion_status[vehicle_name]:
            self.completion_status[vehicle_name] = True
            self._last_report_t = self._sim_t
            self.get_logger().info(f"Vehicle '{vehicle_name}' has completed its mission.")
            
            if all(self.completion_status.values()):
                self.get_logger().info("All vehicles have completed their missions! Waiting for logs to flush before shutting down...")
                self._shutdown_after_flush()

def main(args=None):
    rclpy.init(args=args)
    node = MissionCoordinatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Check if rclpy is still initialized (in case sys.exit wasn't called)
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
