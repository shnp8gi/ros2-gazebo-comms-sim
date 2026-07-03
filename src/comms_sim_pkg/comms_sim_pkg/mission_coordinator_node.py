#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
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

        self.vehicles = list(vehicle_names_param)
        self.completion_status = {v: False for v in self.vehicles}
        self.subscriptions_list = []

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

    def mission_complete_callback(self, msg: Bool, vehicle_name: str):
        if msg.data and not self.completion_status[vehicle_name]:
            self.completion_status[vehicle_name] = True
            self.get_logger().info(f"Vehicle '{vehicle_name}' has completed its mission.")
            
            # Check if all vehicles have completed their missions
            if all(self.completion_status.values()):
                self.get_logger().info("All vehicles have completed their missions! Waiting for logs to flush before shutting down...")
                
                # Sleep briefly to ensure Gazebo CSVs and ROS standard outputs are flushed
                import time
                time.sleep(3.0)
                
                # Exit cleanly so launch's OnProcessExit can catch it
                rclpy.shutdown()
                sys.exit(0)

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
