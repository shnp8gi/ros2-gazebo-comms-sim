#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys

class ProgressLoggerNode(Node):
    def __init__(self):
        super().__init__('progress_logger')
        
        self.declare_parameter('vehicle_name', 'shinkansen')
        vehicle_name = self.get_parameter('vehicle_name').value
        
        topic_name = f'/{vehicle_name}/mission_progress'
        self.subscription = self.create_subscription(
            Float64,
            topic_name,
            self.progress_callback,
            10
        )
        self.get_logger().info(f"ProgressLoggerNode started, listening to {topic_name}")

    def progress_callback(self, msg):
        # Gazebo/TxControllerPlugin publishes 0.0 to 1.0
        # sweep_progress expects PROGRESS: XX.X%
        percentage = msg.data * 100.0
        print(f"PROGRESS: {percentage:.1f}%", flush=True)

def main(args=None):
    rclpy.init(args=args)
    node = ProgressLoggerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
