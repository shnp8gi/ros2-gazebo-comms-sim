#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
# UGV Controller Node
# ROS 2 node for waypoint-following UGV control
# =============================================================================
"""
ROS 2 node for controlling UGV movement along waypoints.

Subscribes to:
    - /odom (nav_msgs/Odometry): UGV odometry for position feedback

Publishes:
    - /cmd_vel (geometry_msgs/Twist): Velocity commands

Parameters:
    - waypoints: List of [X, Y, Z, V] waypoints (nested or flat list)
    - waypoint_tolerance: Distance threshold for waypoint reached
    - control_rate: Control loop frequency [Hz]
    - max_angular_velocity: Maximum turning speed [rad/s]
    - heading_gain: Proportional gain for heading control
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


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """
    Extract yaw angle from quaternion.
    
    Args:
        x, y, z, w: Quaternion components
        
    Returns:
        Yaw angle in radians
    """
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class Waypoint:
    """Represents a waypoint with position and target velocity."""
    
    def __init__(self, x: float, y: float, z: float, velocity: float) -> None:
        self.x = x
        self.y = y
        self.z = z
        self.velocity = velocity
    
    @classmethod
    def from_list(cls, data: List[float]) -> 'Waypoint':
        """Create waypoint from [X, Y, Z, V] list."""
        if len(data) < 4:
            raise ValueError(f"Waypoint requires 4 values, got {len(data)}")
        return cls(data[0], data[1], data[2], data[3])
    
    def distance_to(self, x: float, y: float) -> float:
        """Calculate 2D distance to a point."""
        return math.sqrt((self.x - x) ** 2 + (self.y - y) ** 2)
    
    def heading_to(self, x: float, y: float) -> float:
        """Calculate heading angle from a point to this waypoint."""
        return math.atan2(self.y - y, self.x - x)
    
    def __repr__(self) -> str:
        return f"Waypoint(x={self.x}, y={self.y}, z={self.z}, v={self.velocity})"


class UGVControllerNode(Node):
    """
    ROS 2 node for UGV waypoint-following control.
    
    Implements a simple proportional controller for heading
    and constant velocity control between waypoints.
    """
    
    def __init__(self) -> None:
        super().__init__('ugv_controller_node')
        
        # =====================================================================
        # Declare parameters
        # =====================================================================
        waypoints_descriptor = ParameterDescriptor(
            description='Flat list of waypoints [x, y, z, v, ...]',
        )
        self.declare_parameters(
            '',
            [('waypoints', Parameter.Type.DOUBLE_ARRAY, waypoints_descriptor)],
        )
        self.declare_parameter('waypoint_tolerance', 2.0)
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('heading_gain', 1.5)
        
        # Get parameters
        waypoints_raw = self.get_parameter('waypoints').value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        self.control_rate = self.get_parameter('control_rate').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.heading_gain = self.get_parameter('heading_gain').value
        
        # Parse waypoints
        self.waypoints: List[Waypoint] = []
        if waypoints_raw:
            waypoint_groups: List[List[float]] = []
            if isinstance(waypoints_raw, list) and waypoints_raw:
                if isinstance(waypoints_raw[0], (list, tuple)):
                    waypoint_groups = [list(wp) for wp in waypoints_raw]
                else:
                    if len(waypoints_raw) % 4 != 0:
                        self.get_logger().warn(
                            f'Waypoints flat list length should be multiple of 4, got {len(waypoints_raw)}'
                        )
                    waypoint_groups = [
                        waypoints_raw[i:i + 4] for i in range(0, len(waypoints_raw), 4)
                    ]
            else:
                self.get_logger().warn(f'Waypoints parameter has unexpected type: {type(waypoints_raw)}')

            for wp_data in waypoint_groups:
                try:
                    self.waypoints.append(Waypoint.from_list(wp_data))
                except (ValueError, TypeError) as e:
                    self.get_logger().warn(f'Invalid waypoint data: {wp_data}, error: {e}')
        
        if not self.waypoints:
            self.get_logger().warn('No waypoints configured!')
        
        # =====================================================================
        # State variables
        # =====================================================================
        self.current_waypoint_idx: int = 0
        self.current_x: float = 0.0
        self.current_y: float = 0.0
        self.current_z: float = 0.0
        self.current_yaw: float = 0.0
        self.odom_received: bool = False
        self.mission_complete: bool = False
        
        # Callback for position updates (for comms node)
        self.position_callback: Optional[Callable[[np.ndarray], None]] = None
        
        # Callback for mission complete
        self.on_mission_complete: Optional[Callable[[], None]] = None
        
        # =====================================================================
        # QoS Profile
        # =====================================================================
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # =====================================================================
        # Subscribers
        # =====================================================================
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            sensor_qos
        )
        
        # =====================================================================
        # Publishers
        # =====================================================================
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # =====================================================================
        # Control timer
        # =====================================================================
        period = 1.0 / self.control_rate
        self.control_timer = self.create_timer(period, self.control_loop)
        
        self.get_logger().info(
            f'UGVControllerNode initialized\n'
            f'  Waypoints: {len(self.waypoints)}\n'
            f'  Tolerance: {self.waypoint_tolerance} m\n'
            f'  Control rate: {self.control_rate} Hz\n'
            f'  Max angular vel: {self.max_angular_vel} rad/s'
        )
        
        for i, wp in enumerate(self.waypoints):
            self.get_logger().info(f'  WP{i}: {wp}')
    
    def set_position_callback(self, callback: Callable[[np.ndarray], None]) -> None:
        """
        Set callback for position updates.
        
        Args:
            callback: Function that receives [x, y, z] position
        """
        self.position_callback = callback
    
    def set_mission_complete_callback(self, callback: Callable[[], None]) -> None:
        """
        Set callback for mission completion.
        
        Args:
            callback: Function called when all waypoints are reached
        """
        self.on_mission_complete = callback
    
    def odom_callback(self, msg: Odometry) -> None:
        """Handle odometry updates."""
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
        
        # Notify position callback
        if self.position_callback:
            position = np.array([self.current_x, self.current_y, self.current_z])
            self.position_callback(position)
    
    def control_loop(self) -> None:
        """Main control loop."""
        if not self.odom_received:
            self.get_logger().debug('Waiting for odometry...', throttle_duration_sec=2.0)
            return
        
        if self.mission_complete:
            return
        
        if not self.waypoints or self.current_waypoint_idx >= len(self.waypoints):
            self.complete_mission()
            return
        
        # Get current waypoint
        target = self.waypoints[self.current_waypoint_idx]
        
        # Calculate distance and heading to waypoint
        distance = target.distance_to(self.current_x, self.current_y)
        target_heading = target.heading_to(self.current_x, self.current_y)
        
        # Check if waypoint reached
        if distance < self.waypoint_tolerance:
            self.get_logger().info(
                f'Waypoint {self.current_waypoint_idx} reached! '
                f'({target.x}, {target.y})'
            )
            self.current_waypoint_idx += 1
            
            if self.current_waypoint_idx >= len(self.waypoints):
                self.complete_mission()
                return
            
            # Update target
            target = self.waypoints[self.current_waypoint_idx]
            target_heading = target.heading_to(self.current_x, self.current_y)
        
        # Calculate heading error
        heading_error = self.normalize_angle(target_heading - self.current_yaw)
        
        # Create velocity command
        cmd = Twist()
        
        # Angular velocity (proportional control)
        angular_vel = self.heading_gain * heading_error
        angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, angular_vel))
        cmd.angular.z = angular_vel
        
        # Linear velocity (reduce when turning sharply)
        turn_factor = 1.0 - min(1.0, abs(heading_error) / (math.pi / 2))
        cmd.linear.x = target.velocity * max(0.3, turn_factor)
        
        # Publish command
        self.cmd_vel_pub.publish(cmd)
        
        # Debug log
        self.get_logger().debug(
            f'WP{self.current_waypoint_idx}: dist={distance:.2f}m, '
            f'heading_err={math.degrees(heading_error):.1f}°, '
            f'v={cmd.linear.x:.2f}m/s, w={cmd.angular.z:.2f}rad/s'
        )
    
    def complete_mission(self) -> None:
        """Handle mission completion."""
        if self.mission_complete:
            return
        
        self.mission_complete = True
        
        # Stop the vehicle
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        
        self.get_logger().info('=== MISSION COMPLETE ===')
        self.get_logger().info('All waypoints reached. Stopping vehicle.')
        
        # Call completion callback
        if self.on_mission_complete:
            self.on_mission_complete()
    
    @staticmethod
    def normalize_angle(angle: float) -> float:
        """
        Normalize angle to [-pi, pi].
        
        Args:
            angle: Angle in radians
            
        Returns:
            Normalized angle
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def is_mission_complete(self) -> bool:
        """Check if mission is complete."""
        return self.mission_complete


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = UGVControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop vehicle before shutdown
        stop_cmd = Twist()
        node.cmd_vel_pub.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
