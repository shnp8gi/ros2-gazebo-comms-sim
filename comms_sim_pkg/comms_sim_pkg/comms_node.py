#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
# Communication Simulator Node
# ROS 2 node for simulating wireless communication quality
# =============================================================================
"""
ROS 2 node that simulates communication quality between UGV and base station.

Subscribes to:
    - /odom (nav_msgs/Odometry): UGV odometry (used to derive world position)
    - /gps/fix (sensor_msgs/NavSatFix): UGV GPS position (fallback)
    - /imu/data (sensor_msgs/Imu): UGV orientation

Publishes:
    - /comms/quality (comms_sim_msgs/CommsQuality): Communication metrics

Parameters:
    - sampling_rate: Calculation frequency [Hz]
    - noise_variance: AWGN variance [dB]
    - e_plane_path: E-plane antenna pattern CSV
    - h_plane_path: H-plane antenna pattern CSV
    - comm_data_limit_mb: Data transmission limit [Mb]
    - path_loss.*: Path loss model parameters
    - tx_power: Transmit power [dBm]
    - spawn_pose: UGV spawn pose [x, y, z] in world coordinates
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
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

# Prefer absolute imports so this file can be executed as a script via ros2 launch
try:
    from comms_sim_pkg.antenna_parser import AntennaPatternParser
    from comms_sim_pkg.comms_calculator import (
        CommsCalculator,
        LogDistancePathLossModel,
        TwoRayGroundModel,
    )
except ImportError:
    # Fallback for module execution (python -m comms_sim_pkg.comms_node)
    from .antenna_parser import AntennaPatternParser  # type: ignore
    from .comms_calculator import (  # type: ignore
        CommsCalculator,
        LogDistancePathLossModel,
        TwoRayGroundModel,
    )

# Import custom message (will be available after build)
try:
    from comms_sim_msgs.msg import CommsQuality
except ImportError:
    CommsQuality = None


def quaternion_to_euler(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    """
    Convert quaternion to Euler angles (roll, pitch, yaw).
    
    Args:
        x, y, z, w: Quaternion components
        
    Returns:
        Tuple of (roll, pitch, yaw) in radians
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)
    else:
        pitch = np.arcsin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw


class LinkState(Enum):
    """Communication link state enumeration."""
    DISCONNECTED = 0      # RSSI below threshold
    ESTABLISHING = 1      # Link establishment in progress
    CONNECTED = 2         # Link established and active


class CommsSimulatorNode(Node):
    """
    ROS 2 node for communication simulation.
    
    Calculates RSSI and throughput based on UGV position/orientation
    and base station location using configurable propagation models.
    
    Implements link establishment delay:
    - When RSSI crosses minimum threshold, wait for link establishment time
    - Only after establishment, data transmission begins
    """
    
    def __init__(self) -> None:
        super().__init__('comms_simulator_node')
        
        # =====================================================================
        # Declare parameters
        # =====================================================================
        self.declare_parameter('sampling_rate', 1.0)
        self.declare_parameter('noise_variance', 2.0)
        self.declare_parameter('e_plane_path', '')
        self.declare_parameter('h_plane_path', '')
        self.declare_parameter('comm_data_limit_mb', -1.0)
        self.declare_parameter('path_loss.d0', 1.0)
        self.declare_parameter('path_loss.pl0', 40.0)
        self.declare_parameter('path_loss.exponent', 2.0)
        self.declare_parameter('tx_power', -7.0)
        self.declare_parameter('base_station_position', [0.0, 0.0, 0.0])
        self.declare_parameter('base_station_antenna_offset', 10.5)
        self.declare_parameter('ugv_antenna_offset', 1.3)
        self.declare_parameter('mcs_table_path', '')
        self.declare_parameter('link_establishment_time_ms', 2.0)
        self.declare_parameter('spawn_pose', [0.0, 0.0, 0.0])
        
        # Get parameters
        self.sampling_rate = self.get_parameter('sampling_rate').value
        self.noise_variance = self.get_parameter('noise_variance').value
        self.e_plane_path = self.get_parameter('e_plane_path').value
        self.h_plane_path = self.get_parameter('h_plane_path').value
        self.comm_data_limit_mb = self.get_parameter('comm_data_limit_mb').value
        self.tx_power = self.get_parameter('tx_power').value
        self.mcs_table_path = self.get_parameter('mcs_table_path').value
        self.link_establishment_time = self.get_parameter('link_establishment_time_ms').value / 1000.0  # ms to s
        
        # Path loss parameters
        d0 = self.get_parameter('path_loss.d0').value
        pl0 = self.get_parameter('path_loss.pl0').value
        exponent = self.get_parameter('path_loss.exponent').value
        
        # =====================================================================
        # Initialize components
        # =====================================================================
        # Propagation model (Strategy pattern)
        self.propagation_model = LogDistancePathLossModel(
            d0=d0, pl0=pl0, exponent=exponent
        )
        
        # Communication calculator
        self.comms_calculator = CommsCalculator(
            propagation_model=self.propagation_model,
            tx_power_dbm=self.tx_power,
            noise_variance=self.noise_variance,
            mcs_table_path=self.mcs_table_path
        )
        
        # Get RSSI threshold from MCS table (minimum RSSI)
        self.rssi_threshold = self.comms_calculator.rssi_min
        
        # Antenna pattern parser
        self.antenna_parser = AntennaPatternParser()
        if self.e_plane_path:
            try:
                self.antenna_parser.load_e_plane(self.e_plane_path)
                self.get_logger().info(f'Loaded E-plane pattern: {self.e_plane_path}')
            except Exception as e:
                self.get_logger().warn(f'Failed to load E-plane: {e}')
        
        if self.h_plane_path:
            try:
                self.antenna_parser.load_h_plane(self.h_plane_path)
                self.get_logger().info(f'Loaded H-plane pattern: {self.h_plane_path}')
            except Exception as e:
                self.get_logger().warn(f'Failed to load H-plane: {e}')
        
        # =====================================================================
        # State variables
        # =====================================================================
        # Get base station position from parameters
        bs_pos = self.get_parameter('base_station_position').value
        self.base_station_position: Optional[np.ndarray] = np.array(bs_pos) if bs_pos else None
        self.base_station_antenna_offset: float = self.get_parameter('base_station_antenna_offset').value
        self.ugv_antenna_offset: float = self.get_parameter('ugv_antenna_offset').value
        
        self.ugv_gps_position: Optional[np.ndarray] = None
        self.ugv_orientation: Optional[np.ndarray] = None  # [roll, pitch, yaw]
        self.ugv_local_position: Optional[np.ndarray] = None
        self.odom_offset_set: bool = False
        self.odom_offset: np.ndarray = np.array([0.0, 0.0, 0.0])
        self.spawn_pose: np.ndarray = np.array(self.get_parameter('spawn_pose').value[:3])
        
        self.total_data_transmitted: float = 0.0  # [Mb]
        self.comm_active: bool = True
        self.simulation_start_time: Optional[float] = None
        
        # Link establishment state management
        self.link_state: LinkState = LinkState.DISCONNECTED
        self.link_establishment_start_time: Optional[float] = None
        
        # Data log for CSV output
        self.data_log: List[dict] = []
        
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

        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            sensor_qos
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            sensor_qos
        )
        
        # =====================================================================
        # Publishers
        # =====================================================================
        if CommsQuality is not None:
            self.quality_pub = self.create_publisher(
                CommsQuality,
                '/comms/quality',
                10
            )
        else:
            self.quality_pub = None
            self.get_logger().warn('CommsQuality message not available')
        
        # =====================================================================
        # Timer for periodic calculation
        # =====================================================================
        period = 1.0 / self.sampling_rate
        self.calc_timer = self.create_timer(period, self.calculate_and_publish)
        
        # =====================================================================
        # Register cleanup on exit
        # =====================================================================
        atexit.register(self.save_log_to_csv)
        
        self.get_logger().info(
            f'CommsSimulatorNode initialized\n'
            f'  Sampling rate: {self.sampling_rate} Hz\n'
            f'  TX Power: {self.tx_power} dBm\n'
            f'  Noise variance: {self.noise_variance} dB\n'
            f'  Link establishment time: {self.link_establishment_time*1000:.1f} ms\n'
            f'  RSSI threshold: {self.rssi_threshold:.1f} dBm (from MCS table)\n'
            f'  RSSI min: {self.comms_calculator.rssi_min:.1f} dBm\n'
            f'  RSSI max: {self.comms_calculator.rssi_max:.1f} dBm\n'
            f'  Data limit: {self.comm_data_limit_mb} Mb\n'
            f'  Model: {self.propagation_model.model_name}\n'
            f'  MCS table: {self.mcs_table_path}\n'
            f'  Spawn pose: {self.spawn_pose.tolist()}'
        )
    
    def set_base_station_position(
        self,
        position: List[float],
        antenna_offset: float
    ) -> None:
        """
        Set base station position from launch parameters.
        
        Args:
            position: [x, y, z] position
            antenna_offset: Antenna height offset
        """
        self.base_station_position = np.array(position[:3])
        self.base_station_antenna_offset = antenna_offset
        self.get_logger().info(
            f'Base station set: {position}, antenna offset: {antenna_offset}m'
        )
    
    def set_ugv_antenna_offset(self, offset: float) -> None:
        """
        Set UGV antenna height offset.
        
        Args:
            offset: Antenna height offset from UGV base
        """
        self.ugv_antenna_offset = offset
    
    def gps_callback(self, msg: NavSatFix) -> None:
        """
        Handle GPS position updates.
        
        Note: NavSat provides lat/lon/alt. For local simulation,
        we use a simplified conversion or rely on the bridge providing
        local coordinates via odometry.
        """
        # Store GPS coordinates (lat, lon, alt)
        self.ugv_gps_position = np.array([
            msg.latitude,
            msg.longitude,
            msg.altitude
        ])
        
        if self.simulation_start_time is None:
            self.simulation_start_time = self.get_clock().now().nanoseconds / 1e9

    def odom_callback(self, msg: Odometry) -> None:
        """Handle odometry updates and convert to world coordinates."""
        odom_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])

        if not self.odom_offset_set:
            self.odom_offset = self.spawn_pose - odom_pos
            self.odom_offset_set = True
            self.get_logger().info(
                f'Computed odom offset: '
                f'({self.odom_offset[0]:.3f}, {self.odom_offset[1]:.3f}, {self.odom_offset[2]:.3f})'
            )

        self.ugv_local_position = odom_pos + self.odom_offset

        if self.simulation_start_time is None:
            self.simulation_start_time = self.get_clock().now().nanoseconds / 1e9
    
    def imu_callback(self, msg: Imu) -> None:
        """Handle IMU orientation updates."""
        # Convert quaternion to Euler angles
        roll, pitch, yaw = quaternion_to_euler(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        self.ugv_orientation = np.array([roll, pitch, yaw])
    
    def set_ugv_local_position(self, position: np.ndarray) -> None:
        """
        Set UGV local position (from odometry or direct pose).
        
        Args:
            position: [x, y, z] in world coordinates
        """
        self.ugv_local_position = position
    
    def _update_link_state(self, rssi: float, current_time: float) -> bool:
        """
        Update link establishment state based on RSSI.
        
        State transitions:
        - DISCONNECTED -> ESTABLISHING: RSSI crosses above threshold
        - ESTABLISHING -> CONNECTED: Link establishment time elapsed
        - ESTABLISHING -> DISCONNECTED: RSSI drops below threshold
        - CONNECTED -> DISCONNECTED: RSSI drops below threshold
        
        Args:
            rssi: Current RSSI [dBm]
            current_time: Current simulation time [s]
            
        Returns:
            True if communication is allowed, False otherwise
        """
        if self.link_state == LinkState.DISCONNECTED:
            # Check if RSSI crossed above threshold
            if rssi > self.rssi_threshold:
                self.link_state = LinkState.ESTABLISHING
                self.link_establishment_start_time = current_time
                self.get_logger().info(
                    f'Link establishment started (RSSI: {rssi:.1f} dBm)'
                )
            return False
        
        elif self.link_state == LinkState.ESTABLISHING:
            # Check if RSSI dropped below threshold
            if rssi <= self.rssi_threshold:
                self.link_state = LinkState.DISCONNECTED
                self.link_establishment_start_time = None
                self.get_logger().info('Link establishment aborted (RSSI dropped)')
                return False
            
            # Check if establishment time elapsed
            elapsed = current_time - self.link_establishment_start_time
            if elapsed >= self.link_establishment_time:
                self.link_state = LinkState.CONNECTED
                self.get_logger().info(
                    f'Link established after {elapsed*1000:.1f} ms'
                )
                return True
            
            return False
        
        elif self.link_state == LinkState.CONNECTED:
            # Check if RSSI dropped below threshold
            if rssi <= self.rssi_threshold:
                self.link_state = LinkState.DISCONNECTED
                self.link_establishment_start_time = None
                self.get_logger().info('Link disconnected (RSSI dropped)')
                return False
            
            return True
        
        return False
    
    def calculate_and_publish(self) -> None:
        """Periodic callback to calculate and publish communication quality."""
        # Check if we have required data
        if self.base_station_position is None:
            self.get_logger().warn('Base station position not set', throttle_duration_sec=5.0)
            return
        
        if self.ugv_local_position is None and self.ugv_gps_position is None:
            self.get_logger().debug('Waiting for UGV position data...')
            return
        
        # Use local position if available, otherwise estimate from GPS
        if self.ugv_local_position is not None:
            ugv_pos = self.ugv_local_position.copy()
        else:
            # Simplified: treat GPS lat/lon as local X/Y (not accurate, for demo)
            ugv_pos = np.array([
                self.ugv_gps_position[0] * 111000,  # Rough lat to m
                self.ugv_gps_position[1] * 111000,  # Rough lon to m
                self.ugv_gps_position[2]
            ])
        
        # Apply antenna offsets
        ugv_antenna_pos = ugv_pos + np.array([0, 0, self.ugv_antenna_offset])
        bs_antenna_pos = self.base_station_position + np.array([
            0, 0, self.base_station_antenna_offset
        ])
        
        # Calculate antenna gains based on orientation
        antenna_gain = 0.0
        e_gain, h_gain = 0.0, 0.0
        
        if self.ugv_orientation is not None:
            elevation, azimuth = self.antenna_parser.calculate_angles_from_orientation(
                ugv_antenna_pos, bs_antenna_pos, self.ugv_orientation
            )
            e_gain, h_gain, antenna_gain = self.antenna_parser.get_combined_gain(
                elevation, azimuth
            )
        
        # Calculate communication metrics
        metrics = self.comms_calculator.calculate_all(
            ugv_antenna_pos,
            bs_antenna_pos,
            antenna_gain_db=antenna_gain,
            add_noise=True
        )
        
        # Get current time
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed = current_time - (self.simulation_start_time or current_time)
        
        # Update link state based on RSSI
        link_ready = self._update_link_state(metrics['rssi'], current_time)
        
        # Calculate actual throughput (considering link state)
        actual_throughput = 0.0
        if link_ready and self.comm_active:
            actual_throughput = metrics['throughput']
            
            # Update total data transmitted
            if metrics['throughput'] > 0:
                # Convert Gbps to Mb for the sampling period
                period = 1.0 / self.sampling_rate
                data_this_period = metrics['throughput'] * 1000 * period  # Gbps * s = Gb -> Mb
                self.total_data_transmitted += data_this_period
                
                # Check data limit
                if self.comm_data_limit_mb > 0:
                    if self.total_data_transmitted >= self.comm_data_limit_mb:
                        self.comm_active = False
                        self.get_logger().info(
                            f'Data limit reached: {self.total_data_transmitted:.2f} Mb'
                        )
        
        # Log data
        log_entry = {
            'time_s': elapsed,
            'ugv_x': ugv_pos[0],
            'ugv_y': ugv_pos[1],
            'ugv_z': ugv_pos[2],
            'bs_x': self.base_station_position[0],
            'bs_y': self.base_station_position[1],
            'bs_z': self.base_station_position[2] + self.base_station_antenna_offset,
            'distance': metrics['distance'],
            'rssi': metrics['rssi'],
            'throughput': actual_throughput,
            'total_data_mb': self.total_data_transmitted,
            'path_loss': metrics['path_loss'],
            'e_gain': e_gain,
            'h_gain': h_gain,
            'link_state': self.link_state.name,
        }
        self.data_log.append(log_entry)
        
        # Publish
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
            msg.base_station_z = self.base_station_position[2] + self.base_station_antenna_offset
            msg.antenna_gain_e_plane = e_gain
            msg.antenna_gain_h_plane = h_gain
            msg.path_loss = metrics['path_loss']
            msg.comm_active = self.comm_active
            
            self.quality_pub.publish(msg)
        
        # Log info
        link_status = f"[{self.link_state.name}]"
        self.get_logger().info(
            f'[{elapsed:.1f}s] {link_status} D={metrics["distance"]:.1f}m, '
            f'RSSI={metrics["rssi"]:.1f}dBm, '
            f'TP={actual_throughput:.2f}Gbps, '
            f'Total={self.total_data_transmitted:.1f}Mb',
            throttle_duration_sec=1.0
        )
    
    def save_log_to_csv(self) -> None:
        """Save collected data to CSV file."""
        if not self.data_log:
            self.get_logger().info('No data to save')
            return
        
        # Generate filename
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        if self.comm_data_limit_mb > 0:
            limit_str = f'LIMIT-{self.comm_data_limit_mb:.0f}MB'
        else:
            limit_str = 'LIMIT-UNLIMITED'
        
        filename = f'{timestamp}_{limit_str}.csv'
        output_dir = '/workspace/log/sim_result'
        
        # Create directory if needed
        os.makedirs(output_dir, exist_ok=True)
        filepath = os.path.join(output_dir, filename)
        
        # Write CSV
        try:
            with open(filepath, 'w', newline='', encoding='utf-8') as f:
                # Write header comment
                f.write(f'# Communication Simulation Results\n')
                f.write(f'# Data Limit: {self.comm_data_limit_mb} Mb\n')
                f.write(f'# Model: {self.propagation_model.model_name}\n')
                f.write(f'# TX Power: {self.tx_power} dBm\n')
                f.write(f'# Noise Variance: {self.noise_variance} dB\n')
                f.write('#\n')
                
                # Write data
                fieldnames = [
                    'time_s', 'ugv_x', 'ugv_y', 'ugv_z',
                    'bs_x', 'bs_y', 'bs_z',
                    'distance', 'rssi', 'throughput', 'total_data_mb',
                    'path_loss', 'e_gain', 'h_gain', 'link_state'
                ]
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(self.data_log)
            
            self.get_logger().info(f'Data saved to: {filepath}')
        except Exception as e:
            self.get_logger().error(f'Failed to save CSV: {e}')


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = CommsSimulatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_log_to_csv()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
