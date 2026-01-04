#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
# Antenna Pattern Parser
# Parses E-plane and H-plane antenna gain CSV files
# =============================================================================
"""
Antenna pattern parser for communication simulation.
Reads CSV files containing antenna gain patterns and provides
interpolated gain values for arbitrary angles.
"""

from typing import Tuple, Optional
import numpy as np
from scipy.interpolate import interp1d


class AntennaPatternParser:
    """
    Parser for antenna radiation pattern CSV files.
    
    Provides linear interpolation of gain values for E-plane and H-plane
    patterns based on angle.
    
    Attributes:
        e_plane_interp: Interpolation function for E-plane gain
        h_plane_interp: Interpolation function for H-plane gain
    """
    
    def __init__(
        self,
        e_plane_path: Optional[str] = None,
        h_plane_path: Optional[str] = None
    ) -> None:
        """
        Initialize the antenna pattern parser.
        
        Args:
            e_plane_path: Path to E-plane gain CSV file
            h_plane_path: Path to H-plane gain CSV file
        """
        self.e_plane_interp: Optional[interp1d] = None
        self.h_plane_interp: Optional[interp1d] = None
        
        if e_plane_path:
            self.load_e_plane(e_plane_path)
        if h_plane_path:
            self.load_h_plane(h_plane_path)
    
    def _load_pattern(self, filepath: str) -> interp1d:
        """
        Load antenna pattern from CSV file.
        
        Expected CSV format:
        - Two columns: angle,gain (header row optional)
        - Angle in degrees (e.g., -90 to +90), gain in dBi
        - Lines starting with '#' are treated as comments
        - 0.1 degree resolution (1801 rows for -90 to +90)
        
        Args:
            filepath: Path to CSV file
            
        Returns:
            Interpolation function for the pattern
            
        Raises:
            FileNotFoundError: If file does not exist
            ValueError: If file format is invalid
        """
        angles = []
        gains = []
        
        with open(filepath, 'r', encoding='utf-8') as f:
            header_found = False
            for line in f:
                line = line.strip()
                
                # Skip empty lines and comments
                if not line or line.startswith('#'):
                    continue
                
                # Skip header row
                if not header_found and 'angle' in line.lower():
                    header_found = True
                    continue
                
                # Parse data
                parts = line.split(',')
                if len(parts) >= 2:
                    try:
                        angle = float(parts[0].strip())
                        # Handle negative sign that might be encoded differently
                        gain_str = parts[1].strip().replace('−', '-')
                        gain = float(gain_str)
                        angles.append(angle)
                        gains.append(gain)
                    except ValueError:
                        continue
        
        if not angles:
            raise ValueError(f"No valid data found in {filepath}")
        
        # Convert to numpy arrays
        angles = np.array(angles)
        gains = np.array(gains)
        
        # Sort by angle
        sort_idx = np.argsort(angles)
        angles = angles[sort_idx]
        gains = gains[sort_idx]
        
        # Create interpolation function with extrapolation
        return interp1d(
            angles, gains,
            kind='linear',
            bounds_error=False,
            fill_value=(gains[0], gains[-1])
        )
    
    def load_e_plane(self, filepath: str) -> None:
        """
        Load E-plane antenna pattern.
        
        Args:
            filepath: Path to E-plane CSV file
        """
        self.e_plane_interp = self._load_pattern(filepath)
    
    def load_h_plane(self, filepath: str) -> None:
        """
        Load H-plane antenna pattern.
        
        Args:
            filepath: Path to H-plane CSV file
        """
        self.h_plane_interp = self._load_pattern(filepath)
    
    def get_e_plane_gain(self, angle_deg: float) -> float:
        """
        Get E-plane antenna gain for given angle.
        
        Args:
            angle_deg: Angle in degrees (-180 to 180)
            
        Returns:
            Gain in dBi
        """
        if self.e_plane_interp is None:
            return 0.0
        return float(self.e_plane_interp(angle_deg))
    
    def get_h_plane_gain(self, angle_deg: float) -> float:
        """
        Get H-plane antenna gain for given angle.
        
        Args:
            angle_deg: Angle in degrees (-180 to 180)
            
        Returns:
            Gain in dBi
        """
        if self.h_plane_interp is None:
            return 0.0
        return float(self.h_plane_interp(angle_deg))
    
    def get_combined_gain(
        self,
        elevation_deg: float,
        azimuth_deg: float
    ) -> Tuple[float, float, float]:
        """
        Get combined antenna gain from E-plane (elevation) and H-plane (azimuth).
        
        Args:
            elevation_deg: Elevation angle in degrees (E-plane)
            azimuth_deg: Azimuth angle in degrees (H-plane)
            
        Returns:
            Tuple of (e_plane_gain, h_plane_gain, combined_gain) in dBi
        """
        e_gain = self.get_e_plane_gain(elevation_deg)
        h_gain = self.get_h_plane_gain(azimuth_deg)
        
        # Combined gain (simplified model: sum in dB domain)
        # More accurate models would use 3D pattern data
        combined = e_gain + h_gain
        
        return e_gain, h_gain, combined

    @staticmethod
    def _rpy_to_rotmat(roll: float, pitch: float, yaw: float) -> np.ndarray:
        """Return rotation matrix R = Rz(yaw) * Ry(pitch) * Rx(roll).

        This maps vectors from antenna/body frame -> world frame.
        """
        cr, sr = float(np.cos(roll)), float(np.sin(roll))
        cp, sp = float(np.cos(pitch)), float(np.sin(pitch))
        cy, sy = float(np.cos(yaw)), float(np.sin(yaw))

        rx = np.array(
            [[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]],
            dtype=float,
        )
        ry = np.array(
            [[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]],
            dtype=float,
        )
        rz = np.array(
            [[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]],
            dtype=float,
        )
        return rz @ ry @ rx

    @staticmethod
    def _wrap_pi(angle_rad: float) -> float:
        return float((angle_rad + np.pi) % (2.0 * np.pi) - np.pi)

    def calculate_antenna_frame_angles(
        self,
        antenna_pos_world: np.ndarray,
        target_pos_world: np.ndarray,
        antenna_rpy_world: np.ndarray,
    ) -> Tuple[float, float]:
        """Compute (elevation_rad, azimuth_rad) of target direction in antenna frame.

        - elevation: +up (asin(z))
        - azimuth: atan2(y, x)

        `antenna_rpy_world` is the antenna frame orientation w.r.t world.
        """
        v_world = np.asarray(target_pos_world, dtype=float) - np.asarray(antenna_pos_world, dtype=float)
        norm = float(np.linalg.norm(v_world))
        if norm <= 1e-12:
            return 0.0, 0.0

        v_world /= norm

        r = self._rpy_to_rotmat(
            float(antenna_rpy_world[0]),
            float(antenna_rpy_world[1]),
            float(antenna_rpy_world[2]),
        )

        # world -> antenna is inverse rotation (transpose).
        v_ant = r.T @ v_world

        az = float(np.arctan2(v_ant[1], v_ant[0]))
        el = float(np.arcsin(np.clip(v_ant[2], -1.0, 1.0)))
        return el, self._wrap_pi(az)

    def get_gain_from_angles(self, elevation_rad: float, azimuth_rad: float) -> Tuple[float, float, float]:
        """Lookup (E/H/total) gains using antenna-frame angles (radians)."""
        elevation_deg = float(np.degrees(elevation_rad))
        azimuth_deg = float(np.degrees(azimuth_rad))

        e_gain = float(self.get_e_plane_gain(elevation_deg))
        h_gain = float(self.get_h_plane_gain(azimuth_deg))
        return e_gain, h_gain, e_gain + h_gain

    def get_tx_rx_gains(
        self,
        tx_pos_world: np.ndarray,
        tx_rpy_world: np.ndarray,
        rx_pos_world: np.ndarray,
        rx_rpy_world: np.ndarray,
    ) -> Tuple[float, float, float, float, float, float]:
        """Compute separate antenna gains for TX and RX.

        Returns:
          (tx_e, tx_h, tx_total, rx_e, rx_h, rx_total) [dB]
        """
        tx_el, tx_az = self.calculate_antenna_frame_angles(tx_pos_world, rx_pos_world, tx_rpy_world)
        rx_el, rx_az = self.calculate_antenna_frame_angles(rx_pos_world, tx_pos_world, rx_rpy_world)

        tx_e, tx_h, tx_total = self.get_gain_from_angles(tx_el, tx_az)
        rx_e, rx_h, rx_total = self.get_gain_from_angles(rx_el, rx_az)

        return tx_e, tx_h, tx_total, rx_e, rx_h, rx_total

    def calculate_angles_from_orientation(
        self,
        ugv_position: np.ndarray,
        base_station_position: np.ndarray,
        ugv_orientation_euler: np.ndarray
    ) -> Tuple[float, float]:
        """Backward-compatible API.

        NOTE: This older helper only uses yaw for azimuth correction and does not
        fully reflect roll/pitch. Prefer `calculate_antenna_frame_angles()`.
        """
        # Vector from UGV to base station
        direction = base_station_position - ugv_position
        
        # Calculate distance in XY plane
        horizontal_dist = np.sqrt(direction[0]**2 + direction[1]**2)
        
        # Elevation angle (vertical angle to base station)
        elevation_rad = np.arctan2(direction[2], horizontal_dist)
        
        # Azimuth angle (horizontal angle to base station)
        azimuth_to_bs = np.arctan2(direction[1], direction[0])
        
        # Relative azimuth considering UGV yaw
        yaw = ugv_orientation_euler[2]
        relative_azimuth = azimuth_to_bs - yaw
        
        # Normalize to -180 to 180 degrees
        elevation_deg = np.degrees(elevation_rad)
        azimuth_deg = np.degrees(relative_azimuth)
        
        # Wrap azimuth to -180 to 180
        while azimuth_deg > 180:
            azimuth_deg -= 360
        while azimuth_deg < -180:
            azimuth_deg += 360
        
        return elevation_deg, azimuth_deg
