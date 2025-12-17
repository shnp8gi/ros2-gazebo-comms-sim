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
    
    def calculate_angles_from_orientation(
        self,
        ugv_position: np.ndarray,
        base_station_position: np.ndarray,
        ugv_orientation_euler: np.ndarray
    ) -> Tuple[float, float]:
        """
        Calculate relative angles for antenna gain lookup based on UGV orientation.
        
        Args:
            ugv_position: UGV position [x, y, z]
            base_station_position: Base station position [x, y, z]
            ugv_orientation_euler: UGV orientation [roll, pitch, yaw] in radians
            
        Returns:
            Tuple of (elevation_angle, azimuth_angle) in degrees
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
