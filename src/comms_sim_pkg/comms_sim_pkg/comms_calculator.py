#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
# Communication Calculator Module
# Implements propagation models using Strategy Pattern
# =============================================================================
"""
Communication calculator for RSSI and throughput estimation.
Uses Strategy pattern for swappable propagation models.
"""

from abc import ABC, abstractmethod
from typing import Tuple, Optional, List
import numpy as np
import csv
import os


# =============================================================================
# Strategy Interface: Propagation Model
# =============================================================================
class PropagationModel(ABC):
    """
    Abstract base class for propagation models (Strategy Pattern).
    
    Implement this interface to create new propagation models
    (e.g., NLOS, reflection models).
    """
    
    @abstractmethod
    def calculate_path_loss(
        self,
        distance: float,
        frequency_ghz: float = 60.0
    ) -> float:
        """
        Calculate path loss for given distance.
        
        Args:
            distance: Distance between transmitter and receiver [m]
            frequency_ghz: Operating frequency [GHz]
            
        Returns:
            Path loss in dB
        """
        pass
    
    @property
    @abstractmethod
    def model_name(self) -> str:
        """Return the name of the propagation model."""
        pass


# =============================================================================
# Concrete Strategy: Log-Distance Path Loss Model
# =============================================================================
class LogDistancePathLossModel(PropagationModel):
    """
    Log-distance path loss model.
    
    PL(d) = PL(d0) + 10 * n * log10(d / d0)
    
    Where:
        PL(d): Path loss at distance d [dB]
        PL(d0): Path loss at reference distance d0 [dB]
        n: Path loss exponent
        d: Distance [m]
        d0: Reference distance [m]
    
    Typical values for n:
        - Free space: 2.0
        - Urban area: 2.7 - 3.5
        - Indoor LOS: 1.6 - 1.8
        - Indoor NLOS: 4 - 6
    """
    
    def __init__(
        self,
        d0: float = 1.0,
        pl0: float = 40.0,
        exponent: float = 2.0
    ) -> None:
        """
        Initialize log-distance path loss model.
        
        Args:
            d0: Reference distance [m]
            pl0: Path loss at reference distance [dB]
            exponent: Path loss exponent (n)
        """
        self.d0 = d0
        self.pl0 = pl0
        self.exponent = exponent
    
    def calculate_path_loss(
        self,
        distance: float,
        frequency_ghz: float = 60.0
    ) -> float:
        """
        Calculate path loss using log-distance model.
        
        Args:
            distance: Distance [m]
            frequency_ghz: Operating frequency [GHz] (not used in basic model)
            
        Returns:
            Path loss [dB]
        """
        if distance <= 0:
            return 0.0
        
        if distance < self.d0:
            distance = self.d0
        
        path_loss = self.pl0 + 10 * self.exponent * np.log10(distance / self.d0)
        return path_loss
    
    @property
    def model_name(self) -> str:
        return "Log-Distance Path Loss Model"


# =============================================================================
# Concrete Strategy: Two-Ray Ground Reflection Model
# =============================================================================
class TwoRayGroundModel(PropagationModel):
    """
    Two-Ray Ground Reflection Model.
    
    Suitable for longer distances where ground reflection becomes significant.
    
    PL(d) = 40 * log10(d) - 10 * log10(Gt * Gr * ht^2 * hr^2)
    
    Where:
        d: Distance [m]
        Gt, Gr: Antenna gains (linear)
        ht, hr: Antenna heights [m]
    """
    
    def __init__(
        self,
        tx_height: float = 10.5,
        rx_height: float = 1.3,
        tx_gain_db: float = 0.0,
        rx_gain_db: float = 0.0
    ) -> None:
        """
        Initialize two-ray ground model.
        
        Args:
            tx_height: Transmitter antenna height [m]
            rx_height: Receiver antenna height [m]
            tx_gain_db: Transmitter antenna gain [dBi]
            rx_gain_db: Receiver antenna gain [dBi]
        """
        self.tx_height = tx_height
        self.rx_height = rx_height
        self.tx_gain_linear = 10 ** (tx_gain_db / 10)
        self.rx_gain_linear = 10 ** (rx_gain_db / 10)
    
    def calculate_path_loss(
        self,
        distance: float,
        frequency_ghz: float = 60.0
    ) -> float:
        """
        Calculate path loss using two-ray model.
        
        Args:
            distance: Distance [m]
            frequency_ghz: Operating frequency [GHz]
            
        Returns:
            Path loss [dB]
        """
        if distance <= 0:
            return 0.0
        
        # Crossover distance (where two-ray model becomes valid)
        wavelength = 0.3 / frequency_ghz  # c / f in meters
        crossover = (4 * np.pi * self.tx_height * self.rx_height) / wavelength
        
        if distance < crossover:
            # Use free-space model for short distances
            fspl = 20 * np.log10(distance) + 20 * np.log10(frequency_ghz) + 92.45
            return fspl
        
        # Two-ray model
        gain_term = 10 * np.log10(
            self.tx_gain_linear * self.rx_gain_linear *
            (self.tx_height ** 2) * (self.rx_height ** 2)
        )
        path_loss = 40 * np.log10(distance) - gain_term
        
        return max(path_loss, 0.0)
    
    @property
    def model_name(self) -> str:
        return "Two-Ray Ground Reflection Model"


# =============================================================================
# Communication Calculator (Context)
# =============================================================================
class CommsCalculator:
    """
    Communication quality calculator.
    
    Calculates RSSI and throughput based on propagation model,
    antenna gains, and noise.
    
    Uses Strategy pattern to allow swapping propagation models.
    """
    
    def __init__(
        self,
        propagation_model: Optional[PropagationModel] = None,
        tx_power_dbm: float = 20.0,
        noise_variance: float = 2.0,
        mcs_table_path: Optional[str] = None
    ) -> None:
        """
        Initialize communication calculator.
        
        Args:
            propagation_model: Propagation model strategy (default: LogDistance)
            tx_power_dbm: Transmit power [dBm]
            noise_variance: AWGN variance [dB]
            mcs_table_path: Path to MCS table CSV file
        """
        self.propagation_model = propagation_model or LogDistancePathLossModel()
        self.tx_power_dbm = tx_power_dbm
        self.noise_variance = noise_variance
        self._rng = np.random.default_rng()
        
        # Load MCS table
        self.mcs_rssi: List[float] = []
        self.mcs_throughput: List[float] = []
        
        # RSSI thresholds (will be set from MCS table)
        self.rssi_min: float = 0.0
        self.rssi_max: float = 0.0
        
        if mcs_table_path:
            self._load_mcs_table(mcs_table_path)
        else:
            # Default MCS table if not provided
            self._init_default_mcs_table()
        
        # Set RSSI thresholds from loaded MCS table
        if self.mcs_rssi:
            self.rssi_min = min(self.mcs_rssi)
            self.rssi_max = max(self.mcs_rssi)
        else:
            raise ValueError("MCS table is empty. Cannot determine RSSI thresholds.")
    
    def _load_mcs_table(self, csv_path: str) -> None:
        """
        Load MCS table from CSV file.
        
        CSV format:
            RSSI [dBm], Throughput [Gbps]
            -39, 13.1413
            -45, 9.856
            ...
        
        Args:
            csv_path: Path to MCS table CSV file
            
        Raises:
            FileNotFoundError: If CSV file doesn't exist
            ValueError: If CSV format is invalid
        """
        if not os.path.exists(csv_path):
            raise FileNotFoundError(
                f"\n{'='*70}\n"
                f"ERROR: MCS table file not found!\n"
                f"{'='*70}\n"
                f"Expected location: {csv_path}\n"
                f"Please ensure MCStable.csv exists in the config directory.\n"
                f"{'='*70}\n"
            )
        
        rssi_list = []
        throughput_list = []
        
        try:
            with open(csv_path, 'r', encoding='utf-8') as f:
                reader = csv.reader(f)
                for row in reader:
                    # Skip empty lines and comments
                    if not row or row[0].strip().startswith(('#', '//')):
                        continue
                    
                    if len(row) < 2:
                        continue
                    
                    try:
                        rssi = float(row[0].strip())
                        throughput = float(row[1].strip())
                        rssi_list.append(rssi)
                        throughput_list.append(throughput)
                    except ValueError:
                        continue
            
            if not rssi_list:
                raise ValueError("No valid data found in MCS table")
            
            # Sort by RSSI (ascending order for interpolation)
            sorted_pairs = sorted(zip(rssi_list, throughput_list))
            self.mcs_rssi = [pair[0] for pair in sorted_pairs]
            self.mcs_throughput = [pair[1] for pair in sorted_pairs]
            
        except Exception as e:
            raise ValueError(
                f"\n{'='*70}\n"
                f"ERROR: Failed to load MCS table!\n"
                f"{'='*70}\n"
                f"File: {csv_path}\n"
                f"Error: {e}\n"
                f"{'='*70}\n"
            )
    
    def _init_default_mcs_table(self) -> None:
        """Initialize default MCS table (fallback)."""
        self.mcs_rssi = [-61, -58, -55, -51, -45, -39]
        self.mcs_throughput = [2.5813, 3.2853, 5.1627, 6.5707, 9.856, 13.1413]
    
    def set_propagation_model(self, model: PropagationModel) -> None:
        """
        Set propagation model (Strategy pattern).
        
        Args:
            model: New propagation model to use
        """
        self.propagation_model = model
    
    def set_noise_variance(self, variance: float) -> None:
        """
        Update AWGN variance for dynamic noise.
        
        Args:
            variance: New noise variance [dB]
        """
        self.noise_variance = variance
    
    def calculate_distance(
        self,
        ugv_position: np.ndarray,
        base_station_position: np.ndarray
    ) -> float:
        """
        Calculate 3D Euclidean distance.
        
        Args:
            ugv_position: UGV position [x, y, z]
            base_station_position: Base station position [x, y, z]
            
        Returns:
            Distance [m]
        """
        return float(np.linalg.norm(ugv_position - base_station_position))
    
    def calculate_rssi(
        self,
        distance: float,
        antenna_gain_db: float = 0.0,
        add_noise: bool = True
    ) -> Tuple[float, float]:
        """
        Calculate RSSI with optional AWGN.
        
        RSSI = Tx_Power - Path_Loss + Antenna_Gain + Noise
        
        Args:
            distance: Distance [m]
            antenna_gain_db: Combined antenna gain [dBi]
            add_noise: Whether to add AWGN
            
        Returns:
            Tuple of (rssi [dBm], path_loss [dB])
        """
        # Calculate path loss
        path_loss = self.propagation_model.calculate_path_loss(distance)
        
        # Base RSSI
        rssi = self.tx_power_dbm - path_loss + antenna_gain_db
        
        # Add AWGN
        if add_noise and self.noise_variance > 0:
            noise = self._rng.normal(0, np.sqrt(self.noise_variance))
            rssi += noise
        
        return rssi, path_loss
    
    def calculate_throughput(self, rssi: float) -> float:
        """
        Calculate throughput from RSSI using linear interpolation on MCS table.
        
        Rules:
        - RSSI <= rssi_min: 0 Gbps (no communication)
        - rssi_min < RSSI < rssi_max: Linear interpolation
        - RSSI >= rssi_max: Maximum throughput (saturated)
        
        Args:
            rssi: Received signal strength [dBm]
            
        Returns:
            Throughput [Gbps]
        """
        # Below minimum threshold: no communication
        if rssi <= self.rssi_min:
            return 0.0
        
        # Above maximum threshold: saturate at max throughput
        if rssi >= self.rssi_max:
            return self.mcs_throughput[-1]
        
        # Linear interpolation between MCS table points
        throughput = np.interp(rssi, self.mcs_rssi, self.mcs_throughput)
        
        return float(throughput)
    
    def calculate_all(
        self,
        ugv_position: np.ndarray,
        base_station_position: np.ndarray,
        antenna_gain_db: float = 0.0,
        add_noise: bool = True
    ) -> dict:
        """
        Calculate all communication metrics.
        
        Args:
            ugv_position: UGV position [x, y, z]
            base_station_position: Base station position [x, y, z]
            antenna_gain_db: Combined antenna gain [dBi]
            add_noise: Whether to add AWGN
            
        Returns:
            Dictionary with all metrics
        """
        distance = self.calculate_distance(ugv_position, base_station_position)
        rssi, path_loss = self.calculate_rssi(distance, antenna_gain_db, add_noise)
        throughput = self.calculate_throughput(rssi)
        
        return {
            'distance': distance,
            'rssi': rssi,
            'path_loss': path_loss,
            'throughput': throughput,
            'model_name': self.propagation_model.model_name
        }
