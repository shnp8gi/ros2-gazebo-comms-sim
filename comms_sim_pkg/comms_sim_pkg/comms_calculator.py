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
from typing import Tuple, Optional
import numpy as np


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
    
    # Throughput lookup table based on RSSI thresholds
    THROUGHPUT_TABLE = [
        (-51.0, 6.0),    # RSSI > -51.0 dBm -> 6.0 Gbps
        (-55.0, 4.7),    # -55.0 < RSSI <= -51.0 -> 4.7 Gbps
        (-58.5, 2.7),    # -58.5 < RSSI <= -55.0 -> 2.7 Gbps
        (-61.5, 2.15),   # -61.5 < RSSI <= -58.5 -> 2.15 Gbps
        (-63.5, 1.1),    # -63.5 < RSSI <= -61.5 -> 1.1 Gbps
        (-65.5, 0.5),    # -65.5 < RSSI <= -63.5 -> 0.5 Gbps
        (float('-inf'), 0.0)  # RSSI <= -65.5 -> 0.0 Gbps
    ]
    
    def __init__(
        self,
        propagation_model: Optional[PropagationModel] = None,
        tx_power_dbm: float = 20.0,
        noise_variance: float = 2.0
    ) -> None:
        """
        Initialize communication calculator.
        
        Args:
            propagation_model: Propagation model strategy (default: LogDistance)
            tx_power_dbm: Transmit power [dBm]
            noise_variance: AWGN variance [dB]
        """
        self.propagation_model = propagation_model or LogDistancePathLossModel()
        self.tx_power_dbm = tx_power_dbm
        self.noise_variance = noise_variance
        self._rng = np.random.default_rng()
    
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
        Determine throughput from RSSI using lookup table.
        
        Args:
            rssi: Received signal strength [dBm]
            
        Returns:
            Throughput [Gbps]
        """
        for threshold, throughput in self.THROUGHPUT_TABLE:
            if rssi > threshold:
                return throughput
        return 0.0
    
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
