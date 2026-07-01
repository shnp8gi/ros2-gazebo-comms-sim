import numpy as np
from itertools import permutations
from typing import List, Tuple, Dict, Optional, Any

from comms_sim_pkg.antenna_parser import AntennaPatternParser
from comms_sim_pkg.comms_calculator import CommsCalculator


class PairAllocationOptimizer:
    """
    与えられたRSSI行列に基づき、ヒステリシスマージンを考慮した
    最適なアンテナペアの割り当てを計算する純粋なアルゴリズムクラス。
    """
    
    @staticmethod
    def get_best_allocation(
        rssi_matrix: List[List[float]],
        num_pairs: int,
        vehicle_antennas: List[Dict[str, Any]],
        rx_nodes: List[Dict[str, Any]],
        prev_perm: Optional[Tuple[int, ...]] = None,
        ff_hysteresis_margin_db: float = 0.0
    ) -> Tuple[Optional[Tuple[int, ...]], List[Tuple[str, str, float]], float]:
        """
        最適なペア割り当てを計算する。
        
        Args:
            rssi_matrix: [tx_idx][rx_idx] に対応するRSSI値の2次元リスト
            num_pairs: 割り当てるペアの数
            vehicle_antennas: 送信（車両）アンテナ情報のリスト
            rx_nodes: 受信（基地局）ノード情報のリスト
            prev_perm: 直前の割り当て結果 (rx_idxのタプル)
            ff_hysteresis_margin_db: ヒステリシスマージン [dB]
            
        Returns:
            (best_perm, best_pairs_sorted, max_rssi)
        """
        num_tx = len(vehicle_antennas)
        num_rx = len(rx_nodes)
        
        best_total_rssi = -1e9
        best_pairs = []
        best_perm = None
        
        # 直前の割り当てが存在する場合、ヒステリシスマージンを加算したものをベースラインとする
        if prev_perm is not None:
            prev_total_rssi = sum(rssi_matrix[tx_idx][prev_perm[tx_idx]] for tx_idx in range(num_pairs))
            best_total_rssi = prev_total_rssi + ff_hysteresis_margin_db
            best_pairs = [
                (vehicle_antennas[tx_idx]['name'], rx_nodes[prev_perm[tx_idx]]['name'], rssi_matrix[tx_idx][prev_perm[tx_idx]])
                for tx_idx in range(num_pairs)
            ]
            best_perm = prev_perm
            
        # 全列挙で評価
        for perm in permutations(range(num_rx)):
            if perm == prev_perm:
                continue
                
            total_rssi = sum(rssi_matrix[tx_idx][perm[tx_idx]] for tx_idx in range(num_pairs))
            if total_rssi > best_total_rssi:
                best_total_rssi = total_rssi
                best_pairs = [
                    (vehicle_antennas[tx_idx]['name'], rx_nodes[perm[tx_idx]]['name'], rssi_matrix[tx_idx][perm[tx_idx]])
                    for tx_idx in range(num_pairs)
                ]
                best_perm = perm
                
        # RSSIの高い順にソート
        best_pairs_sorted = sorted(best_pairs, key=lambda p: p[2], reverse=True)
        max_rssi = best_pairs_sorted[0][2] if best_pairs_sorted else -999.0
        
        return best_perm, best_pairs_sorted, max_rssi


class FeedforwardLutGenerator:
    """
    軌道情報と電波モデルを用いて各地点のRSSIを計算し、
    PairAllocationOptimizerを利用してLUTを生成するオーケストレータークラス。
    """
    
    def __init__(
        self,
        parser: AntennaPatternParser,
        calculator: CommsCalculator,
        vehicle_antennas: List[Dict[str, Any]],
        rx_nodes: List[Dict[str, Any]],
        ff_max_pairs: int = -1,
        ff_hysteresis_margin_db: float = 0.0,
        filter_main_lobe: bool = True
    ):
        self.parser = parser
        self.calculator = calculator
        self.vehicle_antennas = vehicle_antennas
        self.rx_nodes = rx_nodes
        self.ff_max_pairs = ff_max_pairs
        self.ff_hysteresis_margin_db = ff_hysteresis_margin_db
        self.filter_main_lobe = filter_main_lobe
        
        num_tx = len(self.vehicle_antennas)
        num_rx = len(self.rx_nodes)
        self.num_pairs = min(num_tx, num_rx)
        if self.ff_max_pairs > 0:
            self.num_pairs = min(self.num_pairs, self.ff_max_pairs)

    def generate(self, samples: List[Tuple[np.ndarray, float]]) -> Tuple[List[Any], List[Dict[str, Any]]]:
        """
        LUTとヒートマップログデータを生成する。
        
        Args:
            samples: 軌道のサンプリングポイント (position, yaw) のリスト
            
        Returns:
            (lut, heatmap_rows)
        """
        lut = []
        heatmap_rows = []
        prev_perm = None
        
        for pt, yaw in samples:
            tx_orientation = np.array([0.0, 0.0, yaw])
            
            row = {
                'x_m': round(pt[0], 4),
                'y_m': round(pt[1], 4),
                'z_m': round(pt[2], 4),
                'yaw_rad': round(yaw, 4)
            }
            
            # Step 1: 全 (TX, RX) 組み合わせのRSSIを計算
            R_veh = self.parser._rpy_to_rotmat(tx_orientation[0], tx_orientation[1], tx_orientation[2])
            num_tx = len(self.vehicle_antennas)
            num_rx = len(self.rx_nodes)
            rssi_matrix = [[-999.0] * num_rx for _ in range(num_tx)]
            
            for tx_idx, va in enumerate(self.vehicle_antennas):
                tx_antenna_pos = pt + R_veh.dot(np.asarray(va['offset'], dtype=float))
                tx_ant_rpy = tx_orientation + np.asarray(va['relative_rpy'], dtype=float)
                
                for rx_idx, bs in enumerate(self.rx_nodes):
                    bs_antenna_pos = bs['position'] + bs['antenna_offset']
                    bs_ant_rpy = bs['rpy'] + bs['antenna_relative_rpy']
                    
                    tx_e, tx_h, tx_total, rx_e, rx_h, rx_total = self.parser.get_tx_rx_gains(
                        tx_pos_world=bs_antenna_pos,
                        tx_rpy_world=bs_ant_rpy,
                        rx_pos_world=tx_antenna_pos,
                        rx_rpy_world=tx_ant_rpy,
                    )
                    antenna_gain_db = float(tx_total + rx_total)
                    
                    in_main = True
                    if self.filter_main_lobe:
                        tx_el, tx_az = self.parser.calculate_antenna_frame_angles(bs_antenna_pos, tx_antenna_pos, bs_ant_rpy)
                        rx_el, rx_az = self.parser.calculate_antenna_frame_angles(tx_antenna_pos, bs_antenna_pos, tx_ant_rpy)
                        tx_in = self.parser.is_in_main_lobe(np.degrees(abs(tx_el)), np.degrees(abs(tx_az)))
                        rx_in = self.parser.is_in_main_lobe(np.degrees(abs(rx_el)), np.degrees(abs(rx_az)))
                        if not (tx_in and rx_in):
                            in_main = False

                    if in_main:
                        metrics = self.calculator.calculate_all(
                            tx_antenna_pos,
                            bs_antenna_pos,
                            antenna_gain_db=antenna_gain_db,
                            add_noise=False
                        )
                        rssi_matrix[tx_idx][rx_idx] = metrics['rssi']
                    
                    col_name = f"rssi_{bs['name']}_{va['name']}"
                    row[col_name] = round(rssi_matrix[tx_idx][rx_idx], 2)
            
            # Step 2: 最適ペアの決定 (ヒステリシス適用)
            best_perm, best_pairs_sorted, max_rssi = PairAllocationOptimizer.get_best_allocation(
                rssi_matrix=rssi_matrix,
                num_pairs=self.num_pairs,
                vehicle_antennas=self.vehicle_antennas,
                rx_nodes=self.rx_nodes,
                prev_perm=prev_perm,
                ff_hysteresis_margin_db=self.ff_hysteresis_margin_db
            )
            
            prev_perm = best_perm
            
            row['optimal_pairs'] = ';'.join(f"{p[0]}:{p[1]}" for p in best_pairs_sorted)
            row['max_rssi'] = round(max_rssi, 2)
            heatmap_rows.append(row)
            
            lut.append((pt[0], pt[1], pt[2], best_pairs_sorted, max_rssi))
            
        return lut, heatmap_rows
