#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
# リンク制御ノード
# 基地局と車両間の一対一通信を保証する中央スケジューラ
# =============================================================================
"""
基地局が同時に通信できる車両を1台に制限する中央制御ノード。

各 comms_node から RSSI を受信し、スケジューリングポリシーに基づいて
1台の車両にのみリンク権（link_grant）を付与する。

サブスクライブトピック:
    - /{vehicle_name}/link_request (std_msgs/Float64): 各車両のRSSI
    - /{vehicle_name}/mission_complete (std_msgs/Bool): 各車両のミッション完了通知

パブリッシュトピック:
    - /{vehicle_name}/link_grant (std_msgs/Bool): リンク権付与

パラメータ:
    - vehicle_names: 車両名リスト
    - scheduling_policy: スケジューリングポリシー
        - sequential: 車両順に通信、ミッション完了で次へ切替
        - round_robin: タイムスロットで順番に切替
        - rssi_priority: RSSI最大の車両を優先
    - time_slot_duration_s: round_robin 時のスロット長 [s]
"""

import csv
import datetime
import math
import os
import re
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import Bool, Float64

from comms_sim_msgs.msg import CommsQuality

# ros2 launch経由で実行可能なように絶対インポートを優先
try:
    from comms_sim_pkg.antenna_parser import AntennaPatternParser
    from comms_sim_pkg.comms_calculator import (
        CommsCalculator,
        LogDistancePathLossModel,
        TwoRayGroundModel,
    )
except ImportError:
    from .antenna_parser import AntennaPatternParser  # type: ignore
    from .comms_calculator import (  # type: ignore
        CommsCalculator,
        LogDistancePathLossModel,
        TwoRayGroundModel,
    )

def get_run_dir(output_dir: str, summary_filename: str, run_timestamp: str, y_pos: float, antenna_yaw: float, output_subdir: str = '', entity_yaw: float = -1.5708) -> str:
    if output_subdir:
        run_idx = None
        match_run = re.search(r'run(\d+)', summary_filename)
        if match_run:
            run_idx = int(match_run.group(1))
            
        entity_yaw_deg = math.degrees(entity_yaw)
        angle_deg = (math.degrees(antenna_yaw) + entity_yaw_deg + 180.0) % 360.0
        angle_deg = round(angle_deg, 1)
        angle_str = f"{angle_deg:g}"
        y_str = f"{round(y_pos, 2):g}"
        
        run_name = f"run_{run_idx:03d}_y{y_str}_a{angle_str}" if run_idx is not None else f"run_{run_timestamp}"
        run_dir = os.path.join(
            output_dir,
            output_subdir,
            "runs",
            run_name
        )
    else:
        match = re.match(r'sweep_summary_(\d{8}_\d{6})_run(\d+)(?:_.*)?\.csv', summary_filename)
        if match:
            sweep_timestamp = match.group(1)
            run_idx = int(match.group(2))
            entity_yaw_deg = math.degrees(entity_yaw)
            angle_deg = (math.degrees(antenna_yaw) + entity_yaw_deg + 180.0) % 360.0
            angle_deg = round(angle_deg, 1)
            angle_str = f"{angle_deg:g}"
            y_str = f"{round(y_pos, 2):g}"
            run_dir = os.path.join(
                output_dir,
                f"sweep_{sweep_timestamp}",
                "runs",
                f"run_{run_idx:03d}_y{y_str}_a{angle_str}"
            )
        else:
            run_dir = os.path.join(output_dir, f"run_{run_timestamp}")
    return run_dir

def sample_trajectory(polyline_points: List[np.ndarray], resolution: float) -> List[Tuple[np.ndarray, float]]:
    samples = []
    if not polyline_points:
        return samples
    
    current_pos = polyline_points[0]
    dist_to_next = 0.0
    
    for i in range(len(polyline_points) - 1):
        pt_a = polyline_points[i]
        pt_b = polyline_points[i+1]
        
        dir_vec = pt_b - pt_a
        length = np.linalg.norm(dir_vec)
        if length < 1e-6:
            continue
        
        unit_dir = dir_vec / length
        yaw = math.atan2(dir_vec[1], dir_vec[0])
        
        t = dist_to_next
        while t <= length:
            pt = pt_a + t * unit_dir
            samples.append((pt, yaw))
            t += resolution
            
        dist_to_next = t - length
        
    return samples


def get_workspace_root() -> str:
    if os.path.exists('/workspace'):
        return '/workspace'
    current_dir = os.path.abspath(os.path.dirname(__file__))
    temp_dir = current_dir
    while True:
        if os.path.exists(os.path.join(temp_dir, '.git')) or os.path.exists(os.path.join(temp_dir, 'src')):
            return temp_dir
        parent = os.path.dirname(temp_dir)
        if parent == temp_dir:
            break
        temp_dir = parent
    return os.getcwd()


def resolve_path(raw_path: str) -> str:
    if not raw_path:
        return raw_path
    
    normalized = raw_path.replace('\\', '/')
    if os.path.isabs(raw_path) and os.path.exists(raw_path):
        return raw_path

    ws_root = get_workspace_root()
    if '/workspace/' in normalized or normalized.startswith('workspace/'):
        parts = normalized.split('/workspace/', 1)
        if len(parts) < 2:
            parts = normalized.split('workspace/', 1)
        rel_path = parts[1]
        
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory('comms_sim_pkg')
            if rel_path.startswith('config/'):
                basename = os.path.basename(rel_path)
                resolved = os.path.join(pkg_share, 'config', basename)
                if os.path.exists(resolved):
                    return resolved
            elif rel_path.startswith('models/'):
                resolved = os.path.join(pkg_share, rel_path)
                if os.path.exists(resolved):
                    return resolved
        except Exception:
            pass

        if rel_path.startswith('config/'):
            basename = os.path.basename(rel_path)
            resolved = os.path.join(ws_root, 'src', 'comms_sim_pkg', 'config', basename)
            if os.path.exists(resolved):
                return resolved
        elif rel_path.startswith('models/'):
            resolved = os.path.join(ws_root, 'src', 'comms_sim_pkg', 'models', rel_path.split('models/', 1)[1])
            if os.path.exists(resolved):
                return resolved

        resolved = os.path.join(ws_root, rel_path)
        return resolved

    return raw_path


class LinkControllerNode(Node):
    """
    基地局の一対一通信を管理するリンク制御ノード。

    基地局の指向性アンテナは同時に1台の車両としか通信できないため、
    このノードがどの車両にリンク権を与えるかを決定する。
    """

    def __init__(self) -> None:
        super().__init__('link_controller_node')

        # =====================================================================
        # パラメータ宣言・取得
        # =====================================================================
        self.declare_parameter('vehicle_names', [''])
        self.declare_parameter('scheduling_policy', 'sequential')
        self.declare_parameter('time_slot_duration_s', 10.0)
        self.declare_parameter('beam_gain_threshold', 5.0)
        self.declare_parameter('weight_distance', 0.7)
        self.declare_parameter('weight_angle', 0.3)
        self.declare_parameter('scheduling_rate_hz', 1000.0)
        self.declare_parameter('proactive_handover_score_threshold', -80.0)
        self.declare_parameter('min_hold_time_s', 1.0)
        self.declare_parameter('switch_margin_db', 2.0)
        self.declare_parameter('proactive_grace_period_s', 0.5)
        self.declare_parameter('logging_level', 1)
        self.declare_parameter('heatmap_resolution_m', 0.2)
        self.declare_parameter('ff_max_pairs', -1)
        self.declare_parameter('run_timestamp', '')
        self.declare_parameter('config_file_path', resolve_path('/workspace/config/sim_params.yaml'))
        self.declare_parameter('output_subdir', '')
        self.declare_parameter('filter_main_lobe', True)
        self.declare_parameter('mainlobe_angle_margin_deg', 5.0)
        self.declare_parameter('mainlobe_e_half_angle_deg', -1.0)
        self.declare_parameter('mainlobe_h_half_angle_deg', -1.0)


        vehicle_names_raw = self.get_parameter('vehicle_names').value
        self.scheduling_policy: str = str(
            self.get_parameter('scheduling_policy').value
        ).lower()
        self.time_slot_duration: float = float(
            self.get_parameter('time_slot_duration_s').value
        )
        self.beam_gain_threshold: float = float(
            self.get_parameter('beam_gain_threshold').value
        )
        self.weight_distance: float = float(
            self.get_parameter('weight_distance').value
        )
        self.weight_angle: float = float(
            self.get_parameter('weight_angle').value
        )
        self.scheduling_rate_hz: float = float(
            self.get_parameter('scheduling_rate_hz').value
        )
        self.proactive_handover_score_threshold: float = float(
            self.get_parameter('proactive_handover_score_threshold').value
        )
        self.min_hold_time_s: float = float(
            self.get_parameter('min_hold_time_s').value
        )
        self.switch_margin_db: float = float(
            self.get_parameter('switch_margin_db').value
        )
        self.proactive_grace_period_s: float = float(
            self.get_parameter('proactive_grace_period_s').value
        )
        self.logging_level: int = int(
            self.get_parameter('logging_level').value
        )
        self.heatmap_resolution_m: float = float(
            self.get_parameter('heatmap_resolution_m').value
        )
        self.ff_max_pairs: int = int(
            self.get_parameter('ff_max_pairs').value
        )
        self.run_timestamp: str = str(
            self.get_parameter('run_timestamp').value
        )
        self.config_file_path: str = str(
            self.get_parameter('config_file_path').value
        )
        self.output_subdir: str = str(
            self.get_parameter('output_subdir').value
        )
        self.filter_main_lobe: bool = bool(
            self.get_parameter('filter_main_lobe').value
        )
        self.mainlobe_angle_margin_deg: float = float(
            self.get_parameter('mainlobe_angle_margin_deg').value
        )
        self.mainlobe_e_half_angle_deg: float = float(
            self.get_parameter('mainlobe_e_half_angle_deg').value
        )
        self.mainlobe_h_half_angle_deg: float = float(
            self.get_parameter('mainlobe_h_half_angle_deg').value
        )


        # 車両名リストのパース
        if isinstance(vehicle_names_raw, list):
            self.vehicle_names: List[str] = [
                str(v) for v in vehicle_names_raw if v
            ]
        else:
            self.vehicle_names = []

        if not self.vehicle_names:
            self.get_logger().error('vehicle_names が空です。リンク制御を開始できません。')
            return

        # ポリシー検証
        valid_policies = (
            'sequential', 'round_robin', 'rssi_priority',
            'geometric_beam_priority', 'physical_score_priority', 'geometric_weighted',
            'feedforward_optimal', 'simple_no_handover'
        )
        if self.scheduling_policy not in valid_policies:
            self.get_logger().warn(
                f'不明なポリシー: "{self.scheduling_policy}"。'
                f'"sequential" を使用します。'
            )
            self.scheduling_policy = 'sequential'

        # =====================================================================
        # 状態変数
        # =====================================================================
        # 現在リンク権を持つ車両のインデックス
        self._active_idx: int = 0
        self._last_grant_change_time: float = 0.0

        # 各車両の最新RSSI
        self._rssi: Dict[str, float] = {
            name: float('-inf') for name in self.vehicle_names
        }

        # 各車両のジオメトリ情報 (距離, アンテナゲイン, 有効フラグ等)
        self._geometry_info: Dict[str, dict] = {
            name: {} for name in self.vehicle_names
        }

        # 各車両のミッション完了フラグ
        self._mission_complete: Dict[str, bool] = {
            name: False for name in self.vehicle_names
        }

        # round_robin 用: 最後にスロットを切り替えた時刻
        self._last_slot_switch_time: Optional[float] = None

        # 決定論的タイムスタンプ同期用バッファ
        self._quality_buffer: Dict[tuple, Dict[str, object]] = {}

        # =====================================================================
        # サブスクライバ・パブリッシャの動的生成
        # =====================================================================
        self._grant_pubs: Dict[str, object] = {}
        self._request_subs: List[object] = []
        self._quality_subs: List[object] = []
        self._mission_subs: List[object] = []

        for name in self.vehicle_names:
            # リンクグラント パブリッシャ
            pub = self.create_publisher(Bool, f'/{name}/link_grant', 10)
            self._grant_pubs[name] = pub

            # リンクリクエスト サブスクライバ（RSSI報告）
            sub = self.create_subscription(
                Float64,
                f'/{name}/link_request',
                lambda msg, vn=name: self._on_link_request(vn, msg),
                10
            )
            self._request_subs.append(sub)

            # 幾何学・品質 サブスクライバ（距離・ゲイン情報を取得）
            sub_q = self.create_subscription(
                CommsQuality,
                f'/{name}/comms/quality',
                lambda msg, vn=name: self._on_comms_quality(vn, msg),
                10
            )
            self._quality_subs.append(sub_q)

            # ミッション完了 サブスクライバ
            sub_mc = self.create_subscription(
                Bool,
                f'/{name}/mission_complete',
                lambda msg, vn=name: self._on_mission_complete(vn, msg),
                10
            )
            self._mission_subs.append(sub_mc)

        # Load run parameters and precalculate nominal RSSI LUT if feedforward_optimal or level >= 5
        self.y_pos = 0.0
        self.angle = 0.0
        self.entity_yaw = -1.5708
        self.summary_filename = 'sweep_summary.csv'
        self.center_antenna_name = 'shinkansen_mid'
        try:
            with open(self.config_file_path, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
                spawn_ent = config.get('spawn_entities', {})
                antenna_keys = [k for k in spawn_ent.keys() if 'antenna' in k.lower()]
                antenna_cfg = spawn_ent.get(antenna_keys[0]) if antenna_keys else {}
                self.y_pos = float(antenna_cfg.get('pose', [0,0,0,0,0,0])[1])
                self.angle = float(antenna_cfg.get('antenna_relative_rpy', [0,0,0])[2])
                self.entity_yaw = float(antenna_cfg.get('pose', [0,0,0,0,0,0])[5])
                self.summary_filename = config.get('simulation', {}).get('summary_filename', 'sweep_summary.csv')
                
                # 車両アンテナから中心アンテナを動的に特定
                vehicles_cfg = config.get('vehicles', [])
                if vehicles_cfg:
                    vehicle = vehicles_cfg[0]
                    vehicle_antennas = vehicle.get('antennas', [])
                    min_x_offset = float('inf')
                    for va in vehicle_antennas:
                        offset = va.get('offset', [0.0, 0.0, 0.0])
                        x_off = abs(float(offset[0]))
                        if x_off < min_x_offset:
                            min_x_offset = x_off
                            self.center_antenna_name = va['name']
                    self.get_logger().info(f"車両中心の基準アンテナとして {self.center_antenna_name} を検出しました (x_offset: {min_x_offset}m)")
        except Exception as e:
            self.get_logger().warn(f"Failed to read yaml for summary / center antenna: {e}")

        # Set run directory
        self.run_dir = get_run_dir(resolve_path('/workspace/sim_results/'), self.summary_filename, self.run_timestamp, self.y_pos, self.angle, self.output_subdir, self.entity_yaw)

        self.lut = []
        self.ff_file = None
        self.ff_writer = None
        self._start_time = None

        if self.scheduling_policy == 'feedforward_optimal' or self.logging_level >= 5:
            self.get_logger().info('フィードフォワード用 nominal RSSI ヒートマップの事前計算を開始します。')
            try:
                with open(self.config_file_path, 'r', encoding='utf-8') as f:
                    config = yaml.safe_load(f)
                
                spawn_ent = config.get('spawn_entities', {})
                rx_nodes = []
                for k, v in spawn_ent.items():
                    if k.startswith('antenna_'):
                        pos = np.array(v.get('pose', [0, 0, 0, 0, 0, 0])[:3], dtype=float)
                        offset = np.array(v.get('antenna_offset', [0, 0, 0]), dtype=float)
                        rpy = np.array(v.get('pose', [0, 0, 0, 0, 0, 0])[3:6], dtype=float)
                        rel_rpy = np.array(v.get('antenna_relative_rpy', [0, 0, 0]), dtype=float)
                        rx_nodes.append({
                            'name': k,
                            'position': pos,
                            'antenna_offset': offset,
                            'rpy': rpy,
                            'antenna_relative_rpy': rel_rpy
                        })
                
                vehicles_cfg = config.get('vehicles', [])
                if not vehicles_cfg:
                    raise ValueError("No vehicles found in config.")
                vehicle = vehicles_cfg[0]
                vehicle_spawn_pose = vehicle.get('pose', [0, 0, 0, 0, 0, 0])
                vehicle_waypoints = vehicle.get('waypoints', [])
                vehicle_antennas = vehicle.get('antennas', [])
                
                polyline_points = [np.array(vehicle_spawn_pose[:3], dtype=float)]
                for wp in vehicle_waypoints:
                    polyline_points.append(np.array(wp[:3], dtype=float))
                
                samples = sample_trajectory(polyline_points, self.heatmap_resolution_m)
                if rx_nodes:
                    filtered_samples = []
                    for pt, yaw in samples:
                        near_bs = False
                        for bs in rx_nodes:
                            if np.linalg.norm(pt[:2] - bs['position'][:2]) < 100.0:
                                near_bs = True
                                break
                        if near_bs:
                            filtered_samples.append((pt, yaw))
                    samples = filtered_samples
                self.get_logger().info(f'軌道をサンプリングしました: {len(samples)} 点 (解像度: {self.heatmap_resolution_m} m, 基地局近傍フィルタ適用後)')
                
                comms_params = config.get('comms_simulator_node', {}).get('ros__parameters', {})
                e_plane_path = resolve_path(comms_params.get('e_plane_path', '/workspace/config/e_plane.csv'))
                h_plane_path = resolve_path(comms_params.get('h_plane_path', '/workspace/config/h_plane.csv'))
                max_att = float(comms_params.get('max_antenna_attenuation', 30.0))
                
                parser = AntennaPatternParser(
                    e_plane_path=e_plane_path,
                    h_plane_path=h_plane_path,
                    max_antenna_attenuation=max_att,
                    mainlobe_angle_margin_deg=self.mainlobe_angle_margin_deg,
                    mainlobe_e_half_angle_override_deg=self.mainlobe_e_half_angle_deg,
                    mainlobe_h_half_angle_override_deg=self.mainlobe_h_half_angle_deg
                )

                
                pl_params = comms_params.get('path_loss', {})
                c = float(pl_params.get('c', 299792458.0))
                frequency = float(pl_params.get('frequency', 6.0e10))
                exponent = float(pl_params.get('exponent', 2.0))
                d0 = float(pl_params.get('d0', 1.0))
                pl_d0 = float(pl_params.get('pl_d0', -1.0))
                
                propagation_model = LogDistancePathLossModel(
                    c=c, frequency=frequency, exponent=exponent, d0=d0, pl_d0=pl_d0
                )
                
                tx_power = float(comms_params.get('tx_power', -7.0))
                noise_variance = float(comms_params.get('noise_variance', 0.0))
                mcs_table_path = resolve_path(comms_params.get('mcs_table_path', '/workspace/config/MCStable.csv'))
                
                calculator = CommsCalculator(
                    propagation_model=propagation_model,
                    tx_power_dbm=tx_power,
                    noise_variance=noise_variance,
                    mcs_table_path=mcs_table_path
                )
                from itertools import permutations
                
                num_tx = len(vehicle_antennas)
                num_rx = len(rx_nodes)
                num_pairs = min(num_tx, num_rx)
                if self.ff_max_pairs > 0:
                    num_pairs = min(num_pairs, self.ff_max_pairs)
                
                heatmap_rows = []
                for pt, yaw in samples:
                    tx_orientation = np.array([0.0, 0.0, yaw])
                    
                    row = {
                        'x_m': round(pt[0], 4),
                        'y_m': round(pt[1], 4),
                        'z_m': round(pt[2], 4),
                        'yaw_rad': round(yaw, 4)
                    }
                    
                    # Step 1: 全 (TX, RX) 組み合わせのRSSIを計算
                    # rssi_matrix[tx_idx][rx_idx] = RSSI
                    R_veh = parser._rpy_to_rotmat(tx_orientation[0], tx_orientation[1], tx_orientation[2])
                    rssi_matrix = [[-999.0] * num_rx for _ in range(num_tx)]
                    
                    for tx_idx, va in enumerate(vehicle_antennas):
                        tx_antenna_pos = pt + R_veh.dot(np.asarray(va['offset'], dtype=float))
                        tx_ant_rpy = tx_orientation + np.asarray(va['relative_rpy'], dtype=float)
                        
                        for rx_idx, bs in enumerate(rx_nodes):
                            bs_antenna_pos = bs['position'] + bs['antenna_offset']
                            bs_ant_rpy = bs['rpy'] + bs['antenna_relative_rpy']
                            
                            tx_e, tx_h, tx_total, rx_e, rx_h, rx_total = parser.get_tx_rx_gains(
                                tx_pos_world=bs_antenna_pos,
                                tx_rpy_world=bs_ant_rpy,
                                rx_pos_world=tx_antenna_pos,
                                rx_rpy_world=tx_ant_rpy,
                            )
                            antenna_gain_db = float(tx_total + rx_total)
                            
                            in_main = True
                            if self.filter_main_lobe:
                                tx_el, tx_az = parser.calculate_antenna_frame_angles(bs_antenna_pos, tx_antenna_pos, bs_ant_rpy)
                                rx_el, rx_az = parser.calculate_antenna_frame_angles(tx_antenna_pos, bs_antenna_pos, tx_ant_rpy)
                                tx_in = parser.is_in_main_lobe(np.degrees(abs(tx_el)), np.degrees(abs(tx_az)))
                                rx_in = parser.is_in_main_lobe(np.degrees(abs(rx_el)), np.degrees(abs(rx_az)))
                                if not (tx_in and rx_in):
                                    in_main = False

                            if in_main:
                                metrics = calculator.calculate_all(
                                    tx_antenna_pos,
                                    bs_antenna_pos,
                                    antenna_gain_db=antenna_gain_db,
                                    add_noise=False
                                )
                                rssi_matrix[tx_idx][rx_idx] = metrics['rssi']
                            
                            col_name = f"rssi_{bs['name']}_{va['name']}"
                            row[col_name] = round(rssi_matrix[tx_idx][rx_idx], 2)
                    
                    # Step 2: 全列挙で最適N個ペアを決定
                    best_total_rssi = -1e9
                    best_pairs = []
                    
                    for perm in permutations(range(num_rx)):
                        total_rssi = sum(rssi_matrix[tx_idx][perm[tx_idx]] for tx_idx in range(num_pairs))
                        if total_rssi > best_total_rssi:
                            best_total_rssi = total_rssi
                            best_pairs = [
                                (vehicle_antennas[tx_idx]['name'], rx_nodes[perm[tx_idx]]['name'], rssi_matrix[tx_idx][perm[tx_idx]])
                                for tx_idx in range(num_pairs)
                            ]
                    
                    # RSSIの高い順にソート
                    best_pairs_sorted = sorted(best_pairs, key=lambda p: p[2], reverse=True)
                    max_rssi = best_pairs_sorted[0][2] if best_pairs_sorted else -999.0
                    
                    row['optimal_pairs'] = ';'.join(f"{p[0]}:{p[1]}" for p in best_pairs_sorted)
                    row['max_rssi'] = round(max_rssi, 2)
                    heatmap_rows.append(row)
                    
                    self.lut.append((pt[0], pt[1], pt[2], best_pairs_sorted, max_rssi))
                
                if self.logging_level >= 5:
                    heatmap_dir = os.path.join(self.run_dir, 'heatmap')
                    os.makedirs(heatmap_dir, exist_ok=True)
                    heatmap_path = os.path.join(heatmap_dir, 'rssi_heatmap.csv')
                    
                    with open(heatmap_path, 'w', newline='', encoding='utf-8') as csvfile:
                        fieldnames = ['x_m', 'y_m', 'z_m', 'yaw_rad']
                        for bs in rx_nodes:
                            for va in vehicle_antennas:
                                fieldnames.append(f"rssi_{bs['name']}_{va['name']}")
                        fieldnames.extend(['optimal_antenna', 'max_rssi'])
                        
                        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                        writer.writeheader()
                        for r in heatmap_rows:
                            writer.writerow(r)
                            
                    self._set_file_ownership(heatmap_path)
                    self.get_logger().info(f'nominal RSSI ヒートマップを保存しました: {heatmap_path}')
                    
            except Exception as e:
                self.get_logger().error(f'事前計算中にエラーが発生しました: {e}')
                import traceback
                self.get_logger().error(traceback.format_exc())

        if self.logging_level >= 5:
            try:
                control_dir = os.path.join(self.run_dir, 'control')
                os.makedirs(control_dir, exist_ok=True)
                ff_path = os.path.join(control_dir, 'feedforward_log.csv')
                
                fieldnames = ['time_s', 'tx_x_m', 'tx_y_m', 'tx_z_m']
                for name in self.vehicle_names:
                    fieldnames.append(f"rssi_{name}")
                fieldnames.extend(['selected_antenna', 'rssi_optimal_dBm'])
                
                self.ff_file = open(ff_path, 'w', newline='', encoding='utf-8')
                self.ff_writer = csv.DictWriter(self.ff_file, fieldnames=fieldnames)
                self.ff_writer.writeheader()
                self._set_file_ownership(ff_path)
            except Exception as e:
                self.get_logger().error(f'フィードフォワードログファイルオープンに失敗しました: {e}')

        # ストラテジーの初期化
        try:
            from .link_scheduling_strategy import (
                SequentialStrategy, RoundRobinStrategy, RssiPriorityStrategy,
                GeometricBeamPriorityStrategy, GeometricWeightedStrategy,
                PhysicalScorePriorityStrategy, FeedforwardOptimalStrategy
            )
        except ImportError:
            from link_scheduling_strategy import (
                SequentialStrategy, RoundRobinStrategy, RssiPriorityStrategy,
                GeometricBeamPriorityStrategy, GeometricWeightedStrategy,
                PhysicalScorePriorityStrategy, FeedforwardOptimalStrategy
            )

        if self.scheduling_policy == 'sequential':
            self.strategy = SequentialStrategy()
        elif self.scheduling_policy == 'round_robin':
            self.strategy = RoundRobinStrategy(self.time_slot_duration)
        elif self.scheduling_policy == 'rssi_priority':
            self.strategy = RssiPriorityStrategy()
        elif self.scheduling_policy == 'geometric_beam_priority':
            self.strategy = GeometricBeamPriorityStrategy(self.beam_gain_threshold, self.filter_main_lobe)
        elif self.scheduling_policy == 'physical_score_priority':
            self.strategy = PhysicalScorePriorityStrategy(
                self.beam_gain_threshold,
                self.min_hold_time_s,
                self.switch_margin_db,
                self.proactive_handover_score_threshold,
                self.filter_main_lobe
            )
        elif self.scheduling_policy == 'simple_no_handover':
            self.strategy = SequentialStrategy()

        elif self.scheduling_policy == 'geometric_weighted':
            self.strategy = GeometricWeightedStrategy(self.weight_distance, self.weight_angle)
        elif self.scheduling_policy == 'feedforward_optimal':
            self.strategy = FeedforwardOptimalStrategy(self.lut, self.center_antenna_name)
        else:
            self.strategy = SequentialStrategy()

        # =====================================================================
        # スケジューリングタイマー (決定論的動作のため、_on_comms_quality内で同期呼び出しされます)
        # =====================================================================
        # self.timer_cb_group = MutuallyExclusiveCallbackGroup()
        # timer_period = 1.0 / self.scheduling_rate_hz if self.scheduling_rate_hz > 0 else 0.001
        # self._schedule_timer = self.create_timer(
        #     timer_period, 
        #     self._schedule_tick,
        #     callback_group=self.timer_cb_group
        # )

        self.get_logger().info(
            f'LinkControllerNode 初期化完了\n'
            f'  車両数: {len(self.vehicle_names)}\n'
            f'  車両名: {self.vehicle_names}\n'
            f'  ポリシー: {self.scheduling_policy} (Strategy initialized)\n'
            f'  タイムスロット: {self.time_slot_duration} s'
        )

    # =========================================================================
    # コールバック
    # =========================================================================

    def _on_link_request(self, vehicle_name: str, msg: Float64) -> None:
        """各車両からのRSSI報告を受信。"""
        self._rssi[vehicle_name] = msg.data

    def _on_comms_quality(self, vehicle_name: str, msg: CommsQuality) -> None:
        """各車両からの通信品質・幾何学情報を受信。"""
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # 幾何情報を更新
        self._geometry_info[vehicle_name] = {
            'distance': msg.distance,
            'antenna_gain_e_plane': msg.antenna_gain_e_plane,
            'antenna_gain_h_plane': msg.antenna_gain_h_plane,
            'path_loss': msg.path_loss,
            'comm_active': msg.comm_active,
            'link_state': msg.link_state,
            'rssi': msg.rssi,
            'in_main_lobe': msg.in_main_lobe,
            'off_boresight_e_deg': msg.off_boresight_e_deg,
            'off_boresight_h_deg': msg.off_boresight_h_deg,
            'tx_x': msg.tx_x,
            'tx_y': msg.tx_y,
            'tx_z': msg.tx_z
        }

        
        # すべてのアンテナデータが一度揃ったら、基準アンテナ受信時にのみスケジューリングを実行して無駄な多重実行を防止する。
        # 揃う前は、全てのアンテナデータが集まった瞬間に一度実行する。
        if len(self._geometry_info) == len(self.vehicle_names):
            if vehicle_name == self.center_antenna_name or self._last_grant_change_time == 0.0:
                self._schedule_tick(current_time)

    def _on_mission_complete(self, vehicle_name: str, msg: Bool) -> None:
        """各車両のミッション完了通知を受信。"""
        if not msg.data:
            return
        if not self._mission_complete.get(vehicle_name, False):
            self._mission_complete[vehicle_name] = True
            self.get_logger().info(
                f'ミッション完了通知受信: {vehicle_name}'
            )

    # =========================================================================
    # スケジューリング
    # =========================================================================

    def _schedule_tick(self, current_time: Optional[float] = None) -> None:
        """
        定期的にリンク権を決定し、各車両にブロードキャストする。
        
        スケジューリングは以下の2フェーズで行われます:
        【フェーズ1: ポリシーベースのスケジュール決定】
          設定された scheduling_policy (例: physical_score_priority, sequential 等)
          に従って、Strategyクラスが「次にリンク権を持つべき車両」を決定します。
        
        【フェーズ2: プロアクティブ・ハンドオーバー（通信断絶の回避）】
          フェーズ1で選ばれた車両が物理的な要因（距離やアンテナ角度など）により
          実際には通信を確立できていない（DISCONNECTED）場合、通信可能な別の車両を
          探して強制的にリンク権を切り替える安全装置です。
        """
        if current_time is None:
            current_time = self.get_clock().now().nanoseconds / 1e9

        # --- フェーズ1: 基本的なスケジューリング実行 ---
        new_idx, log_msg = self.strategy.determine_active_link(
            self._rssi,
            self._mission_complete,
            self._geometry_info,
            self._active_idx,
            self.vehicle_names,
            current_time
        )

        if log_msg:
            self.get_logger().info(log_msg)
            self._active_idx = new_idx
            self._last_grant_change_time = current_time

        # --- フェーズ2: プロアクティブ・ハンドオーバー実行 ---
        # 現在のgrant保持者のlink_stateを確認
        # DISCONNECTED状態の場合、物理スコア（受信電力の推定値）が最良の車両に即座にgrantを移す
        active_name = self.vehicle_names[self._active_idx]
        active_info = self._geometry_info.get(active_name, {})
        active_link_state = active_info.get('link_state', 'DISCONNECTED')

        # リンク権付与直後は状態が伝播・確立するまでの猶予期間（0.1秒）を設ける
        grace_period_passed = (current_time - self._last_grant_change_time) > self.proactive_grace_period_s
        hold_time_passed = (current_time - self._last_grant_change_time) >= self.min_hold_time_s

        if self.scheduling_policy != 'feedforward_optimal' and active_link_state == 'DISCONNECTED' and active_info and grace_period_passed and hold_time_passed:
            best_score = float('-inf')
            best_idx = None
            for i, name in enumerate(self.vehicle_names):
                if i == self._active_idx:
                    continue
                info = self._geometry_info.get(name, {})
                if not info.get('comm_active', False):
                    continue
                if self.filter_main_lobe and not info.get('in_main_lobe', True):
                    continue

                # 物理スコア = 送信側総ゲイン(E) + 受信側総ゲイン(H) - パスロス
                # これは実質的にノイズを含まない理想的なRSSI(受信電力)に比例します
                e_gain = info.get('antenna_gain_e_plane', -999.0)
                h_gain = info.get('antenna_gain_h_plane', -999.0)
                path_loss = info.get('path_loss', 999.0)
                score = e_gain + h_gain - path_loss
                if score > best_score:
                    best_score = score
                    best_idx = i
            if best_idx is not None:
                active_score = (
                    active_info.get('antenna_gain_e_plane', -999.0) +
                    active_info.get('antenna_gain_h_plane', -999.0) -
                    active_info.get('path_loss', 999.0)
                )
                # 最良スコアが閾値を下回る場合は強制切替しない
                # （geometry_info 未到達・初期値など信頼できないデータによる誤切替を防ぐ）
                if best_score < self.proactive_handover_score_threshold:
                    self.get_logger().debug(
                        f'リンク切り替え（接続制御）スキップ: best_score={best_score:.1f} < '
                        f'threshold={self.proactive_handover_score_threshold:.1f}'
                    )
                elif best_score > active_score + self.switch_margin_db:
                    old_name = active_name
                    self._active_idx = best_idx
                    active_name = self.vehicle_names[self._active_idx]
                    self.get_logger().info(
                        f'リンク切り替え（接続制御）: {old_name}(DISCONNECTED) → {active_name} '
                        f'(geo_score={best_score:.1f})'
                    )
                    self._last_grant_change_time = current_time

        # リンクグラントをパブリッシュ
        for name, pub in self._grant_pubs.items():
            msg = Bool()
            # feedforward_optimal: マルチペアの場合、active_pairsに含まれるアンテナ全てにgrant
            if hasattr(self.strategy, 'active_pairs') and self.strategy.active_pairs:
                has_grant = (name in self.strategy.active_pairs)
            else:
                has_grant = (name == active_name)
            msg.data = has_grant
            pub.publish(msg)

        # Feedforward log
        if self.logging_level >= 5 and self.ff_writer is not None:
            if self._start_time is None:
                self._start_time = current_time
            elapsed = current_time - self._start_time
            
            ux, uy, uz = 0.0, 0.0, 0.0
            for name in self.vehicle_names:
                info = self._geometry_info.get(name, {})
                if 'tx_x' in info:
                    ux = info['tx_x']
                    uy = info['tx_y']
                    uz = info['tx_z']
                    break
            
            nominal_rssi = float('-inf')
            if hasattr(self.strategy, 'lut') and self.strategy.lut and hasattr(self.strategy, '_last_idx'):
                last_idx = getattr(self.strategy, '_last_idx', 0)
                if 0 <= last_idx < len(self.strategy.lut):
                    nominal_rssi = self.strategy.lut[last_idx][4]
            
            row = {
                'time_s': round(elapsed, 4),
                'tx_x_m': round(ux, 4),
                'tx_y_m': round(uy, 4),
                'tx_z_m': round(uz, 4)
            }
            for name in self.vehicle_names:
                row[f"rssi_{name}"] = round(self._rssi.get(name, float('-inf')), 2)
            
            # マルチペア情報をログに含める
            active_pairs = getattr(self.strategy, 'active_pairs', {})
            if active_pairs:
                row['active_pairs'] = ';'.join(f"{tx}:{rx}" for tx, rx in active_pairs.items())
                row['num_active_pairs'] = len(active_pairs)
            else:
                row['active_pairs'] = self.vehicle_names[self._active_idx]
                row['num_active_pairs'] = 1
            row['selected_antenna'] = self.vehicle_names[self._active_idx]
            row['rssi_optimal_dBm'] = round(nominal_rssi, 2)
            
            try:
                self.ff_writer.writerow(row)
                self.ff_file.flush()
            except Exception as e:
                self.get_logger().error(f"Failed to write feedforward log: {e}")

    def _set_file_ownership(self, filepath):
        try:
            ws_stat = os.stat(get_workspace_root())
            os.chown(filepath, ws_stat.st_uid, ws_stat.st_gid)
        except Exception:
            pass

    def destroy_node(self):
        if hasattr(self, 'ff_file') and self.ff_file and not self.ff_file.closed:
            self.ff_file.close()
            self.ff_file = None
        super().destroy_node()


def main(args=None):
    """メインエントリポイント。"""
    rclpy.init(args=args)

    node = LinkControllerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
