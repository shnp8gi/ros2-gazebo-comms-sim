#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
通信シミュレーション結果を4段階のレベルに分けてロギングするノード。
"""

import atexit
import csv
import os
import re
import math
from datetime import datetime
from typing import Dict, List
import sys
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

from comms_sim_msgs.msg import CommsQuality

# mission_complete は VOLATILE で受信する
# 以前の実行での古いキャッシュメッセージを誤って受信するのを防ぐため、ラッチしない
_MISSION_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

# 車両停止を検知してから強制シャットダウンするまでの猶予時間 [壁時計秒]
# RTF=10（固定）なら 30s壁時計 = 300s シミュレーション時間
_WATCHDOG_STOP_TIMEOUT_SEC = 30.0

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


class SimLoggerNode(Node):
    def __init__(self):
        super().__init__('sim_logger_node')
        
        self.declare_parameter('vehicle_names', [''])
        self.declare_parameter('base_vehicle_names', [''])
        ws_root = get_workspace_root()
        self.declare_parameter('output_dir', os.path.join(ws_root, 'sim_results', ''))
        self.declare_parameter('output_subdir', '')
        self.declare_parameter('logging_level', 1)
        self.declare_parameter('run_timestamp', '')
        self.declare_parameter('config_file_path', resolve_path('/workspace/config/sim_params.yaml'))
        
        vehicle_names_raw = self.get_parameter('vehicle_names').value
        if isinstance(vehicle_names_raw, list):
            self.vehicle_names = [str(v) for v in vehicle_names_raw if v]
        else:
            self.vehicle_names = []
            
        base_vehicle_names_raw = self.get_parameter('base_vehicle_names').value
        if isinstance(base_vehicle_names_raw, list):
            self.base_vehicle_names = [str(v) for v in base_vehicle_names_raw if v]
        else:
            self.base_vehicle_names = self.vehicle_names.copy()
            
        self.output_dir = self.get_parameter('output_dir').value
        self.output_subdir = str(self.get_parameter('output_subdir').value)
        self.logging_level = self.get_parameter('logging_level').value
        self.run_timestamp = str(self.get_parameter('run_timestamp').value)
        self.config_file_path = str(self.get_parameter('config_file_path').value)
        
        if self.run_timestamp:
            self.timestamp = self.run_timestamp
        else:
            self.timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # YAMLからシミュレーション条件を読み込む（サマリー用）
        self.y_pos = 0.0
        self.angle = 0.0
        self.summary_filename = 'sweep_summary.csv'
        try:
            with open(self.config_file_path, 'r') as f:
                config = yaml.safe_load(f)
                sim_cfg = config.get('simulation', {})
                self.summary_filename = sim_cfg.get('summary_filename', 'sweep_summary.csv')
                if not self.output_subdir:
                    self.output_subdir = sim_cfg.get('output_subdir', '')
                
                # simulationブロックから y_position と angle_deg を直接取得
                if 'y_position' in sim_cfg and 'angle_deg' in sim_cfg:
                    self.y_pos = float(sim_cfg['y_position'])
                    self.angle = float(sim_cfg['angle_deg'])
                else:
                    # 従来のフォールバック
                    spawn_ent = config.get('spawn_entities', {})
                    antenna_keys = [k for k in spawn_ent.keys() if 'antenna' in k.lower()]
                    antenna_cfg = spawn_ent.get(antenna_keys[0]) if antenna_keys else {}
                    self.y_pos = float(antenna_cfg.get('pose', [0,0,0,0,0,0])[1])
                    raw_yaw = float(antenna_cfg.get('antenna_relative_rpy', [0,0,0])[2])
                    entity_yaw = float(antenna_cfg.get('pose', [0,0,0,0,0,0])[5])
                    entity_yaw_deg = math.degrees(entity_yaw)
                    self.angle = round((math.degrees(raw_yaw) + entity_yaw_deg + 180.0) % 360.0, 1)
        except Exception as e:
            self.get_logger().warn(f"Failed to read yaml for summary ({self.config_file_path}): {e}")

        # ディレクトリパスの解決

        if self.output_subdir:
            # output_subdir が指定されている場合、正規表現でのパース不要。直接ディレクトリ構成を決定
            run_idx = None
            match_run = re.search(r'run(\d+)', self.summary_filename)
            if match_run:
                run_idx = int(match_run.group(1))

            angle_deg = self.angle
            angle_str = f"{angle_deg:g}"
            y_str = f"{round(self.y_pos, 2):g}"
            
            run_name = f"run_{run_idx:03d}_y{y_str}_a{angle_str}" if run_idx is not None else f"run_{self.timestamp}"
            self.run_dir = os.path.join(
                self.output_dir,
                self.output_subdir,
                "runs",
                run_name
            )
        else:
            # 従来通りのフォールバック
            match = re.match(r'sweep_summary_(\d{8}_\d{6})_run(\d+)(?:_.*)?\.csv', self.summary_filename)
            if match:
                sweep_timestamp = match.group(1)
                run_idx = int(match.group(2))
                angle_deg = self.angle
                angle_str = f"{angle_deg:g}"
                y_str = f"{round(self.y_pos, 2):g}"
                self.run_dir = os.path.join(
                    self.output_dir,
                    f"sweep_{sweep_timestamp}",
                    "runs",
                    f"run_{run_idx:03d}_y{y_str}_a{angle_str}"
                )
            else:
                self.run_dir = os.path.join(self.output_dir, f"run_{self.timestamp}")

        # コムズ、コントロールのサブディレクトリ作成
        comms_dir = os.path.join(self.run_dir, 'comms')
        control_dir = os.path.join(self.run_dir, 'control')
        
        os.makedirs(comms_dir, exist_ok=True)
        os.makedirs(control_dir, exist_ok=True)
        self._set_file_ownership(self.run_dir)
        self._set_file_ownership(comms_dir)
        self._set_file_ownership(control_dir)

        # Level 1: サマリー用集計データ
        self.summary_stats = {
            vn: {
                'total_data': 0.0,
                'tp_sum': 0.0,
                'rssi_sum': 0.0,
                'connected_count': 0,
                'connected_time': 0.0,
                'handover_count': 0
            } for vn in self.vehicle_names
        }
        
        self._latest_sim_time = 0.0
        
        # Level 2: イベントログ用
        self.last_state = {vn: {'link_state': 'DISCONNECTED', 'has_link_grant': False} for vn in self.vehicle_names}
        self.event_file = None
        self.event_writer = None
        if self.logging_level >= 2:
            event_path = os.path.join(control_dir, 'events_ros.csv')
            self.event_file = open(event_path, 'w', newline='', encoding='utf-8')
            self.event_writer = csv.DictWriter(self.event_file, fieldnames=[
                'time_s', 'vehicle_name', 'link_state', 'has_link_grant', 'rssi_dBm', 'distance_m'
            ])
            self.event_writer.writeheader()
            self._set_file_ownership(event_path)
            
        # Level 3/4: 時系列ログ用 (C++側が直接出力するため、Python側では無効化)
        self.ts_files = {}
        self.ts_writers = {}
        # if self.logging_level >= 3:
        #     suffix = 'connected' if self.logging_level == 3 else 'full'
        #     for vn in self.vehicle_names:
        #         path = os.path.join(comms_dir, f'{vn}_{suffix}.csv')
        #         f = open(path, 'w', newline='', encoding='utf-8')
        #         self.ts_files[vn] = f
        #         
        #         if self.logging_level == 3:
        #             fields = ['time_s', 'vehicle_name', 'distance_m', 'rssi_dBm', 'throughput_Gbps', 
        #                       'total_data_MB', 'path_loss_dB', 'e_gain_dB', 'h_gain_dB',
        #                       'tx_x_m', 'tx_y_m', 'tx_z_m', 'bs_x_m', 'bs_y_m', 'bs_z_m']
        #         else: # Level 4
        #             fields = ['time_s', 'vehicle_time_s', 'vehicle_name', 'has_link_grant', 'distance_m', 
        #                       'rssi_dBm', 'throughput_Gbps', 'total_data_MB', 'path_loss_dB', 'e_gain_dB', 
        #                       'h_gain_dB', 'comm_active', 'tx_x_m', 'tx_y_m', 'tx_z_m', 
        #                       'bs_x_m', 'bs_y_m', 'bs_z_m', 'link_state']
        #         
        #         self.ts_writers[vn] = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
        #         self.ts_writers[vn].writeheader()
        #         self._set_file_ownership(path)

        # 状態保持
        self.ts_buffers = {vn: [] for vn in self.vehicle_names}
        self._link_grants: Dict[str, bool] = {name: False for name in self.vehicle_names}
        self._mission_status: Dict[str, bool] = {name: False for name in self.base_vehicle_names}
        self._last_time = {name: None for name in self.vehicle_names}
        self._last_vehicle_pos: Dict[str, tuple] = {}
        
        self._quality_subs = []
        self._grant_subs = []
        self._mission_subs = []
        
        # タイマー代わりの最新時刻
        self._start_time = None
        self._vehicle_start_times: Dict[str, float] = {}

        # ウォッチドッグ: 車両が停止してから一定時間後に強制シャットダウン
        # (mission_complete が取りこぼされた場合のフォールバック)
        self._last_moving_sim_time: float = 0.0  # 最後に速度を検知したシミュレーション時刻
        self._vehicle_ever_moved: bool = False
        self._watchdog_timer = self.create_timer(1.0, self._watchdog_tick)
        
        for name in self.vehicle_names:
            sub_grant = self.create_subscription(Bool, f'/{name}/link_grant', lambda msg, vn=name: self._on_link_grant(vn, msg), 10)
            self._grant_subs.append(sub_grant)
            sub_quality = self.create_subscription(CommsQuality, f'/{name}/comms/quality', lambda msg, vn=name: self._on_quality(vn, msg), 1000)
            self._quality_subs.append(sub_quality)

        for name in self.base_vehicle_names:
            # TRANSIENT_LOCAL: tx_controller より遅く起動しても最後のメッセージを受信できる
            sub_mission = self.create_subscription(
                Bool,
                f'/{name}/mission_complete',
                lambda msg, vn=name: self._on_mission_complete(vn, msg),
                _MISSION_QOS
            )
            self._mission_subs.append(sub_mission)

        self.get_logger().info(f'SimLoggerNode (Level {self.logging_level}) 初期化完了')
        atexit.register(self.save_summary_and_close)

    def _set_file_ownership(self, filepath):
        try:
            ws_stat = os.stat(get_workspace_root())
            os.chown(filepath, ws_stat.st_uid, ws_stat.st_gid)
        except Exception:
            pass

    def _on_link_grant(self, vehicle_name: str, msg: Bool):
        self._link_grants[vehicle_name] = msg.data

    def _on_mission_complete(self, vehicle_name: str, msg: Bool):
        if msg.data and not self._mission_status[vehicle_name]:
            self._mission_status[vehicle_name] = True
            if all(self._mission_status.values()):
                self.get_logger().info('=== ミッション完了。すべてのデータを受信するため 3 秒後にシミュレーションを終了します ===')
                self._exit_timer = self.create_timer(3.0, self._exit_now)

    def _exit_now(self):
        self.get_logger().info('=== 終了タイマー満了。シミュレーションを終了します ===')
        self._exit_timer.cancel()
        sys.exit(0)

    def _watchdog_tick(self) -> None:
        """車両が停止してから _WATCHDOG_STOP_TIMEOUT_SEC 秒後に強制シャットダウン。"""
        if all(self._mission_status.values()):
            return  # 正常終了済み
        if not self._vehicle_ever_moved:
            return  # まだ動いていない (初期化待ち)
        sim_now = self._latest_sim_time
        elapsed_stopped = sim_now - self._last_moving_sim_time
        if elapsed_stopped >= _WATCHDOG_STOP_TIMEOUT_SEC:
            self.get_logger().warn(
                f'[Watchdog] 車両が {elapsed_stopped:.1f}s (シミュレーション時間) 停止中。'
                f'mission_complete未受信のため強制終了します。'
                f'(mission_status={self._mission_status})'
            )
            sys.exit(0)

    def _on_quality(self, vehicle_name: str, msg: CommsQuality):
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self._latest_sim_time = current_time
        if self._start_time is None: self._start_time = current_time
        if vehicle_name not in self._vehicle_start_times: self._vehicle_start_times[vehicle_name] = current_time
            
        elapsed = current_time - self._start_time
        vehicle_elapsed = current_time - self._vehicle_start_times[vehicle_name]
        has_grant = self._link_grants.get(vehicle_name, False)

        # 実際の車両位置変化から移動を検出し、ウォッチドッグを更新する
        pos = (msg.tx_x, msg.tx_y, msg.tx_z)
        if vehicle_name in self._last_vehicle_pos:
            last_pos = self._last_vehicle_pos[vehicle_name]
            dx = pos[0] - last_pos[0]
            dy = pos[1] - last_pos[1]
            dz = pos[2] - last_pos[2]
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            if dist > 0.01:  # 前回の品質メッセージから1cm以上移動している場合
                self._vehicle_ever_moved = True
                self._last_moving_sim_time = current_time
        self._last_vehicle_pos[vehicle_name] = pos
        
        # msg.header.stamp を基に動的かつ決定論的に時間差を計算
        prev_time = self._last_time.get(vehicle_name)
        if prev_time is not None:
            dt = current_time - prev_time
            if dt < 0.0:
                dt = 0.0
        else:
            dt = 0.0
        self._last_time[vehicle_name] = current_time

        # --- Level 1: サマリーの集計 ---
        stats = self.summary_stats[vehicle_name]
        stats['total_data'] = max(stats['total_data'], msg.total_data_transmitted)
        if msg.link_state == 'CONNECTED':
            stats['tp_sum'] += msg.throughput
            stats['rssi_sum'] += msg.rssi
            stats['connected_count'] += 1
            stats['connected_time'] += dt

        # --- Handover判定 & 状態更新 (常に実行) ---
        prev_state = self.last_state[vehicle_name]
        state_changed = (prev_state['link_state'] != msg.link_state or prev_state['has_link_grant'] != has_grant)

        if state_changed:
            if prev_state['link_state'] != 'CONNECTED' and msg.link_state == 'CONNECTED':
                stats['handover_count'] += 1

            # --- Level 2: イベントの記録 ---
            if self.logging_level >= 2 and self.event_writer is not None:
                self.event_writer.writerow({
                    'time_s': elapsed,
                    'vehicle_name': vehicle_name,
                    'link_state': msg.link_state,
                    'has_link_grant': has_grant,
                    'rssi_dBm': msg.rssi,
                    'distance_m': msg.distance
                })
                self.event_file.flush()

            self.last_state[vehicle_name]['link_state'] = msg.link_state
            self.last_state[vehicle_name]['has_link_grant'] = has_grant

        # --- Level 3 / 4: 時系列データの記録 (C++側が直接出力するため、Python側では無効化)
        # if self.logging_level >= 3:
        #     if self.logging_level == 3 and msg.link_state != 'CONNECTED':
        #         pass # Level 3 は CONNECTED のみ記録
        #     else:
        #         row = {
        #             'time_s': elapsed,
        #             'vehicle_name': vehicle_name,
        #             'distance_m': msg.distance,
        #             'rssi_dBm': msg.rssi,
        #             'throughput_Gbps': msg.throughput,
        #             'total_data_MB': msg.total_data_transmitted,
        #             'path_loss_dB': msg.path_loss,
        #             'e_gain_dB': msg.antenna_gain_e_plane,
        #             'h_gain_dB': msg.antenna_gain_h_plane,
        #             'tx_x_m': msg.tx_x,
        #             'tx_y_m': msg.tx_y,
        #             'tx_z_m': msg.tx_z,
        #             'bs_x_m': msg.rx_x,
        #             'bs_y_m': msg.rx_y,
        #             'bs_z_m': msg.rx_z
        #         }
        #         if self.logging_level >= 4:
        #             row.update({
        #                 'vehicle_time_s': vehicle_elapsed,
        #                 'has_link_grant': has_grant,
        #                 'comm_active': msg.comm_active,
        #                 'link_state': msg.link_state
        #             })
        #         self.ts_buffers[vehicle_name].append(row)

    def save_summary_and_close(self):
        if getattr(self, '_summary_saved', False):
            return
        self._summary_saved = True
        
        import signal
        handler_int = signal.signal(signal.SIGINT, signal.SIG_IGN)
        handler_term = signal.signal(signal.SIGTERM, signal.SIG_IGN)
        try:
            # C++ノードがCSVを書き終えるまで少し待つ
            import time
            time.sleep(1.0)

            # C++側が出力したデータ欠損のないCSVファイルがあれば、それに基づいて正確な統計値を再計算する
            for vn in self.vehicle_names:
                stats = self.summary_stats[vn]
                suffix = '_connected.csv' if self.logging_level == 3 else '_full.csv'
                csv_path = os.path.join(self.run_dir, 'comms', f'{vn}{suffix}')
                if os.path.exists(csv_path):
                    try:
                        with open(csv_path, 'r', encoding='utf-8') as f:
                            reader = csv.DictReader(f)
                            rows = list(reader)
                        if rows:
                            if self.logging_level == 3:
                                connected_rows = rows
                            else:
                                connected_rows = [r for r in rows if r.get('link_state') == 'CONNECTED']
                            
                            cnt = len(connected_rows)
                            if cnt > 0:
                                stats['rssi_sum'] = sum(float(r['rssi_dBm']) for r in connected_rows)
                                stats['tp_sum'] = sum(float(r['throughput_Gbps']) for r in connected_rows)
                                stats['connected_count'] = cnt
                                
                                if self.logging_level == 3:
                                    conn_time = 0.0
                                    ho_count = 0
                                    prev_t = None
                                    for r in rows:
                                        t = float(r['time_s'])
                                        if prev_t is None:
                                            ho_count += 1
                                        else:
                                            dt = t - prev_t
                                            if dt <= 0.05:
                                                conn_time += dt
                                            else:
                                                ho_count += 1
                                        prev_t = t
                                    stats['connected_time'] = conn_time
                                    stats['handover_count'] = ho_count
                                elif self.logging_level >= 4:
                                    conn_time = 0.0
                                    ho_count = 0
                                    prev_t = None
                                    prev_state = 'DISCONNECTED'
                                    for r in rows:
                                        t = float(r['time_s'])
                                        state = r.get('link_state', 'DISCONNECTED')
                                        if prev_t is not None:
                                            dt = t - prev_t
                                            if prev_state == 'CONNECTED':
                                                conn_time += dt
                                        if prev_state != 'CONNECTED' and state == 'CONNECTED':
                                            ho_count += 1
                                        prev_t = t
                                        prev_state = state
                                    stats['connected_time'] = conn_time
                                    stats['handover_count'] = ho_count
                                
                                stats['total_data'] = max(float(r['total_data_MB']) for r in rows)
                                
                                self.get_logger().info(f"[{vn}] Recalculated summary stats from C++ CSV (no drops)")
                    except Exception as e:
                        self.get_logger().warn(f"Failed to read C++ CSV for summary recalculation: {e}")
            
            # 終了処理 (ファイルを閉じる)
            if self.event_file:
                self.event_file.close()

            if self.output_subdir:
                summary_path = os.path.join(self.output_dir, self.output_subdir, self.summary_filename)
            else:
                match = re.match(r'sweep_summary_(\d{8}_\d{6})_run(\d+)(?:_.*)?\.csv', self.summary_filename)
                if match:
                    sweep_timestamp = match.group(1)
                    summary_path = os.path.join(self.output_dir, f"sweep_{sweep_timestamp}", self.summary_filename)
                else:
                    summary_path = os.path.join(self.output_dir, self.summary_filename)
            
            summary_dir = os.path.dirname(summary_path)
            os.makedirs(summary_dir, exist_ok=True)
            self._set_file_ownership(summary_dir)
            file_exists = os.path.isfile(summary_path)
            
            try:
                with open(summary_path, 'a', newline='', encoding='utf-8') as f:
                    fields = ['run_id', 'y_position', 'antenna_angle', 'vehicle_name', 
                              'total_data_MB', 'connected_time_s', 'average_throughput_Gbps', 
                              'average_rssi_dBm', 'handover_count']
                    writer = csv.DictWriter(f, fieldnames=fields)
                    if not file_exists:
                        writer.writeheader()
                        self._set_file_ownership(summary_path)

                    for vn in self.vehicle_names:
                        stats = self.summary_stats[vn]
                        cnt = stats['connected_count']
                        writer.writerow({
                            'run_id': self.timestamp,
                            'y_position': self.y_pos,
                            'antenna_angle': self.angle,
                            'vehicle_name': vn,
                            'total_data_MB': round(stats['total_data'], 3),
                            'connected_time_s': round(stats['connected_time'], 3),
                            'average_throughput_Gbps': round(stats['tp_sum'] / cnt, 3) if cnt > 0 else 0.0,
                            'average_rssi_dBm': round(stats['rssi_sum'] / cnt, 3) if cnt > 0 else 0.0,
                            'handover_count': stats['handover_count']
                        })
                        
                    shinkansen_vns = [vn for vn in self.vehicle_names if 'shinkansen' in vn]
                    if len(shinkansen_vns) >= 3:
                        total_data = sum(self.summary_stats[vn]['total_data'] for vn in shinkansen_vns)
                        total_connected_time = sum(self.summary_stats[vn]['connected_time'] for vn in shinkansen_vns)
                        total_connected_count = sum(self.summary_stats[vn]['connected_count'] for vn in shinkansen_vns)
                        total_tp_sum = sum(self.summary_stats[vn]['tp_sum'] for vn in shinkansen_vns)
                        total_rssi_sum = sum(self.summary_stats[vn]['rssi_sum'] for vn in shinkansen_vns)
                        total_handover_count = sum(self.summary_stats[vn]['handover_count'] for vn in shinkansen_vns)

                        writer.writerow({
                            'run_id': self.timestamp,
                            'y_position': self.y_pos,
                            'antenna_angle': self.angle,
                            'vehicle_name': 'shinkansen_total',
                            'total_data_MB': round(total_data, 3),
                            'connected_time_s': round(total_connected_time, 3),
                            'average_throughput_Gbps': round(total_tp_sum / total_connected_count, 3) if total_connected_count > 0 else 0.0,
                            'average_rssi_dBm': round(total_rssi_sum / total_connected_count, 3) if total_connected_count > 0 else 0.0,
                            'handover_count': total_handover_count
                        })
                self.get_logger().info(f'サマリー結果を追記しました: {summary_path}')
            except Exception as e:
                self.get_logger().error(f'サマリー保存失敗: {e}')
        finally:
            signal.signal(signal.SIGINT, handler_int)
            signal.signal(signal.SIGTERM, handler_term)

def main(args=None):
    rclpy.init(args=args)
    node = SimLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_summary_and_close()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
