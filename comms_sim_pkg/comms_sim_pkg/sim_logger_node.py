#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
通信シミュレーション結果を4段階のレベルに分けてロギングするノード。
"""

import atexit
import csv
import os
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

# mission_complete はラッチ（TRANSIENT_LOCAL）で受信する
# ugv_controller_node より sim_logger_node が遅く起動した場合でも受信できる
_MISSION_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

# 車両停止を検知してから強制シャットダウンするまでの猶予時間 [壁時計秒]
# RTF=10（固定）なら 3s壁時計 = 30s シミュレーション時間
_WATCHDOG_STOP_TIMEOUT_SEC = 3.0

class SimLoggerNode(Node):
    def __init__(self):
        super().__init__('sim_logger_node')
        
        self.declare_parameter('vehicle_names', [''])
        self.declare_parameter('base_vehicle_names', [''])
        self.declare_parameter('output_dir', '/workspace/sim_results/')
        self.declare_parameter('logging_level', 1)
        
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
        self.logging_level = self.get_parameter('logging_level').value
        
        self.timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        os.makedirs(self.output_dir, exist_ok=True)
        
        # YAMLからシミュレーション条件を読み込む（サマリー用）
        self.y_pos = 0.0
        self.angle = 0.0
        self.summary_filename = 'sweep_summary.csv'
        try:
            with open('/workspace/config/sim_params.yaml', 'r') as f:
                config = yaml.safe_load(f)
                self.y_pos = float(config.get('spawn_entities', {}).get('antenna', {}).get('pose', [0,0,0,0,0,0])[1])
                self.angle = float(config.get('spawn_entities', {}).get('antenna', {}).get('antenna_relative_rpy', [0,0,0])[2])
                self.summary_filename = config.get('simulation', {}).get('summary_filename', 'sweep_summary.csv')
        except Exception as e:
            self.get_logger().warn(f"Failed to read yaml for summary: {e}")

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
        
        # Level 2: イベントログ用
        self.last_state = {vn: {'link_state': 'DISCONNECTED', 'has_link_grant': False} for vn in self.vehicle_names}
        self.event_file = None
        self.event_writer = None
        if self.logging_level >= 2:
            event_path = os.path.join(self.output_dir, f'{self.timestamp}_events.csv')
            self.event_file = open(event_path, 'w', newline='', encoding='utf-8')
            self.event_writer = csv.DictWriter(self.event_file, fieldnames=[
                'time_s', 'vehicle_name', 'link_state', 'has_link_grant', 'rssi_dBm', 'distance_m'
            ])
            self.event_writer.writeheader()
            self._set_file_ownership(event_path)
            
        # Level 3/4: 時系列ログ用
        self.ts_files = {}
        self.ts_writers = {}
        if self.logging_level >= 3:
            suffix = 'connected' if self.logging_level == 3 else 'full'
            for vn in self.vehicle_names:
                path = os.path.join(self.output_dir, f'{self.timestamp}_{vn}_{suffix}.csv')
                f = open(path, 'w', newline='', encoding='utf-8')
                self.ts_files[vn] = f
                
                if self.logging_level == 3:
                    fields = ['time_s', 'vehicle_name', 'distance_m', 'rssi_dBm', 'throughput_Gbps', 
                              'total_data_MB', 'path_loss_dB', 'e_gain_dB', 'h_gain_dB',
                              'ugv_x_m', 'ugv_y_m', 'ugv_z_m', 'bs_x_m', 'bs_y_m', 'bs_z_m']
                else: # Level 4
                    fields = ['time_s', 'vehicle_time_s', 'vehicle_name', 'has_link_grant', 'distance_m', 
                              'rssi_dBm', 'throughput_Gbps', 'total_data_MB', 'path_loss_dB', 'e_gain_dB', 
                              'h_gain_dB', 'comm_active', 'ugv_x_m', 'ugv_y_m', 'ugv_z_m', 
                              'bs_x_m', 'bs_y_m', 'bs_z_m', 'link_state']
                
                self.ts_writers[vn] = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
                self.ts_writers[vn].writeheader()
                self._set_file_ownership(path)

        # 状態保持
        self._link_grants: Dict[str, bool] = {name: False for name in self.vehicle_names}
        self._mission_status: Dict[str, bool] = {name: False for name in self.base_vehicle_names}
        self._last_time = {name: None for name in self.vehicle_names}
        
        self._quality_subs = []
        self._grant_subs = []
        self._mission_subs = []
        self._cmd_vel_subs = []
        
        # タイマー代わりの最新時刻
        self._start_time = None
        self._vehicle_start_times: Dict[str, float] = {}

        # ウォッチドッグ: 車両が停止してから一定時間後に強制シャットダウン
        # (mission_complete が取りこぼされた場合のフォールバック)
        self._last_moving_wall_time: float = 0.0  # 最後に速度を検知した壁時計時刻
        self._vehicle_ever_moved: bool = False
        self._watchdog_timer = self.create_timer(1.0, self._watchdog_tick)
        
        for name in self.vehicle_names:
            sub_grant = self.create_subscription(Bool, f'/{name}/link_grant', lambda msg, vn=name: self._on_link_grant(vn, msg), 10)
            self._grant_subs.append(sub_grant)
            sub_quality = self.create_subscription(CommsQuality, f'/{name}/comms/quality', lambda msg, vn=name: self._on_quality(vn, msg), 10)
            self._quality_subs.append(sub_quality)

        for name in self.base_vehicle_names:
            # TRANSIENT_LOCAL: ugv_controller より遅く起動しても最後のメッセージを受信できる
            sub_mission = self.create_subscription(
                Bool,
                f'/{name}/mission_complete',
                lambda msg, vn=name: self._on_mission_complete(vn, msg),
                _MISSION_QOS
            )
            self._mission_subs.append(sub_mission)
            sub_cmd = self.create_subscription(Twist, f'/{name}/cmd_vel', lambda msg, vn=name: self._on_cmd_vel(vn, msg), 10)
            self._cmd_vel_subs.append(sub_cmd)

        self.get_logger().info(f'SimLoggerNode (Level {self.logging_level}) 初期化完了')
        atexit.register(self.save_summary_and_close)

    def _set_file_ownership(self, filepath):
        try:
            ws_stat = os.stat('/workspace')
            os.chown(filepath, ws_stat.st_uid, ws_stat.st_gid)
        except Exception:
            pass

    def _on_link_grant(self, vehicle_name: str, msg: Bool):
        self._link_grants[vehicle_name] = msg.data

    def _on_mission_complete(self, vehicle_name: str, msg: Bool):
        if msg.data and not self._mission_status[vehicle_name]:
            self._mission_status[vehicle_name] = True
            if all(self._mission_status.values()):
                self.get_logger().info('=== ミッション完了。シミュレーションを終了します ===')
                import time
                time.sleep(0.5)
                sys.exit(0)

    def _watchdog_tick(self) -> None:
        """車両が停止してから _WATCHDOG_STOP_TIMEOUT_SEC 秒後に強制シャットダウン。"""
        import time as _time
        if all(self._mission_status.values()):
            return  # 正常終了済み
        if not self._vehicle_ever_moved:
            return  # まだ動いていない (初期化待ち)
        wall_now = _time.monotonic()
        elapsed_stopped = wall_now - self._last_moving_wall_time
        if elapsed_stopped >= _WATCHDOG_STOP_TIMEOUT_SEC:
            self.get_logger().warn(
                f'[Watchdog] 車両が {elapsed_stopped:.1f}s 停止中。'
                f'mission_complete未受信のため強制終了します。'
                f'(mission_status={self._mission_status})'
            )
            sys.exit(0)

    def _on_cmd_vel(self, vehicle_name: str, msg: Twist):
        import time as _time
        if abs(msg.linear.x) > 0.001 or abs(msg.linear.y) > 0.001:
            current_time = self.get_clock().now().nanoseconds / 1e9
            if self._start_time is None:
                self._start_time = current_time
            if vehicle_name not in self._vehicle_start_times:
                self._vehicle_start_times[vehicle_name] = current_time
            # ウォッチドッグ更新
            self._vehicle_ever_moved = True
            self._last_moving_wall_time = _time.monotonic()

    def _on_quality(self, vehicle_name: str, msg: CommsQuality):
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if self._start_time is None: self._start_time = current_time
        if vehicle_name not in self._vehicle_start_times: self._vehicle_start_times[vehicle_name] = current_time
            
        elapsed = current_time - self._start_time
        vehicle_elapsed = current_time - self._vehicle_start_times[vehicle_name]
        has_grant = self._link_grants.get(vehicle_name, False)
        
        # 各comms_nodeのサンプリング周波数(100Hz)に合わせた決定論的な固定時間ステップを使用
        dt = 0.01
        self._last_time[vehicle_name] = current_time

        # --- Level 1: サマリーの集計 ---
        stats = self.summary_stats[vehicle_name]
        stats['total_data'] = max(stats['total_data'], msg.total_data_transmitted)
        if msg.link_state == 'CONNECTED':
            stats['tp_sum'] += msg.throughput
            stats['rssi_sum'] += msg.rssi
            stats['connected_count'] += 1
            stats['connected_time'] += dt

        # --- Level 2: イベントの記録 ---
        if self.logging_level >= 2:
            prev_state = self.last_state[vehicle_name]
            # Handover判定 (Grantが付与/剥奪された) または LinkStateが変化した
            if prev_state['link_state'] != msg.link_state or prev_state['has_link_grant'] != has_grant:
                if prev_state['link_state'] == 'DISCONNECTED' and msg.link_state == 'CONNECTED':
                    stats['handover_count'] += 1 # 接続確立をハンドオーバーとみなすか？ 一旦カウントする
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

        # --- Level 3 / 4: 時系列データの記録 ---
        if self.logging_level >= 3:
            if self.logging_level == 3 and msg.link_state != 'CONNECTED':
                pass # Level 3 は CONNECTED のみ記録
            else:
                row = {
                    'time_s': elapsed,
                    'vehicle_name': vehicle_name,
                    'distance_m': msg.distance,
                    'rssi_dBm': msg.rssi,
                    'throughput_Gbps': msg.throughput,
                    'total_data_MB': msg.total_data_transmitted,
                    'path_loss_dB': msg.path_loss,
                    'e_gain_dB': msg.antenna_gain_e_plane,
                    'h_gain_dB': msg.antenna_gain_h_plane,
                    'ugv_x_m': msg.ugv_x,
                    'ugv_y_m': msg.ugv_y,
                    'ugv_z_m': msg.ugv_z,
                    'bs_x_m': msg.base_station_x,
                    'bs_y_m': msg.base_station_y,
                    'bs_z_m': msg.base_station_z
                }
                if self.logging_level == 4:
                    row.update({
                        'vehicle_time_s': vehicle_elapsed,
                        'has_link_grant': has_grant,
                        'comm_active': msg.comm_active,
                        'link_state': msg.link_state
                    })
                self.ts_writers[vehicle_name].writerow(row)

    def save_summary_and_close(self):
        if getattr(self, '_summary_saved', False):
            return
        self._summary_saved = True
        
        # 終了処理 (ファイルを閉じる)
        if self.event_file:
            self.event_file.close()
        for f in self.ts_files.values():
            f.close()

        summary_path = os.path.join(self.output_dir, self.summary_filename)
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

                    writer.writerow({
                        'run_id': self.timestamp,
                        'y_position': self.y_pos,
                        'antenna_angle': self.angle,
                        'vehicle_name': 'shinkansen_total',
                        'total_data_MB': round(total_data, 3)
                    })
            self.get_logger().info(f'サマリー結果を追記しました: {summary_path}')
        except Exception as e:
            self.get_logger().error(f'サマリー保存失敗: {e}')

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
