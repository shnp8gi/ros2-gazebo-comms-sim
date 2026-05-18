#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
通信シミュレーション結果を一つのCSVに集約してロギングするノード。
全車両の CommsQuality メッセージと link_grant を監視する。
"""

import atexit
import csv
import os
from datetime import datetime
from typing import Dict, List
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

# カスタムメッセージのインポート
from comms_sim_msgs.msg import CommsQuality

class SimLoggerNode(Node):
    def __init__(self):
        super().__init__('sim_logger_node')
        
        self.declare_parameter('vehicle_names', [''])
        self.declare_parameter('base_vehicle_names', [''])
        self.declare_parameter('output_dir', '/workspace/sim_results/')
        self.declare_parameter('log_only_connected', True)
        
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
        self.log_only_connected = self.get_parameter('log_only_connected').value
        
        self.data_log: List[dict] = []
        self._csv_saved = False
        
        # 状態保持
        self._link_grants: Dict[str, bool] = {name: False for name in self.vehicle_names}
        self._mission_status: Dict[str, bool] = {name: False for name in self.base_vehicle_names}
        
        self._quality_subs = []
        self._grant_subs = []
        self._mission_subs = []
        self._cmd_vel_subs = []
        
        # タイマー代わりの最新時刻
        self._start_time = None
        self._vehicle_start_times: Dict[str, float] = {}
        
        for name in self.vehicle_names:
            # Grant Subscriber
            sub_grant = self.create_subscription(
                Bool,
                f'/{name}/link_grant',
                lambda msg, vn=name: self._on_link_grant(vn, msg),
                10
            )
            self._grant_subs.append(sub_grant)
            
            # Quality Subscriber
            sub_quality = self.create_subscription(
                CommsQuality,
                f'/{name}/comms/quality',
                lambda msg, vn=name: self._on_quality(vn, msg),
                10
            )
            self._quality_subs.append(sub_quality)

        for name in self.base_vehicle_names:
            # Mission Complete Subscriber
            sub_mission = self.create_subscription(
                Bool,
                f'/{name}/mission_complete',
                lambda msg, vn=name: self._on_mission_complete(vn, msg),
                10
            )
            self._mission_subs.append(sub_mission)

            # Cmd Vel Subscriber (移動開始時刻の検知用)
            sub_cmd = self.create_subscription(
                Twist,
                f'/{name}/cmd_vel',
                lambda msg, vn=name: self._on_cmd_vel(vn, msg),
                10
            )
            self._cmd_vel_subs.append(sub_cmd)

        self.get_logger().info(f'SimLoggerNode 初期化完了。監視対象: {self.vehicle_names}')
        atexit.register(self.save_log_to_csv)

    def _on_link_grant(self, vehicle_name: str, msg: Bool):
        self._link_grants[vehicle_name] = msg.data

    def _on_mission_complete(self, vehicle_name: str, msg: Bool):
        if msg.data and not self._mission_status[vehicle_name]:
            self._mission_status[vehicle_name] = True
            self.get_logger().info(f'車両 {vehicle_name} が全ウェイポイントに到達しました。')
            
            if all(self._mission_status.values()):
                self.get_logger().info('=== 全ての車両がミッションを完了しました ===')
                self.get_logger().info('シミュレーションを安全に終了します...')
                # atexit フックによって save_log_to_csv が自動実行される
                sys.exit(0)
    def _on_cmd_vel(self, vehicle_name: str, msg: Twist):
        # 直進速度が検出されたら「動き始めた」と判定して基準時刻を記録
        if abs(msg.linear.x) > 0.001 or abs(msg.linear.y) > 0.001:
            current_time = self.get_clock().now().nanoseconds / 1e9
            
            if self._start_time is None:
                self._start_time = current_time
                self.get_logger().info(f'[{vehicle_name}] の移動開始を検知。全体タイムラインを 0.0秒 として開始します。')
                
            if vehicle_name not in self._vehicle_start_times:
                self._vehicle_start_times[vehicle_name] = current_time
                self.get_logger().info(f'[{vehicle_name}] の移動開始を検知。車両タイムラインを 0.0秒 として開始します。')


    def _on_quality(self, vehicle_name: str, msg: CommsQuality):
        # リンク確立(CONNECTED)状態の期間のみログに残す（設定パラメータに従う）
        if self.log_only_connected and msg.link_state != 'CONNECTED':
            return
            
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        
        # 全体基準時間の初期化 (最初の通信が始まった瞬間を0.0とする)
        if self._start_time is None:
            self._start_time = current_time
            
        # 車両別基準時間の初期化 (その車両の通信が始まった瞬間を0.0とする)
        if vehicle_name not in self._vehicle_start_times:
            self._vehicle_start_times[vehicle_name] = current_time
            
        elapsed = current_time - self._start_time
        vehicle_elapsed = current_time - self._vehicle_start_times[vehicle_name]
        
        log_entry = {
            'time_s': elapsed,
            'vehicle_time_s': vehicle_elapsed,
            'vehicle_name': vehicle_name,
            'has_link_grant': self._link_grants.get(vehicle_name, False),
            'distance_m': msg.distance,
            'rssi_dBm': msg.rssi,
            'throughput_Gbps': msg.throughput,
            'total_data_MB': msg.total_data_transmitted,
            'path_loss_dB': msg.path_loss,
            'e_gain_dB': msg.antenna_gain_e_plane,
            'h_gain_dB': msg.antenna_gain_h_plane,
            'comm_active': msg.comm_active,
            'ugv_x_m': msg.ugv_x,
            'ugv_y_m': msg.ugv_y,
            'ugv_z_m': msg.ugv_z,
            'bs_x_m': msg.base_station_x,
            'bs_y_m': msg.base_station_y,
            'bs_z_m': msg.base_station_z,
            'link_state': msg.link_state,
        }
        self.data_log.append(log_entry)

    def save_log_to_csv(self):
        if self._csv_saved:
            return
        if not self.data_log:
            self.get_logger().info('保存するデータがありません')
            return

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'{timestamp}_combined_results.csv'
        os.makedirs(self.output_dir, exist_ok=True)
        
        try:
            ws_stat = os.stat('/workspace')
            uid, gid = ws_stat.st_uid, ws_stat.st_gid
            os.chown(self.output_dir, uid, gid)
        except Exception:
            uid = gid = -1

        filepath = os.path.join(self.output_dir, filename)

        try:
            with open(filepath, 'w', newline='', encoding='utf-8') as f:
                f.write('# 通信シミュレーション統合結果 (Centralized Log)\n')
                f.write(f'# 対象車両: {", ".join(self.vehicle_names)}\n')
                
                fieldnames = [
                    'time_s', 'vehicle_time_s', 'vehicle_name', 'has_link_grant', 'distance_m', 'rssi_dBm', 
                    'throughput_Gbps', 'total_data_MB', 'path_loss_dB', 'e_gain_dB', 'h_gain_dB',
                    'comm_active', 'ugv_x_m', 'ugv_y_m', 'ugv_z_m', 'bs_x_m', 'bs_y_m', 'bs_z_m',
                    'link_state'
                ]
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                # time_s でソートして書き込むと後で見やすい
                self.data_log.sort(key=lambda x: x['time_s'])
                writer.writerows(self.data_log)
                
            self._csv_saved = True
            if uid != -1:
                os.chown(filepath, uid, gid)
            self.get_logger().info(f'統合データ保存完了: {filepath}')

            # 車両ごとの個別CSVも作成する
            for vehicle_name in self.vehicle_names:
                v_data = [row for row in self.data_log if row['vehicle_name'] == vehicle_name]
                if not v_data:
                    continue
                v_filename = f'{timestamp}_{vehicle_name}_results.csv'
                v_filepath = os.path.join(self.output_dir, v_filename)
                with open(v_filepath, 'w', newline='', encoding='utf-8') as f:
                    f.write(f'# 通信シミュレーション個別結果: {vehicle_name}\n')
                    writer = csv.DictWriter(f, fieldnames=fieldnames)
                    writer.writeheader()
                    writer.writerows(v_data)
                
                if uid != -1:
                    os.chown(v_filepath, uid, gid)
                self.get_logger().info(f'個別データ保存完了: {v_filepath}')

        except Exception as e:
            self.get_logger().error(f'CSV保存失敗: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SimLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_log_to_csv()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
