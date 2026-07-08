#!/usr/bin/env python3
"""
kkf_scheduler_node
------------------
KKF予測ハンドオーバーの制御プレーンノード (設計書 §6)。

責務: gz-transport の I/O と周期制御のみ。数理 (KKF地図・ビタビDP) は
comms_sim_pkg.kkf_core に全面委譲し、本ファイルは数式を持たない。

  - /comms/measurement_report 購読 → RSUごとの KKF 地図更新 (第1層)
  - 再計画周期ごとに LCB 系列を構成しビタビDPで計画 (第3層, MPC運用)
  - /comms/ho_schedule へスケジュール配信 (実行はプラグインの ExternalScheduleStrategy)

起動: kkf_scheduler_node.py --config <sim_params.yaml>
(GZ_PARTITION は環境変数から継承。sim_launch.py が control_plane: "kkf_mpc" 時に自動起動)
"""
import argparse
import math
import os
import sys
import threading
import time

import numpy as np
import yaml

from comms_sim_pkg.kkf_core import (
    RoadCoordinate, LogDistanceBasis, KkfParams, Observation,
    KrigedKalmanFilter, solve_handover_plan, TrackerParams, BlockageTracker,
)
from comms_sim_pkg.kkf_core.geometry import rpy_to_rotmat

from gz.transport13 import Node
from comms_sim_proto import comms_sim_msgs_pb2 as msgs


class SchedulerConfig:
    """責務: sim_params.yaml からの制御プレーン設定の読取のみ。"""

    def __init__(self, config_path):
        cfg = yaml.safe_load(open(config_path, encoding='utf-8'))
        link = cfg.get('link_controller_node', {}).get('ros__parameters', {})
        comms = cfg.get('comms_simulator_node', {}).get('ros__parameters', {})
        report_cfg = comms.get('measurement_report', {})

        vehicle = cfg.get('vehicles', [{}])[0]
        self.road_points = [[wp[0], wp[1], wp[2]] for wp in vehicle.get('waypoints', [])]
        self.antenna_offsets = [np.array(a['offset'], dtype=float)
                                for a in vehicle.get('antennas', [])]

        self.bs_positions = []
        for _, bs in sorted(cfg.get('spawn_entities', {}).items()):
            pose = bs['pose']
            rpy_total = np.array(pose[3:6]) + np.array(bs.get('antenna_relative_rpy', [0, 0, 0]))
            rot = rpy_to_rotmat(*rpy_total)
            self.bs_positions.append(
                np.array(pose[0:3]) + rot @ np.array(bs.get('antenna_offset', [0, 0, 0])))

        # C++ KkfConfig と同一のパラメータ名 (link_controller_node.ros__parameters)
        self.replan_period_s = link.get('kkf_replan_period_s', 0.2)
        self.horizon_s = link.get('kkf_horizon_s', 6.0)
        self.plan_dt_s = link.get('kkf_plan_dt_s', 0.3)
        self.kappa = link.get('kkf_kappa', 1.64)
        self.switch_cost = link.get('kkf_switch_cost', 5.0)
        self.velocity_ema_beta = link.get('kkf_velocity_ema_beta', 0.7)
        self.report_topic = report_cfg.get('topic', '/comms/measurement_report')
        self.schedule_topic = link.get('schedule_topic', '/comms/ho_schedule')

        self.kkf_params = KkfParams(
            process_noise_q=link.get('kkf_process_noise_q', 1e-4),
            initial_state_var=link.get('kkf_initial_state_var', 100.0),
            sigma_nu=link.get('kkf_sigma_nu_db', 4.0),
            corr_length_s_m=link.get('kkf_corr_length_s_m', 20.0),
            corr_length_t_s=link.get('kkf_corr_length_t_s', 5.0),
            residual_buffer_size=link.get('kkf_residual_buffer_size', 64),
            initial_state_mean=link.get('kkf_initial_state_mean', [0.0, -20.0]),
        )
        # 観測雑音分散はデータプレーンのレポート雑音設定と一致させる
        self.meas_noise_var = float(report_cfg.get('noise_std_db', 2.0)) ** 2

        # --- 第2層: 遮蔽トラッカー (文書 4章) ---
        self.tracker_enabled = link.get('kkf_tracker_enabled', True)
        self.tracker_params = TrackerParams(
            threshold_db=link.get('kkf_tracker_threshold_db', -10.0),
            cluster_gap_m=link.get('kkf_tracker_cluster_gap_m', 25.0),
            zone_halfwidth_min_m=link.get('kkf_tracker_zone_halfwidth_min_m', 8.0),
            track_timeout_s=link.get('kkf_tracker_timeout_s', 1.5),
            confirm_hits=link.get('kkf_tracker_confirm_hits', 3),
        )
        # 式(11): 遮蔽帯内の観測に先回りで与える σ²_NLOS
        self.nlos_noise_var = float(link.get('kkf_nlos_noise_std_db', 8.0)) ** 2
        # 式(12): 予測遮蔽区間の LCB ペナルティ [dB]
        self.blockage_penalty_db = link.get('kkf_blockage_penalty_db', 15.0)


class KkfMpcScheduler:
    """責務: 観測→地図更新→計画→配信のオーケストレーションのみ。"""

    def __init__(self, cfg: SchedulerConfig):
        self.cfg = cfg
        self.road = RoadCoordinate(cfg.road_points)
        self.maps = [KrigedKalmanFilter(LogDistanceBasis(self.road, p), cfg.kkf_params)
                     for p in cfg.bs_positions]
        # 第2層: RSUごとに独立の遮蔽トラッカー (そのRSUの残差マップの影を追跡)
        self.trackers = [BlockageTracker(cfg.tracker_params)
                         for _ in cfg.bs_positions] if cfg.tracker_enabled else None
        self.num_ant = len(cfg.antenna_offsets)
        self.num_bs = len(cfg.bs_positions)

        self.lock = threading.Lock()
        self.v_hat = 0.0
        self.has_velocity = False
        self.prev_t = -1.0
        self.prev_s = 0.0
        self.next_plan_t = -1e18
        self.current_pair = -1
        self.report_count = 0

        self.node = Node()
        self.pub = self.node.advertise(cfg.schedule_topic, msgs.HoSchedule)
        ok = self.node.subscribe(msgs.MeasurementReport, cfg.report_topic, self.on_report)
        print(f"[kkf_scheduler] subscribe={ok} partition={os.environ.get('GZ_PARTITION', '(default)')} "
              f"bs={self.num_bs} ant={self.num_ant}", flush=True)

    # --- 観測受信 (第1層更新) ---
    def on_report(self, msg):
        with self.lock:
            t = msg.t_sim
            vehicle_pos = np.array([msg.vehicle_pos.x, msg.vehicle_pos.y, msg.vehicle_pos.z])
            s_veh = self.road.project(vehicle_pos)
            self._update_velocity(t, s_veh)

            antenna_s = self._antenna_arc_positions(vehicle_pos, s_veh)
            per_bs = [[] for _ in range(self.num_bs)]
            for e in msg.reports:
                if 0 <= e.bs < self.num_bs and 0 <= e.ant < len(antenna_s):
                    s_obs = antenna_s[e.ant]
                    # 式(11): 予測遮蔽帯内の観測は先回りで観測雑音を減格 (σ²_NLOS)
                    noise_var = self.cfg.meas_noise_var
                    if self.trackers and self.trackers[e.bs].is_blocked(s_obs, t):
                        noise_var = self.cfg.nlos_noise_var
                    per_bs[e.bs].append(Observation(
                        s=s_obs, t=t, z=e.rssi_dbm, noise_var=noise_var))
            for b in range(self.num_bs):
                fresh_residuals = self.maps[b].update(t, per_bs[b])
                if self.trackers:
                    self.trackers[b].ingest(t, fresh_residuals)

            self.report_count += 1
            if t >= self.next_plan_t:
                self._replan(t, antenna_s)
                self.next_plan_t = t + self.cfg.replan_period_s

    def _antenna_arc_positions(self, vehicle_pos, s_veh):
        """車体姿勢は経路接線に沿うと仮定してアンテナ位置を弧長座標へ射影する。"""
        yaw = self.road.tangent_yaw_at(s_veh)
        rot = rpy_to_rotmat(0.0, 0.0, yaw)
        return [self.road.project(vehicle_pos + rot @ off)
                for off in self.cfg.antenna_offsets]

    def _update_velocity(self, t, s_veh):
        if self.prev_t >= 0.0 and t > self.prev_t:
            v_inst = (s_veh - self.prev_s) / (t - self.prev_t)
            beta = self.cfg.velocity_ema_beta
            self.v_hat = beta * self.v_hat + (1 - beta) * v_inst if self.has_velocity else v_inst
            self.has_velocity = True
        self.prev_t = t
        self.prev_s = s_veh

    # --- 再計画 (第3層, 式(12)(13)) ---
    def _replan(self, t, antenna_s):
        cfg = self.cfg
        K = max(1, round(cfg.horizon_s / cfg.plan_dt_s))
        num_states = self.num_ant * self.num_bs
        lcb = np.empty((num_states, K))
        for a in range(self.num_ant):
            for b in range(self.num_bs):
                for k in range(K):
                    s_future = antenna_s[a] + self.v_hat * k * cfg.plan_dt_s
                    t_future = t + k * cfg.plan_dt_s
                    mean, var = self.maps[b].predict_at(s_future, t_future)
                    value = mean - cfg.kappa * math.sqrt(var)
                    # 式(12): 遮蔽帯の通過が予測される区間・時刻にはペナルティを加算
                    if self.trackers and self.trackers[b].is_blocked(s_future, t_future):
                        value -= cfg.blockage_penalty_db
                    lcb[a * self.num_bs + b, k] = value

        assignments, _ = solve_handover_plan(lcb, cfg.plan_dt_s, cfg.switch_cost,
                                             self.current_pair)
        if not assignments:
            return
        self.current_pair = assignments[0]

        sched = msgs.HoSchedule(t_issued=t,
                                valid_until=t + cfg.horizon_s + 2.0 * cfg.replan_period_s)
        prev = -1
        for k, state in enumerate(assignments):
            if state != prev:  # 変化点のみエントリ化
                sched.plan.add(t_start=t + k * cfg.plan_dt_s,
                               ant=state // self.num_bs,
                               bs=state % self.num_bs,
                               mode=msgs.DATA)
                prev = state
        self.pub.publish(sched)

    def spin(self):
        try:
            while True:
                time.sleep(2.0)
                with self.lock:
                    n_tracks = 0
                    if self.trackers:
                        n_tracks = sum(sum(1 for tr in trk.tracks if tr.is_confirmed())
                                       for trk in self.trackers)
                    print(f"[kkf_scheduler] reports={self.report_count} t={self.prev_t:.2f} "
                          f"v̂={self.v_hat:.1f} m/s pair={self.current_pair} "
                          f"tracks={n_tracks}", flush=True)
        except KeyboardInterrupt:
            pass


def main():
    parser = argparse.ArgumentParser(description="KKF MPC control-plane scheduler")
    parser.add_argument('--config', required=True, help="sim_params.yaml のパス")
    # ros2 launch の Node アクションが付加する --ros-args 等は無視する
    args, _ = parser.parse_known_args()

    cfg = SchedulerConfig(args.config)
    if not cfg.road_points or not cfg.bs_positions or not cfg.antenna_offsets:
        print("[kkf_scheduler] ERROR: config に waypoints / spawn_entities / antennas が必要",
              file=sys.stderr)
        sys.exit(1)
    KkfMpcScheduler(cfg).spin()


if __name__ == '__main__':
    main()
