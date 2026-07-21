#!/usr/bin/env python3
"""
kkf_scheduler_node
------------------
予測ハンドオーバーの制御プレーンノード (設計書 §6)。

責務: gz-transport の I/O と周期制御のみ。数理 (予測器・ビタビDP) は
comms_sim_pkg.kkf_core に全面委譲し、本ファイルは数式を持たない。

制御則は link_controller_node.control_plane で選択する:
  - "kkf_mpc": KKF地図 (第1層) + 遮蔽トラッカー (第2層) + ビタビDP — 提案手法
  - "ts_kf":   ペアごとのスカラーKF時系列外挿 + ビタビDP — 先行研究ベースライン
                (空間構造・遮蔽追跡なし。プランナ・実行系は kkf_mpc と共通)
  - "a3":      A3イベント型 (hysteresis+TTT、MEASUREスロットで近傍プローブ)
                — 予測なしの標準ベースライン (kkf_core.a3 参照)

複数Tx車両 (順次配信):
  - 全車両のレポートを単一トピックで購読し、レポートの vehicle フィールドで振り分ける
  - KKF地図はBSごとに車間共有 (全車が同一路線の弧長座標に射影されるため、
    先行車の測定が後続車の予測に寄与する)
  - BS排他 (1BSは同時に1台のみ) は逐次優先度付き計画で解決する:
    進入順に各車をビタビDPで計画し、上位車が占有する (bs, k) をマスクする。
    アイドル状態 (bs=-1) を持ち、有望なリンクが無い車はBSを占有しない
  - スケジュールはエントリごとに vehicle を付与して一括配信 (プラグイン側で選別)
  - a3/静的LUTは車間調整を行わない (分散制御の正直なコスト。衝突はデータプレーン
    の BsOccupancyRegistry が物理的に拒否する)

  - /comms/measurement_report 購読 → 予測器の更新
  - 再計画周期ごとに LCB 系列を構成しビタビDPで計画 (第3層, MPC運用)
  - /comms/ho_schedule へスケジュール配信 (実行はプラグインの ExternalScheduleStrategy)

起動: kkf_scheduler_node.py --config <sim_params.yaml>
(GZ_PARTITION は環境変数から継承。sim_launch.py が対応する control_plane 設定時に自動起動)
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
    RoadCoordinate, ConstantBasis, LogDistanceBasis, KkfParams, Observation,
    KrigedKalmanFilter, solve_handover_plan, TrackerParams, BlockageTracker,
    ScalarKfParams, ScalarRssiKF, A3Params, A3Controller,
)
from comms_sim_pkg.kkf_core.geometry import rpy_to_rotmat

from gz.transport13 import Node
from comms_sim_proto import comms_sim_msgs_pb2 as msgs

# 占有マスク: 上位優先車が使うBSの (state, k) に与える値 (選択不能)
MASKED_LCB = -1e6


class SchedulerConfig:
    """責務: sim_params.yaml からの制御プレーン設定の読取のみ。"""

    def __init__(self, config_path):
        cfg = yaml.safe_load(open(config_path, encoding='utf-8'))
        link = cfg.get('link_controller_node', {}).get('ros__parameters', {})
        comms = cfg.get('comms_simulator_node', {}).get('ros__parameters', {})
        report_cfg = comms.get('measurement_report', {})

        # 全Tx車両 (定義順 = 進入順 = 計画の優先順)
        self.vehicles = []
        for v in cfg.get('vehicles', []):
            self.vehicles.append({
                'name': v.get('name'),
                'antenna_offsets': [np.array(a['offset'], dtype=float)
                                    for a in v.get('antennas', [])],
            })
        # 路線形状は全車共通 (同一路線の順次通過を前提)。進入位置が車両ごとに
        # 異なるため、弧長座標が全車の走行範囲を覆うよう最長経路の車両を採用する
        def _path_len(v):
            wps = v.get('waypoints', [])
            return sum(math.dist(wps[i][:3], wps[i + 1][:3])
                       for i in range(len(wps) - 1)) if len(wps) >= 2 else 0.0
        road_src = max(cfg.get('vehicles', [{}]), key=_path_len, default={})
        self.road_points = [[wp[0], wp[1], wp[2]] for wp in road_src.get('waypoints', [])]

        self.bs_positions = []
        for _, bs in sorted(cfg.get('spawn_entities', {}).items()):
            pose = bs['pose']
            rpy_total = np.array(pose[3:6]) + np.array(bs.get('antenna_relative_rpy', [0, 0, 0]))
            rot = rpy_to_rotmat(*rpy_total)
            self.bs_positions.append(
                np.array(pose[0:3]) + rot @ np.array(bs.get('antenna_offset', [0, 0, 0])))

        self.control_plane = link.get('control_plane', 'kkf_mpc')

        # 車間の計画優先度:
        #   "entry"        定義順 = 進入順の固定優先 (従来)。需要超過
        #                  (車数 > BS数) では下位車が構造的に飢餓する
        #   "deficit_time" 累積grant時間が最も少ない車から計画する
        #                  (max-min 志向の飢餓解消。最小変更の動的優先度)
        self.priority_mode = link.get('kkf_priority', 'entry')

        # 事前地図 (決定論RSSIプロファイル): プラグインが export_rssi_profile で
        # sim_results/<output_subdir>/rssi_profiles/<model>_profile.csv を出力する
        self.use_prior_profile = bool(link.get('export_rssi_profile', False))
        subdir = cfg.get('simulation', {}).get('output_subdir', '')
        self.profile_dir = (f"/workspace/sim_results/{subdir}/rssi_profiles"
                            if subdir else None)

        # C++ KkfConfig と同一のパラメータ名 (link_controller_node.ros__parameters)
        self.replan_period_s = link.get('kkf_replan_period_s', 0.2)
        self.horizon_s = link.get('kkf_horizon_s', 6.0)
        self.plan_dt_s = link.get('kkf_plan_dt_s', 0.3)
        self.kappa = link.get('kkf_kappa', 1.64)
        self.switch_cost = link.get('kkf_switch_cost', 5.0)
        self.velocity_ema_beta = link.get('kkf_velocity_ema_beta', 0.7)
        # アイドル状態のLCB: 全ペアがこれを下回る (=見込みが皆無) 時のみBSを掴まない。
        # LCBは前方予測の κσ を含み実RSSIよりかなり悲観的なため、実リンク下限より
        # 十分低くしておく (高すぎると過剰アイドルで転送機会を失う)
        self.idle_lcb_db = link.get('kkf_idle_lcb_db', -110.0)
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
        # 式(10): 遮蔽帯内の観測に先回りで与える σ²_NLOS
        self.nlos_noise_var = float(link.get('kkf_nlos_noise_std_db', 8.0)) ** 2
        # 式(11): 予測遮蔽区間の LCB ペナルティ [dB]
        self.blockage_penalty_db = link.get('kkf_blockage_penalty_db', 15.0)

        # --- ts_kf ベースライン: ペア単位スカラーKF ---
        self.scalar_kf_params = ScalarKfParams(
            process_noise_q=link.get('tskf_process_noise_q', 25.0),
            initial_level_var=link.get('tskf_initial_level_var', 400.0),
            initial_rate_var=link.get('tskf_initial_rate_var', 100.0),
            prior_mean_dbm=link.get('tskf_prior_mean_dbm', -120.0),
            prior_var=link.get('tskf_prior_var', 400.0),
        )

        # --- a3 ベースライン: A3イベント則 ---
        self.a3_params = A3Params(
            hysteresis_db=link.get('a3_hysteresis_db', 3.0),
            time_to_trigger_s=link.get('a3_time_to_trigger_s', 0.1),
            l3_filter_beta=link.get('a3_l3_filter_beta', 0.5),
            measure_period_s=link.get('a3_measure_period_s', 0.4),
            measure_duration_s=link.get('a3_measure_duration_s', 0.06),
            estimate_timeout_s=link.get('a3_estimate_timeout_s', 4.0),
        )


class VehicleState:
    """車両ごとの運動推定と直近観測位置 (制御プレーン側の追跡状態)。"""

    def __init__(self, name, antenna_offsets):
        self.name = name
        self.antenna_offsets = antenna_offsets
        self.v_hat = 0.0
        self.has_velocity = False
        self.prev_t = -1.0
        self.prev_s = 0.0
        self.antenna_s = None      # 直近レポート時の各アンテナ弧長 [m]
        self.last_report_t = -1.0
        self.current_state = -1    # DP状態 (num_states=idle、-1=未接続)

    def update_kinematics(self, t, s_veh, beta):
        if self.prev_t >= 0.0 and t > self.prev_t:
            v_inst = (s_veh - self.prev_s) / (t - self.prev_t)
            self.v_hat = (beta * self.v_hat + (1 - beta) * v_inst
                          if self.has_velocity else v_inst)
            self.has_velocity = True
        self.prev_t = t
        self.prev_s = s_veh

    def antenna_s_at(self, t):
        """直近観測のアンテナ弧長を等速外挿で時刻 t へ進める。"""
        if self.antenna_s is None:
            return None
        dt = max(0.0, t - self.last_report_t)
        return [s + self.v_hat * dt for s in self.antenna_s]


class PriorProfile:
    """車両ごとの決定論RSSIプロファイル (事前地図 = 事前測量) の平均関数 m(ant,bs,s)。

    プラグインが export_rssi_profile で出力する CSV を読み、経路弧長 s 上の
    区分線形補間として提供する。
    """

    def __init__(self, csv_path, road: RoadCoordinate):
        rows = np.genfromtxt(csv_path, delimiter=',', names=True)
        names = rows.dtype.names
        pos = np.stack([rows['px'], rows['py'], rows['pz']], axis=1)
        s = np.array([road.project(p) for p in pos])
        order = np.argsort(s)
        self.s = s[order]
        self.table = {}
        for name in names:
            if name.startswith('rssi_'):
                _, a, b = name.split('_')
                self.table[(int(a), int(b))] = np.asarray(rows[name])[order]

    def mean(self, ant, bs, s_query):
        vals = self.table.get((ant, bs))
        if vals is None:
            return None
        return float(np.interp(s_query, self.s, vals))

    def mean_batch(self, ant, bs, s_arr):
        vals = self.table.get((ant, bs))
        if vals is None:
            return np.zeros(len(s_arr))
        return np.interp(np.asarray(s_arr, dtype=float), self.s, vals)


def load_prior_profiles(cfg: SchedulerConfig, road: RoadCoordinate,
                        timeout_s=30.0):
    """全車両のプロファイルCSVが出揃うのを待って読み込む (プラグインが起動時に出力)。"""
    if not (cfg.use_prior_profile and cfg.profile_dir):
        return None
    names = [v['name'] for v in cfg.vehicles]
    deadline = time.monotonic() + timeout_s
    paths = {n: os.path.join(cfg.profile_dir, f"{n}_profile.csv") for n in names}
    prev_sizes = None
    while time.monotonic() < deadline:
        try:
            sizes = {n: os.path.getsize(p) for n, p in paths.items()}
        except OSError:
            sizes = None
        # 書き込み途中の読み込みを避ける: 全ファイルのサイズが1周期安定してから読む
        if sizes and all(v > 0 for v in sizes.values()) and sizes == prev_sizes:
            try:
                profiles = {n: PriorProfile(p, road) for n, p in paths.items()}
                print(f"[kkf_scheduler] prior profiles loaded: "
                      f"{ {n: len(pr.s) for n, pr in profiles.items()} }", flush=True)
                return profiles
            except Exception as e:  # 途中読みでのパース失敗はリトライ
                print(f"[kkf_scheduler] prior profile parse retry: {e}", flush=True)
        prev_sizes = sizes
        time.sleep(0.5)
    print("[kkf_scheduler] WARN: prior profiles not found, "
          "falling back to log-distance basis", flush=True)
    return None


class KkfMapPredictor:
    """提案手法: RSUごとのKKF地図 (第1層) + 遮蔽トラッカー (第2層)。

    地図は弧長座標上の場であり車両に依存しない = 複数車両で自然に共有される
    (先行車の測定が後続車の予測を改善する)。

    事前地図 (PriorProfile) がある場合は「決定論プロファイルを平均関数、
    KKFは偏差 (シャドウイング・遮蔽・モデル誤差) のみ学習」のハイブリッド動作。
    事前地図が無い場合は従来どおり対数距離トレンドで RSSI を直接学習する。
    """

    def __init__(self, cfg: SchedulerConfig, road: RoadCoordinate, priors=None):
        self.cfg = cfg
        self.priors = priors
        if priors:
            self.maps = [KrigedKalmanFilter(ConstantBasis(), cfg.kkf_params)
                         for _ in cfg.bs_positions]
        else:
            self.maps = [KrigedKalmanFilter(LogDistanceBasis(road, p), cfg.kkf_params)
                         for p in cfg.bs_positions]
        # 第2層: RSUごとに独立の遮蔽トラッカー (そのRSUの残差マップの影を追跡)
        self.trackers = [BlockageTracker(cfg.tracker_params)
                         for _ in cfg.bs_positions] if cfg.tracker_enabled else None

    def _prior_mean(self, vid, ant, bs, s):
        if not self.priors or vid not in self.priors:
            return 0.0
        m = self.priors[vid].mean(ant, bs, s)
        return 0.0 if m is None else m

    def ingest(self, vid, t, antenna_s, entries):
        """entries: [(ant, bs, rssi_dbm)] を地図更新・トラッカー観測に振り分ける。"""
        per_bs = [[] for _ in self.maps]
        for ant, bs, rssi in entries:
            s_obs = antenna_s[ant]
            z = rssi
            if self.priors:
                z = rssi - self._prior_mean(vid, ant, bs, s_obs)  # 偏差のみ学習
            # 式(10): 予測遮蔽帯内の観測は先回りで観測雑音を減格 (σ²_NLOS)
            noise_var = self.cfg.meas_noise_var
            if self.trackers and self.trackers[bs].is_blocked(s_obs, t):
                noise_var = self.cfg.nlos_noise_var
            per_bs[bs].append(Observation(s=s_obs, t=t, z=z, noise_var=noise_var))
        for b, obs in enumerate(per_bs):
            fresh_residuals = self.maps[b].update(t, obs)
            if self.trackers:
                self.trackers[b].ingest(t, fresh_residuals)

    def lcb(self, vid, ant, bs, s_future, t_future, kappa):
        mean, var = self.maps[bs].predict_at(s_future, t_future)
        if self.priors:
            mean += self._prior_mean(vid, ant, bs, s_future)
        value = mean - kappa * math.sqrt(var)
        # 式(11) 第3項: 遮蔽帯の通過が予測される区間・時刻にはペナルティを加算
        if self.trackers and self.trackers[bs].is_blocked(s_future, t_future):
            value -= self.cfg.blockage_penalty_db
        return value

    def lcb_batch(self, vid, ant, bs, s_arr, t_arr, kappa):
        """ステージ一括の LCB (再計画の計算量ボトルネック解消用)。"""
        means, variances = self.maps[bs].predict_batch(s_arr, t_arr)
        if self.priors and vid in self.priors:
            means = means + self.priors[vid].mean_batch(ant, bs, s_arr)
        values = means - kappa * np.sqrt(variances)
        if self.trackers:
            trk = self.trackers[bs]
            for i, (s, t) in enumerate(zip(s_arr, t_arr)):
                if trk.is_blocked(s, t):
                    values[i] -= self.cfg.blockage_penalty_db
        return values

    def status(self):
        n_tracks = 0
        if self.trackers:
            n_tracks = sum(sum(1 for tr in trk.tracks if tr.is_confirmed())
                           for trk in self.trackers)
        return f"tracks={n_tracks} prior={'on' if self.priors else 'off'}"

    def track_detail(self, t):
        """検証用: 確定トラックの (BS index, 中心弧長s, 速度) を返す。"""
        out = []
        if self.trackers:
            for b, trk in enumerate(self.trackers):
                for tr in trk.tracks:
                    if tr.is_confirmed():
                        out.append((b, tr.predict_center(t), tr.velocity))
        return out


class TimeSeriesKfPredictor:
    """先行研究ベースライン: (車両, アンテナ, BS) ごとのRSSI時系列スカラーKF外挿。

    空間構造を持たないため、前方の未観測区間の遮蔽・利得変化は原理的に
    予測できない (s_future は受け取るが使わない)。車間の情報共有もない。
    """

    def __init__(self, cfg: SchedulerConfig, num_bs):
        self.cfg = cfg
        self.num_bs = num_bs
        self.filters = {}  # (vid, ant, bs) -> ScalarRssiKF

    def _filter(self, vid, ant, bs):
        key = (vid, ant, bs)
        if key not in self.filters:
            self.filters[key] = ScalarRssiKF(self.cfg.scalar_kf_params)
        return self.filters[key]

    def ingest(self, vid, t, antenna_s, entries):
        for ant, bs, rssi in entries:
            self._filter(vid, ant, bs).update(t, rssi, self.cfg.meas_noise_var)

    def lcb(self, vid, ant, bs, s_future, t_future, kappa):
        mean, var = self._filter(vid, ant, bs).predict_at(t_future)
        return mean - kappa * math.sqrt(var)

    def lcb_batch(self, vid, ant, bs, s_arr, t_arr, kappa):
        f = self._filter(vid, ant, bs)
        out = np.empty(len(t_arr))
        for i, t in enumerate(t_arr):
            mean, var = f.predict_at(t)
            out[i] = mean - kappa * math.sqrt(var)
        return out

    def status(self):
        n_init = sum(1 for f in self.filters.values() if f.initialized())
        return f"kf_init={n_init}/{len(self.filters)}"


def make_predictor(cfg: SchedulerConfig, road: RoadCoordinate):
    if cfg.control_plane == 'kkf_mpc':
        priors = load_prior_profiles(cfg, road)
        return KkfMapPredictor(cfg, road, priors)
    if cfg.control_plane == 'ts_kf':
        return TimeSeriesKfPredictor(cfg, len(cfg.bs_positions))
    raise ValueError(f"未知の control_plane: {cfg.control_plane}")


class MpcScheduler:
    """責務: 観測→予測器更新→逐次優先度付き計画→配信のオーケストレーションのみ。"""

    def __init__(self, cfg: SchedulerConfig):
        self.cfg = cfg
        self.road = RoadCoordinate(cfg.road_points)
        self.num_bs = len(cfg.bs_positions)
        self.predictor = make_predictor(cfg, self.road)

        # 進入順 (=定義順) が kkf_priority: "entry" 時の計画優先順
        self.vstates = {v['name']: VehicleState(v['name'], v['antenna_offsets'])
                        for v in cfg.vehicles}
        self.default_vid = cfg.vehicles[0]['name']

        # 車両ごとの累積grant時間の推定 [s] (deficit_time 優先度のキー)。
        # 実配信バイトのフィードバックは無いため、自ノードが発行した計画の
        # k=0 が非idleだった時間 (replan周期の積算) を提供サービス量の代理とする
        self.granted_time = {v['name']: 0.0 for v in cfg.vehicles}

        self.lock = threading.Lock()
        self.inbox = []
        self.next_plan_t = -1e18
        self.report_count = 0
        self.last_t = -1.0

        self.node = Node()
        self.pub = self.node.advertise(cfg.schedule_topic, msgs.HoSchedule)
        ok = self.node.subscribe(msgs.MeasurementReport, cfg.report_topic, self.on_report)
        print(f"[kkf_scheduler] control_plane={cfg.control_plane} subscribe={ok} "
              f"partition={os.environ.get('GZ_PARTITION', '(default)')} "
              f"bs={self.num_bs} vehicles={list(self.vstates)}", flush=True)

    # --- 観測受信。gz-transport のコールバックスレッドで numpy/クリギング等の
    #     処理を行うと pybind11 の GIL 状態異常 (scoped_acquire::dec_ref) で
    #     クラッシュするため、ここではメッセージの退避のみ行い、
    #     取り込み・計画・配信のすべてを spin スレッド側で実行する ---
    def on_report(self, msg):
        with self.lock:
            self.inbox.append(msg)

    def _process_report(self, msg):
        t = msg.t_sim
        vid = msg.vehicle or self.default_vid
        vs = self.vstates.get(vid)
        if vs is None:
            return
        self.last_t = max(self.last_t, t)

        vehicle_pos = np.array([msg.vehicle_pos.x, msg.vehicle_pos.y, msg.vehicle_pos.z])
        s_veh = self.road.project(vehicle_pos)
        vs.update_kinematics(t, s_veh, self.cfg.velocity_ema_beta)

        yaw = self.road.tangent_yaw_at(s_veh)
        rot = rpy_to_rotmat(0.0, 0.0, yaw)
        vs.antenna_s = [self.road.project(vehicle_pos + rot @ off)
                        for off in vs.antenna_offsets]
        vs.last_report_t = t

        entries = [(e.ant, e.bs, e.rssi_dbm) for e in msg.reports
                   if 0 <= e.bs < self.num_bs and 0 <= e.ant < len(vs.antenna_s)]
        self.predictor.ingest(vid, t, vs.antenna_s, entries)

        self.report_count += 1

    # --- 再計画 (第3層 + 車間の逐次優先度付き割当) ---
    def _replan(self, t):
        wall0 = time.monotonic()
        cfg = self.cfg
        K = max(1, round(cfg.horizon_s / cfg.plan_dt_s))
        occupied = [set() for _ in range(K)]  # k -> 上位車が占有するBS集合

        sched = msgs.HoSchedule(t_issued=t,
                                valid_until=t + cfg.horizon_s + 2.0 * cfg.replan_period_s)
        published_any = False

        # 計画順 = 車間優先度。deficit_time は「累積grant時間が最少の車から」
        # 計画してBSを先取りさせる (占有マスクは計画順の後続にのみ及ぶため、
        # 順序の動的化だけで飢餓が解消される)。タイブレークは定義順で決定的
        plan_order = self.cfg.vehicles
        if cfg.priority_mode == 'deficit_time':
            order_idx = {v['name']: i for i, v in enumerate(self.cfg.vehicles)}
            plan_order = sorted(
                self.cfg.vehicles,
                key=lambda v: (self.granted_time[v['name']], order_idx[v['name']]))

        for v in plan_order:
            vs = self.vstates[v['name']]
            ant_s = vs.antenna_s_at(t)
            if ant_s is None:
                continue  # 未進入 (レポート未受信)
            num_ant = len(ant_s)
            num_states = num_ant * self.num_bs
            idle = num_states  # アイドル行: どのBSも掴まない

            k_grid = np.arange(K)
            t_grid = t + k_grid * cfg.plan_dt_s
            lcb = np.empty((num_states + 1, K))
            lcb[idle, :] = cfg.idle_lcb_db
            for a in range(num_ant):
                s_grid = ant_s[a] + vs.v_hat * k_grid * cfg.plan_dt_s
                for b in range(self.num_bs):
                    row = a * self.num_bs + b
                    lcb[row, :] = self.predictor.lcb_batch(
                        v['name'], a, b, s_grid, t_grid, cfg.kappa)
                    for k in range(K):
                        if b in occupied[k]:
                            lcb[row, k] = MASKED_LCB

            assignments, _ = solve_handover_plan(lcb, cfg.plan_dt_s, cfg.switch_cost,
                                                 vs.current_state)
            if not assignments:
                continue
            vs.current_state = assignments[0]
            if assignments[0] != idle:
                self.granted_time[v['name']] += cfg.replan_period_s

            if os.environ.get('KKF_DEBUG'):
                real = lcb[:num_states, :]
                print(f"[kkf_debug] t={t:.2f} {v['name']} s={ant_s[0]:.1f} "
                      f"lcb_max={real.max():.1f} lcb_k0_max={real[:, 0].max():.1f} "
                      f"idle={cfg.idle_lcb_db} pick={assignments[0]}", flush=True)

            prev = -2
            for k, state in enumerate(assignments):
                if state != idle:
                    occupied[k].add(state % self.num_bs)
                if state != prev:  # 変化点のみエントリ化
                    if state == idle:
                        sched.plan.add(t_start=t + k * cfg.plan_dt_s,
                                       ant=0, bs=-1, mode=msgs.DATA,
                                       vehicle=v['name'])
                    else:
                        sched.plan.add(t_start=t + k * cfg.plan_dt_s,
                                       ant=state // self.num_bs,
                                       bs=state % self.num_bs,
                                       mode=msgs.DATA, vehicle=v['name'])
                    prev = state
            published_any = True

        if published_any:
            self.pub.publish(sched)
        if os.environ.get('KKF_DEBUG'):
            print(f"[kkf_debug] replan_wall={1e3 * (time.monotonic() - wall0):.0f}ms",
                  flush=True)
        if os.environ.get('KKF_TRACK_DEBUG') and hasattr(self.predictor, 'track_detail'):
            for b, s, v in self.predictor.track_detail(t):
                print(f"[track_debug] t={t:.2f} bs={b} s_center={s:.1f} v_est={v:+.1f}",
                      flush=True)

    def spin(self):
        """取込・計画・配信ループ (メインスレッド)。sim time はレポート由来の last_t。"""
        last_status_wall = time.monotonic()
        try:
            while True:
                time.sleep(0.01)
                with self.lock:
                    pending, self.inbox = self.inbox, []
                for msg in pending:
                    self._process_report(msg)

                if self.last_t >= 0.0 and self.last_t >= self.next_plan_t:
                    self._replan(self.last_t)
                    self.next_plan_t = self.last_t + self.cfg.replan_period_s

                if time.monotonic() - last_status_wall >= 2.0:
                    last_status_wall = time.monotonic()
                    kin = " ".join(
                        f"{name}:v̂={vs.v_hat:.1f},st={vs.current_state},"
                        f"gt={self.granted_time.get(name, 0.0):.1f}"
                        for name, vs in self.vstates.items() if vs.antenna_s is not None)
                    print(f"[kkf_scheduler] reports={self.report_count} t={self.last_t:.2f} "
                          f"{kin} {self.predictor.status()}", flush=True)
        except KeyboardInterrupt:
            pass


class A3Scheduler:
    """責務: 車両ごとの A3コントローラ (kkf_core.a3) の gz-transport I/O と配信のみ。

    A3は本質的に端末 (車両) ごとの分散制御であり、車間のBS競合は解決しない
    (競合はデータプレーンの BsOccupancyRegistry が物理的に拒否する)。
    プローブ窓 (数十ms) は再計画周期 (0.2s) より短いため、計画の状態が
    変化したレポートティックでは周期を待たず即時配信する。
    """

    def __init__(self, cfg: SchedulerConfig):
        self.cfg = cfg
        self.num_bs = len(cfg.bs_positions)
        self.ctrls = {v['name']: A3Controller(cfg.a3_params,
                                              len(v['antenna_offsets']), self.num_bs)
                      for v in cfg.vehicles}
        self.default_vid = cfg.vehicles[0]['name']

        self.lock = threading.Lock()
        self.inbox = []
        self.prev_t = -1.0
        self.next_plan_t = -1e18
        self.last_signature = None
        self.report_count = 0

        self.node = Node()
        self.pub = self.node.advertise(cfg.schedule_topic, msgs.HoSchedule)
        ok = self.node.subscribe(msgs.MeasurementReport, cfg.report_topic, self.on_report)
        print(f"[kkf_scheduler] control_plane=a3 subscribe={ok} "
              f"partition={os.environ.get('GZ_PARTITION', '(default)')} "
              f"bs={self.num_bs} vehicles={list(self.ctrls)}", flush=True)

    def on_report(self, msg):
        # メッセージ退避のみ (GILクラッシュ回避のため、処理・配信は spin スレッド側)
        with self.lock:
            self.inbox.append(msg)

    def _process_report(self, msg):
        t = msg.t_sim
        self.prev_t = max(self.prev_t, t)
        vid = msg.vehicle or self.default_vid
        ctrl = self.ctrls.get(vid)
        if ctrl is None:
            return
        entries = [(e.ant, e.bs, e.rssi_dbm) for e in msg.reports
                   if 0 <= e.bs < self.num_bs and 0 <= e.ant < ctrl.num_ant]
        ctrl.ingest(t, entries)
        self.report_count += 1

    def _publish(self, t):
        # 失効時はプラグインが現割当を維持 (フェイルセーフ) するため短めで良い
        sched = msgs.HoSchedule(t_issued=t, valid_until=t + 1.0)
        published_any = False
        for vid, ctrl in self.ctrls.items():
            plan = ctrl.plan(t)
            if not plan:
                continue
            for t_start, pair, is_measure in plan:
                sched.plan.add(t_start=t_start,
                               ant=pair // self.num_bs,
                               bs=pair % self.num_bs,
                               mode=msgs.MEASURE if is_measure else msgs.DATA,
                               vehicle=vid)
            published_any = True
        if published_any:
            self.pub.publish(sched)

    def spin(self):
        """取込・計画・配信ループ (メインスレッド)。状態変化時は即時、無変化でも周期配信。"""
        last_status_wall = time.monotonic()
        try:
            while True:
                time.sleep(0.01)
                with self.lock:
                    pending, self.inbox = self.inbox, []
                for msg in pending:
                    self._process_report(msg)

                if self.prev_t >= 0.0:
                    signature = tuple((c.serving, c.probe) for c in self.ctrls.values())
                    if signature != self.last_signature or self.prev_t >= self.next_plan_t:
                        self._publish(self.prev_t)
                        self.last_signature = signature
                        self.next_plan_t = self.prev_t + self.cfg.replan_period_s

                if time.monotonic() - last_status_wall >= 2.0:
                    last_status_wall = time.monotonic()
                    stat = " ".join(f"{vid}:{c.status()}" for vid, c in self.ctrls.items())
                    print(f"[kkf_scheduler] reports={self.report_count} t={self.prev_t:.2f} "
                          f"{stat}", flush=True)
        except KeyboardInterrupt:
            pass


def main():
    parser = argparse.ArgumentParser(description="Predictive handover control-plane scheduler")
    parser.add_argument('--config', required=True, help="sim_params.yaml のパス")
    # ros2 launch の Node アクションが付加する --ros-args 等は無視する
    args, _ = parser.parse_known_args()

    cfg = SchedulerConfig(args.config)
    if (not cfg.road_points or not cfg.bs_positions or not cfg.vehicles
            or not cfg.vehicles[0]['antenna_offsets']):
        print("[kkf_scheduler] ERROR: config に waypoints / spawn_entities / antennas が必要",
              file=sys.stderr)
        sys.exit(1)
    if cfg.control_plane == 'a3':
        A3Scheduler(cfg).spin()
    else:
        MpcScheduler(cfg).spin()


if __name__ == '__main__':
    main()
