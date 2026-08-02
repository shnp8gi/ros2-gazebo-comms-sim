#!/usr/bin/env python3
"""
kkf_scheduler_node
------------------
予測ハンドオーバーの制御プレーンノード (設計書 §6)。

責務: gz-transport の I/O と周期制御のみ。数理 (予測器・ビタビDP) は
comms_sim_pkg.kkf_core に全面委譲し、本ファイルは数式を持たない。

制御則は link_controller_node.control_plane で選択する (本番仕様 §7 の arm):
  - "kkf_mpc": KKF地図 (第1層、RBF基底 + σ_ν²(s) 自己組織化) — 提案手法。
                kkf_state_dir 指定で収束状態をロード (kkf_conv)、なしで kkf_cold
  - "ts_kf":   ペアごとのスカラーKF時系列外挿 — 先行研究ベースライン
                (tskf_trend_fallback: true で未観測対は決定論プロファイル代用)
  - "trend":   決定論プロファイル (距離減衰+両端指向性) のみ、実測不使用 — 下界
  - "oracle":  全対の現在真値 (observe_all_pairs + 雑音0 が前提) — 情報上界
  - "a3":      A3イベント型 (hysteresis+TTT、MEASUREスロットで近傍プローブ)
                — 予測なしの標準ベースライン (kkf_core.a3 参照)

L3 (車間割当) は kkf_assigner で選択する:
  - "priority_dp": 逐次優先度付きビタビDP (従来。kkf_priority: entry/deficit_time)
  - "hungarian":   リスク調整効用 μ−κσ のハンガリアン割当 (本番仕様 §6)。
                    切替抑制は現割当BSへの kkf_switch_bonus_db で行う

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
    RoadCoordinate, ConstantBasis, LogDistanceBasis, RbfBasis, KkfParams,
    Observation, KrigedKalmanFilter, solve_handover_plan, TrackerParams,
    BlockageTracker, ScalarKfParams, ScalarRssiKF, A3Params, A3Controller,
    VarianceMapParams, AleatoricVarianceMap, RemStore, basis_hash,
    solve_assignment, FORBIDDEN_UTILITY,
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
        # 車両ごとの進行方向 (+1 = +x / -1 = -x)。waypoints の x 変位の符号で決まる。
        # 双方向道路では、同じ位置でも進行方向によってアンテナの向きも RSU までの
        # 横距離も異なる (車線が方向で決まるため) ので、REM を方向別に分ける
        # 鍵になる (本節の direction-split)
        self.vehicle_dirs = {}
        for v in cfg.get('vehicles', []):
            wps = v.get('waypoints', [])
            d = 1
            if len(wps) >= 2:
                d = 1 if float(wps[-1][0]) >= float(wps[0][0]) else -1
            self.vehicle_dirs[v.get('name')] = d
        # 地図を分ける方向の集合。**環境の性質であって run の交通実現ではない**。
        # kkf_directions が明示されていればそれを使う (シナリオ生成器が道路の
        # 車線構成から書く)。無ければ車両から推定し、単一方向のシナリオ
        # (road_10car 等) では要素1つ = 従来と同一構成に縮退する。
        #
        # 明示指定が要る理由: 対象車を交通流から確率生成すると、run によっては
        # 片方向の対象車しか出ないことがある。推定に頼ると run ごとに地図の
        # キーが変わり、走行間で状態を読めず学習が一切蓄積しない
        # 車載アンテナ本数。**向きが違えば同じ位置でも利得が全く違う**ので、
        # REM は (RSU, 方向, アンテナ) ごとに分ける必要がある。位置 s だけで
        # 索引すると前向き/後ろ向きの予測値が同一になり、割当は常にアンテナ0を
        # 選び、学習も両者の観測を混ぜて地図を壊す (実測: 後ろ向きの grant が
        # 0.00s、繋がり得た 47.9s を全て捨てていた)
        ants = [len(v.get('antennas', [])) for v in cfg.get('vehicles', [])]
        self.num_antennas = max(ants) if ants else 1

        declared = link.get('kkf_directions')
        if declared:
            self.directions = sorted({1 if int(d) >= 0 else -1 for d in declared})
        else:
            self.directions = sorted(set(self.vehicle_dirs.values())) or [1]

        # 路線形状: x 軸方向の直線。弧長 s は常に +x 向きに増加するため、
        # 方向 -1 の車は v̂ < 0 として一貫して扱える。
        #
        # **座標系は環境の性質であって run の交通実現ではない**。kkf_road_x_range
        # が明示されていればそれを使う (生成器が RSU 列と評価半径から書く)。
        #
        # 明示指定が要る理由: 対象車を交通流から確率生成すると、車両の x 範囲が
        # run ごとに変わる。そこから道路を作ると (a) 同じ物理位置が run ごとに
        # 違う弧長になり学習した地図が無意味になる、(b) 基底の定義域が変わって
        # 状態が読めない、(c) 待機位置まで含めて 1.5km に伸び、20 基底では
        # 1 基底 75m となって接続窓 (~17m) を表現できない、という三重の破綻になる。
        # 未指定なら従来どおり車両から推定する (固定車両数のシナリオは不変)。
        rng_x = link.get('kkf_road_x_range')
        if rng_x and len(rng_x) == 2:
            self.road_points = [[float(rng_x[0]), 0.0, 0.0],
                                [float(rng_x[1]), 0.0, 0.0]]
        else:
            xs, ys, zs = [], [], []
            for v in cfg.get('vehicles', []):
                for wp in v.get('waypoints', []):
                    xs.append(float(wp[0])); ys.append(float(wp[1])); zs.append(float(wp[2]))
            if xs:
                y0 = sum(ys) / len(ys)
                z0 = sum(zs) / len(zs)
                self.road_points = [[min(xs), y0, z0], [max(xs), y0, z0]]
            else:
                self.road_points = []

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
        # kkf arm が事前地図ハイブリッドを使うか。本番仕様 §7 では kkf_cold/conv
        # は「まっさら→自己組織化」のため false (プロファイルは trend/kf 用に
        # 出力だけする)。既定 true = 従来の kkf_full (=kkf_prior arm) 互換
        self.kkf_use_prior = bool(link.get('kkf_use_prior', True))

        # --- L3 割当方式 (本番仕様 §6) ---
        self.assigner = link.get('kkf_assigner', 'priority_dp')
        self.switch_bonus_db = float(link.get('kkf_switch_bonus_db', 3.0))
        # 先読み (hungarian のみ): ペア (車 i, RSU j) の効用を、その組を保持した
        # ままホライズン上を進んだ場合の LCB の重み付き平均とする。
        #   U[i][j] = Σ_k γ^k · LCB(i, j, s_i + v̂_i·kΔ, t + kΔ) / Σ_k γ^k
        # K=1 (既定) は「今この瞬間の最適」= 先読みなし。K>1 で「窓を抜けつつある
        # ペアより、これから窓に入るペアを選ぶ」挙動になる。
        # KKF の価値のうち「先読み」の寄与を空間補間・リスクと分離して測るための
        # つまみであり、K=1 と K>1 の 2 arm を同一コードで構成できる
        self.lookahead_stages = max(1, int(link.get('kkf_lookahead_stages', 1)))
        self.lookahead_discount = float(link.get('kkf_lookahead_discount', 1.0))
        # レポートがこの時間途絶えた車は計画対象から外す [s]。連続交通流では
        # コリドーを出た車が延々と計画対象に残り、計算量と割当の両方を汚す
        # (退出は「レポートが来なくなる」ことでしか観測できない)。
        # 0 以下 = 無効 = 従来動作 (固定車両数のシナリオ向け)
        self.stale_report_s = float(link.get('kkf_stale_report_s', 1.0))

        # --- 第1層の基底 (本番仕様 §5.1) ---
        # "auto" = 従来動作 (prior あり→Constant / なし→LogDistance)、"rbf" = RBF基底
        self.basis_type = link.get('kkf_basis', 'auto')
        self.rbf_num_bases = int(link.get('kkf_rbf_num_bases', 20))
        # 平均場の事前値 [dBm]。地図は「この値からのずれ」を学習する。
        # 0 のままだと未観測の位置で μ=0 dBm (= 極めて強い信号) と評価され、
        # 悲観的であるべき LCB が逆に楽観的になる (実測: 道路の約12%で
        # データが無いのに接続閾値超え)。既定 0.0 は従来互換
        self.mean_prior_dbm = float(link.get('kkf_mean_prior_dbm', 0.0))
        self.rbf_width_m = float(link.get('kkf_rbf_width_m', 0.0))
        self.rbf_s_min = float(link.get('kkf_rbf_s_min', 0.0))
        self.rbf_s_max = float(link.get('kkf_rbf_s_max', 0.0))  # 0 = 道路全長

        # --- σ_ν²(s) 自己組織化マップ (本番仕様 §5.2) ---
        self.varmap_enabled = bool(link.get('kkf_varmap_enabled', False))
        self.varmap_grid_m = float(link.get('kkf_varmap_grid_m', 2.0))
        self.varmap_prior_var_db2 = float(link.get('kkf_varmap_prior_var_db2', 0.0))

        # --- 走行間永続化 (本番仕様 §5.3) ---
        # 制御プレーンをシム時刻のエポックに同期させる (実時間非依存)。
        # 既定 false は従来動作 (実時間ループ)
        self.lockstep = bool(link.get('kkf_lockstep', False))
        self.state_dir = str(link.get('kkf_state_dir', '') or '')
        self.state_save = bool(link.get('kkf_state_save', False))
        self.q_forget = float(link.get('kkf_q_forget', 0.05))
        self.state_gamma = float(link.get('kkf_state_gamma', 0.997))
        self.state_save_period_s = float(link.get('kkf_state_save_period_s', 2.0))
        self.kkf_freeze = bool(link.get('kkf_freeze', False))

        # --- arm: ts_kf の未観測対フォールバック / oracle (本番仕様 §7) ---
        self.tskf_trend_fallback = bool(link.get('tskf_trend_fallback', False))
        self.oracle_staleness_s = float(link.get('oracle_staleness_s', 1.0))
        self.observe_all_pairs = bool(report_cfg.get('observe_all_pairs', False))
        self.report_noise_std = float(report_cfg.get('noise_std_db', 2.0))

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
    """提案手法: RSUごとのKKF地図 (第1層) + σ_ν²(s) 自己組織化マップ。

    地図は弧長座標上の場であり車両に依存しない = 複数車両で自然に共有される
    (先行車の測定が後続車の予測を改善する)。

    基底 (kkf_basis):
      - "auto": 従来動作。事前地図 (PriorProfile) があれば ConstantBasis の
        偏差学習、なければ LogDistanceBasis の直接学習
      - "rbf":  RBF基底で平均場 (指向性ウィンドウ込み) を直接学習する。
        まっさら開始→自己組織化 (kkf_cold/conv、本番仕様 §5.1) の構成

    σ_ν²(s) (kkf_varmap_enabled): 更新後残差の二乗を割引累積し、予測分散に
    のみ加算する遮蔽リスクマップ (Kalman ゲインには使わない、本番仕様 §5.2)。

    永続化 (kkf_state_dir / kkf_state_save): (α, P, S2, W) を RSU ごとに
    保存・復元し、忘却は load 時に適用する (本番仕様 §5.3)。

    方向別地図 (direction-split): 双方向道路では、同じ弧長 s でも進行方向で
    アンテナの向きと RSU までの横距離が変わる (方向が車線を決めるため) ので、
    地図を (RSU, 方向) ごとに持つ。単一方向のシナリオでは方向集合の要素が
    1 つになり、従来と同一の「RSU ごとに 1 地図」構成へ自動的に縮退する
    (road_10car の学習済み状態もそのまま読める)。
    """

    def __init__(self, cfg: SchedulerConfig, road: RoadCoordinate, priors=None):
        self.cfg = cfg
        self.priors = priors
        self.frozen = cfg.kkf_freeze
        self.dirs = list(cfg.directions)
        self.split = len(self.dirs) > 1     # 方向別に分けるか

        s_max = cfg.rbf_s_max if cfg.rbf_s_max > cfg.rbf_s_min else road.total_length()
        self.n_ant = max(1, int(getattr(cfg, 'num_antennas', 1)))
        self.keys = [(b, d, a) for b in range(len(cfg.bs_positions))
                     for d in self.dirs for a in range(self.n_ant)]

        def _make_basis(b):
            if cfg.basis_type == 'rbf':
                return RbfBasis(cfg.rbf_s_min, s_max, cfg.rbf_num_bases, cfg.rbf_width_m)
            if priors:
                return ConstantBasis()
            return LogDistanceBasis(road, cfg.bs_positions[b])

        self.bases = {k: _make_basis(k[0]) for k in self.keys}
        self.maps = {k: KrigedKalmanFilter(self.bases[k], cfg.kkf_params)
                     for k in self.keys}

        # σ_ν²(s): 位置依存の偶然的分散 (遮蔽リスクマップ)
        self.varmaps = None
        if cfg.varmap_enabled:
            vm_params = VarianceMapParams(
                s_min=cfg.rbf_s_min, s_max=s_max, grid_m=cfg.varmap_grid_m,
                prior_var_db2=cfg.varmap_prior_var_db2)
            self.varmaps = {k: AleatoricVarianceMap(vm_params) for k in self.keys}

        # 第2層: RSUごとに独立の遮蔽トラッカー (そのRSUの残差マップの影を追跡)。
        # 本番構成では無効 (σ_ν²マップが代替) だが比較用に残す
        self.trackers = ({k: BlockageTracker(cfg.tracker_params) for k in self.keys}
                         if cfg.tracker_enabled else None)

        # 走行間永続化: load はここ (忘却込み)、save は save_state()
        self.store = RemStore(cfg.state_dir) if cfg.state_dir else None
        self.run_count = 0
        if self.store:
            loaded = 0
            for k in self.keys:
                st = self.store.load(self._state_key(k),
                                     expected_basis_hash=basis_hash(self.bases[k].config()),
                                     q_forget=cfg.q_forget, gamma=cfg.state_gamma)
                if st is None:
                    continue
                self.maps[k].alpha = st['alpha']
                self.maps[k].P = st['P']
                if self.varmaps:
                    self.varmaps[k].load_state(st['S2'], st['W'])
                self.run_count = max(self.run_count, int(st['meta'].get('run_count', 0)))
                loaded += 1
            print(f"[kkf_scheduler] REM state loaded: {loaded}/{len(self.maps)} maps "
                  f"(run_count={self.run_count}, dir={cfg.state_dir})", flush=True)

    def _state_key(self, key):
        """永続化ファイル名のキー。単一方向・単一アンテナなら従来と同じ '<bs>'。"""
        b, d, a = key
        name = f"{b}" if not self.split else f"{b}_{'p' if d > 0 else 'm'}"
        return name if self.n_ant <= 1 else f"{name}_a{a}"

    def _key(self, vid, bs, ant=0):
        """(車, RSU, アンテナ) → 地図キー。

        アンテナを含めるのが要点。前向き/後ろ向きは同じ位置 s にあるので、
        位置だけで索引すると予測が区別できず、割当が常に同じアンテナを選ぶ。
        """
        d = self.cfg.vehicle_dirs.get(vid, self.dirs[0]) if self.split else self.dirs[0]
        return (bs, d, min(int(ant), self.n_ant - 1))

    def save_state(self, t):
        """定期スナップショット (アトミック上書き)。学習相の run 間受け渡しに使う。"""
        if not self.store:
            return
        for k in self.keys:
            kkf = self.maps[k]
            if self.varmaps:
                s2, w = self.varmaps[k].S2, self.varmaps[k].W
            else:
                s2, w = np.zeros(1), np.zeros(1)
            self.store.save(self._state_key(k), kkf.alpha, kkf.P, s2, w, {
                'basis_hash': basis_hash(kkf.basis.config()),
                'bs_id': k[0],
                'direction': k[1],
                'antenna': k[2],
                'run_count': self.run_count + 1,
                'last_t': float(t),
            })

    def _prior_mean(self, vid, ant, bs, s):
        """平均場の事前値 = 定数事前値 + (あれば) 理論プロファイル。"""
        mu0 = self.cfg.mean_prior_dbm
        if not self.priors or vid not in self.priors:
            return mu0
        m = self.priors[vid].mean(ant, bs, s)
        return mu0 if m is None else mu0 + m

    def _prior_mean_batch(self, vid, ant, bs, s_arr):
        mu0 = self.cfg.mean_prior_dbm
        if not self.priors or vid not in self.priors:
            return np.full(len(s_arr), mu0)
        return mu0 + self.priors[vid].mean_batch(ant, bs, s_arr)

    def ingest(self, vid, t, antenna_s, entries):
        """entries: [(ant, bs, rssi_dbm)] を地図更新・トラッカー観測に振り分ける。"""
        if self.frozen:
            return  # kkf_freeze: 評価用に状態を凍結 (予測のみ)
        per_key = {}
        for ant, bs, rssi in entries:
            s_obs = antenna_s[ant]
            # 地図は事前値からの偏差のみ学習する
            z = rssi - self._prior_mean(vid, ant, bs, s_obs)
            key = self._key(vid, bs, ant)
            # 式(10): 予測遮蔽帯内の観測は先回りで観測雑音を減格 (σ²_NLOS)。
            # 本番構成 (トラッカー無効) では固定 R_meas = σ_ν² を観測ノイズに
            # 流用しない (平均場学習を殺さない、本番仕様 §5.2)
            noise_var = self.cfg.meas_noise_var
            if self.trackers and self.trackers[key].is_blocked(s_obs, t):
                noise_var = self.cfg.nlos_noise_var
            per_key.setdefault(key, []).append(
                Observation(s=s_obs, t=t, z=z, noise_var=noise_var))
        for key, obs in per_key.items():
            fresh_residuals = self.maps[key].update(t, obs)
            if self.varmaps:
                self.varmaps[key].update_batch(fresh_residuals)
            if self.trackers:
                self.trackers[key].ingest(t, fresh_residuals)

    def lcb(self, vid, ant, bs, s_future, t_future, kappa):
        key = self._key(vid, bs, ant)
        mean, var = self.maps[key].predict_at(s_future, t_future)
        mean += self._prior_mean(vid, ant, bs, s_future)
        # σ_ν²(s) は予測分散にのみ加算 (本番仕様 §5.2)
        if self.varmaps:
            var += self.varmaps[key].query(s_future)
        value = mean - kappa * math.sqrt(var)
        # 式(11) 第3項: 遮蔽帯の通過が予測される区間・時刻にはペナルティを加算
        if self.trackers and self.trackers[key].is_blocked(s_future, t_future):
            value -= self.cfg.blockage_penalty_db
        return value

    def lcb_batch(self, vid, ant, bs, s_arr, t_arr, kappa):
        """ステージ一括の LCB (再計画の計算量ボトルネック解消用)。"""
        key = self._key(vid, bs, ant)
        means, variances = self.maps[key].predict_batch(s_arr, t_arr)
        means = means + self._prior_mean_batch(vid, ant, bs, s_arr)
        if self.varmaps:
            variances = variances + self.varmaps[key].query_batch(s_arr)
        values = means - kappa * np.sqrt(variances)
        if self.trackers:
            trk = self.trackers[key]
            for i, (s, t) in enumerate(zip(s_arr, t_arr)):
                if trk.is_blocked(s, t):
                    values[i] -= self.cfg.blockage_penalty_db
        return values

    def lcb_multi(self, queries, bs, kappa):
        """複数 (車, アンテナ) × 複数ステージの (LCB, 平均) を一括評価する。

        queries: [(vid, ant, s_arr, t_arr)]。
        戻り値: 同順の [(lcb(ステージ数), mean(ステージ数))]。

        **LCB と平均を分けて返す理由**: 割当の優先順位付けにはリスク調整効用
        (LCB = μ − κσ) が正しいが、「そもそも繋がるか」の判定に LCB を使うと
        σ_ν² が大きい場所で過度に保守的になり、繋がる機会を見送ってしまう
        (実測: 競合下で配信ゼロの車が 18% 発生した)。接続可能性は平均 μ で
        判定し、σ は「どの RSU を誰に渡すか」の順位付けにのみ使う。

        再計画は「全車 × 全 RSU × 全ステージ」の評価が要り、1 点ずつ predict_at を
        呼ぶと 64×64 のクリギング解を毎回解き直して律速する (実測 13.7 倍差)。
        同一地図に属するクエリ点をまとめて predict_batch に渡すことで、
        解を 1 回に償却する。数値は逐次呼び出しと同一 (差 ~1e-15 dB)。
        """
        groups = {}
        for i, (vid, ant, s_arr, t_arr) in enumerate(queries):
            groups.setdefault(self._key(vid, bs, ant), []).append(i)

        out = [None] * len(queries)
        for key, idxs in groups.items():
            lens = [len(queries[i][2]) for i in idxs]
            s_cat = np.concatenate([np.asarray(queries[i][2], dtype=float) for i in idxs])
            t_cat = np.concatenate([np.asarray(queries[i][3], dtype=float) for i in idxs])
            means, variances = self.maps[key].predict_batch(s_cat, t_cat)
            if self.varmaps:
                variances = variances + self.varmaps[key].query_batch(s_cat)
            values = means - kappa * np.sqrt(variances)
            if self.trackers:
                trk = self.trackers[key]
                for j, (s, t) in enumerate(zip(s_cat, t_cat)):
                    if trk.is_blocked(s, t):
                        values[j] -= self.cfg.blockage_penalty_db
            off = 0
            for i, n in zip(idxs, lens):
                add = self._prior_mean_batch(
                    queries[i][0], queries[i][1], bs,
                    np.asarray(queries[i][2], dtype=float))
                v = values[off:off + n] + add
                mu = means[off:off + n] + add
                out[i] = (v, mu)
                off += n
        return out

    def status(self):
        parts = [f"prior={'on' if self.priors else 'off'}"]
        if self.split:
            parts.append(f"dirs={self.dirs}")
        if self.trackers:
            n_tracks = sum(sum(1 for tr in trk.tracks if tr.is_confirmed())
                           for trk in self.trackers.values())
            parts.append(f"tracks={n_tracks}")
        if self.varmaps:
            contrasts = ",".join(f"{self.varmaps[k].contrast():.1f}" for k in self.keys)
            parts.append(f"σν²contrast=[{contrasts}]")
        if self.store:
            parts.append(f"run_count={self.run_count}"
                         + (" frozen" if self.frozen else ""))
        return " ".join(parts)

    def track_detail(self, t):
        """検証用: 確定トラックの (BS index, 中心弧長s, 速度) を返す。"""
        out = []
        if self.trackers:
            for (b, _d, _a), trk in self.trackers.items():
                for tr in trk.tracks:
                    if tr.is_confirmed():
                        out.append((b, tr.predict_center(t), tr.velocity))
        return out


class _LcbMultiMixin:
    """lcb_batch しか持たない予測器に一括インターフェースを与える既定実装。

    KkfMapPredictor は地図単位でまとめ直す専用実装 (13.7 倍高速) を持つが、
    他の arm は 1 クエリが辞書引き・スカラー KF 程度なので逐次で十分。
    """

    def lcb_multi(self, queries, bs, kappa):
        """(lcb, mean) を返す。σ を持たない予測器では両者が一致する。"""
        out = []
        for vid, ant, s_arr, t_arr in queries:
            v = self.lcb_batch(vid, ant, bs, s_arr, t_arr, kappa)
            mu = self.lcb_batch(vid, ant, bs, s_arr, t_arr, 0.0)
            out.append((v, mu))
        return out


class TimeSeriesKfPredictor(_LcbMultiMixin):
    """先行研究ベースライン: (車両, アンテナ, BS) ごとのRSSI時系列スカラーKF外挿。

    空間構造を持たないため、前方の未観測区間の遮蔽・利得変化は原理的に
    予測できない (s_future は受け取るが使わない)。車間の情報共有もない。

    tskf_trend_fallback: true (本番仕様 §7 の kf arm) では未観測対の予測を
    無情報事前 (prior_mean_dbm) ではなく決定論プロファイル (trend) で代用する
    — 情報制約を「trend + 実測時系列」に揃えるため。
    """

    def __init__(self, cfg: SchedulerConfig, num_bs, priors=None):
        self.cfg = cfg
        self.num_bs = num_bs
        self.filters = {}  # (vid, ant, bs) -> ScalarRssiKF
        self.priors = priors if cfg.tskf_trend_fallback else None

    def _filter(self, vid, ant, bs):
        key = (vid, ant, bs)
        if key not in self.filters:
            self.filters[key] = ScalarRssiKF(self.cfg.scalar_kf_params)
        return self.filters[key]

    def _trend_mean(self, vid, ant, bs, s):
        if not self.priors or vid not in self.priors:
            return None
        return self.priors[vid].mean(ant, bs, s)

    def ingest(self, vid, t, antenna_s, entries):
        for ant, bs, rssi in entries:
            self._filter(vid, ant, bs).update(t, rssi, self.cfg.meas_noise_var)

    def lcb(self, vid, ant, bs, s_future, t_future, kappa):
        f = self._filter(vid, ant, bs)
        if not f.initialized():
            trend = self._trend_mean(vid, ant, bs, s_future)
            if trend is not None:
                return trend - kappa * math.sqrt(self.cfg.scalar_kf_params.prior_var)
        mean, var = f.predict_at(t_future)
        return mean - kappa * math.sqrt(var)

    def lcb_batch(self, vid, ant, bs, s_arr, t_arr, kappa):
        f = self._filter(vid, ant, bs)
        if not f.initialized() and self.priors and vid in self.priors:
            means = self.priors[vid].mean_batch(ant, bs, s_arr)
            return means - kappa * math.sqrt(self.cfg.scalar_kf_params.prior_var)
        out = np.empty(len(t_arr))
        for i, t in enumerate(t_arr):
            mean, var = f.predict_at(t)
            out[i] = mean - kappa * math.sqrt(var)
        return out

    def status(self):
        n_init = sum(1 for f in self.filters.values() if f.initialized())
        return (f"kf_init={n_init}/{len(self.filters)}"
                f" fallback={'trend' if self.priors else 'prior'}")


class TrendPredictor(_LcbMultiMixin):
    """arm: trend (下界)。決定論成分 (距離減衰 + 両端指向性) の完全知識のみ。

    実測レポートを一切使わない (ingest は破棄)。σ=0 のため κ は効かない。
    シャドウ・動的遮蔽を知らないことが情報制約 (本番仕様 §7)。
    export_rssi_profile: true が前提 (決定論プロファイルが情報源)。
    """

    MISSING_DB = -150.0  # プロファイルに無いペア (割当対象外相当)

    def __init__(self, cfg: SchedulerConfig, priors):
        if not priors:
            raise ValueError(
                "control_plane: trend には export_rssi_profile: true が必要")
        self.priors = priors

    def ingest(self, vid, t, antenna_s, entries):
        pass  # 実測は使わない (情報制約)

    def lcb(self, vid, ant, bs, s_future, t_future, kappa):
        prof = self.priors.get(vid)
        m = prof.mean(ant, bs, s_future) if prof else None
        return self.MISSING_DB if m is None else m

    def lcb_batch(self, vid, ant, bs, s_arr, t_arr, kappa):
        prof = self.priors.get(vid)
        if prof is None or (ant, bs) not in prof.table:
            return np.full(len(s_arr), self.MISSING_DB)
        return prof.mean_batch(ant, bs, s_arr)

    def status(self):
        return "trend (deterministic profile only)"


class OraclePredictor(_LcbMultiMixin):
    """arm: oracle (情報上界)。全対の現在真値を保持し、予測せず瞬時値で選ぶ。

    observe_all_pairs: true + noise_std_db: 0 のレポート設定が前提
    (満たさない場合は起動時に警告)。T_est ≪ スロットのため瞬時貪欲で
    ほぼ上界になる (本番仕様 §7)。
    """

    MISSING_DB = -150.0

    def __init__(self, cfg: SchedulerConfig):
        self.staleness_s = cfg.oracle_staleness_s
        self.latest = {}  # (vid, ant, bs) -> (t, rssi)
        if not cfg.observe_all_pairs or cfg.report_noise_std > 0.0:
            print("[kkf_scheduler] WARN: oracle には observe_all_pairs: true と "
                  f"noise_std_db: 0 が必要 (現在 all_pairs={cfg.observe_all_pairs}, "
                  f"noise={cfg.report_noise_std}) — 情報上界になっていない",
                  flush=True)

    def ingest(self, vid, t, antenna_s, entries):
        for ant, bs, rssi in entries:
            self.latest[(vid, ant, bs)] = (t, rssi)

    def lcb(self, vid, ant, bs, s_future, t_future, kappa):
        rec = self.latest.get((vid, ant, bs))
        if rec is None or (t_future - rec[0]) > self.staleness_s:
            return self.MISSING_DB
        return rec[1]

    def lcb_batch(self, vid, ant, bs, s_arr, t_arr, kappa):
        return np.array([self.lcb(vid, ant, bs, 0.0, t, kappa) for t in t_arr])

    def status(self):
        return f"oracle pairs={len(self.latest)}"


def make_predictor(cfg: SchedulerConfig, road: RoadCoordinate):
    if cfg.control_plane == 'kkf_mpc':
        priors = load_prior_profiles(cfg, road) if cfg.kkf_use_prior else None
        return KkfMapPredictor(cfg, road, priors)
    if cfg.control_plane == 'ts_kf':
        priors = (load_prior_profiles(cfg, road)
                  if cfg.tskf_trend_fallback else None)
        return TimeSeriesKfPredictor(cfg, len(cfg.bs_positions), priors)
    if cfg.control_plane == 'trend':
        return TrendPredictor(cfg, load_prior_profiles(cfg, road))
    if cfg.control_plane == 'oracle':
        return OraclePredictor(cfg)
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

        # hungarian 割当の現ペア (vid -> (ant, bs) | None)。切替ヒステリシス
        # (kkf_switch_bonus_db) の参照元
        self.current_pair = {v['name']: None for v in cfg.vehicles}
        self.last_save_t = -1e18

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

    # --- 再計画 (第3層) : kkf_assigner で割当方式を選ぶ ---
    def _replan(self, t):
        if self.cfg.assigner == 'hungarian':
            self._replan_hungarian(t)
        else:
            self._replan_priority_dp(t)

    def _replan_hungarian(self, t):
        """リスク調整効用 μ−κσ のハンガリアン割当 (本番仕様 §6)。

        効用はペアを保持したままホライズン上を進んだ場合の LCB の重み付き平均:
            U[i][j] = Σ_k γ^k · LCB(i, j, s_i + v̂_i·kΔ, t + kΔ) / Σ_k γ^k
        kkf_lookahead_stages K=1 (既定) なら「今この瞬間の最適」= 先読みなしで、
        K>1 なら「窓を抜けつつあるペアより、これから窓に入るペアを選ぶ」。
        予測の価値を空間補間・リスク・先読みに分解するためのつまみ。

        ping-pong 抑制は現割当 BS への switch_bonus_db (ヒステリシス)。
        余剰の車は自然に idle (min_utility = kkf_idle_lcb_db)。

        LCB は地図ごとに一括評価する (lcb_multi)。1 点ずつ predict_at を呼ぶと
        クリギング解を毎回解き直して律速するため (実測 13.7 倍差)。
        """
        wall0 = time.monotonic()
        cfg = self.cfg
        active = []   # (vid, vs, ant_s)
        for v in cfg.vehicles:
            vs = self.vstates[v['name']]
            ant_s = vs.antenna_s_at(t)
            if ant_s is None:
                continue                      # 未進入 (レポート未受信)
            if (cfg.stale_report_s > 0.0
                    and t - vs.last_report_t > cfg.stale_report_s):
                continue                      # 退出済み (レポート途絶)
            active.append((v['name'], vs, ant_s))
        if not active:
            return

        n_veh, n_bs = len(active), self.num_bs
        K = cfg.lookahead_stages
        w = cfg.lookahead_discount ** np.arange(K)
        w = w / w.sum()
        dt_grid = np.arange(K) * cfg.plan_dt_s
        t_grid = t + dt_grid

        # (車, アンテナ) ごとのホライズン上の位置・時刻。等速外挿は方向の符号を
        # v̂ が持つため、上り・下り車の双方でそのまま正しい
        queries = []      # [(vid, ant, s_arr, t_arr)]
        q_index = []      # queries[q] が (車 i, アンテナ a) であることの対応
        for i, (vid, vs, ant_s) in enumerate(active):
            for a, s0 in enumerate(ant_s):
                queries.append((vid, a, s0 + vs.v_hat * dt_grid, t_grid))
                q_index.append((i, a))

        # utility = 割当の優先順位 (リスク調整効用 LCB)、
        # feasible_mu = 接続可能性の判定に使う平均 μ。両者を分けるのが要点で、
        # LCB で門番をするとリスクの高い場所で繋がる機会を見送る (§lcb_multi)
        utility = np.full((n_veh, n_bs), -np.inf)
        feasible_mu = np.full((n_veh, n_bs), -np.inf)
        best_ant = np.zeros((n_veh, n_bs), dtype=int)
        for b in range(n_bs):
            vals = self.predictor.lcb_multi(queries, b, cfg.kappa)
            for q, (i, a) in enumerate(q_index):
                lcb_q, mu_q = vals[q]
                u = float(np.dot(w, lcb_q))       # ホライズン重み付き平均
                if u > utility[i, b]:             # 車の各アンテナのうち最良を代表に
                    utility[i, b] = u
                    feasible_mu[i, b] = float(np.dot(w, mu_q))
                    best_ant[i, b] = a
        for i, (vid, _vs, _ant_s) in enumerate(active):
            cur = self.current_pair.get(vid)
            if cur is not None and cur[1] < n_bs:
                utility[i, cur[1]] += cfg.switch_bonus_db

        # 接続見込みのないペア (平均 μ が接続閾値未満) は割当対象から外す。
        # 掴んでも 1 バイトも送れず、需要超過では他車を締め出すだけなので。
        # 判定は μ で行い、σ は上の utility (順位付け) にのみ効かせる
        utility = np.where(feasible_mu >= cfg.idle_lcb_db, utility, FORBIDDEN_UTILITY)
        pairs, _ = solve_assignment(utility)

        sched = msgs.HoSchedule(
            t_issued=t, valid_until=t + max(1.0, 2.5 * cfg.replan_period_s))
        for i, (vid, vs, ant_s) in enumerate(active):
            if i in pairs:
                b = pairs[i]
                a = int(best_ant[i, b])
                self.current_pair[vid] = (a, b)
                vs.current_state = a * n_bs + b
                self.granted_time[vid] += cfg.replan_period_s
                sched.plan.add(t_start=t, ant=a, bs=b, mode=msgs.DATA, vehicle=vid)
            else:
                self.current_pair[vid] = None
                vs.current_state = len(ant_s) * n_bs  # idle
                sched.plan.add(t_start=t, ant=0, bs=-1, mode=msgs.DATA, vehicle=vid)
        self.pub.publish(sched)

        if os.environ.get('KKF_DEBUG'):
            picked = {active[i][0]: (int(best_ant[i, b]), int(b))
                      for i, b in pairs.items()}
            print(f"[kkf_debug] t={t:.2f} hungarian pairs={picked} "
                  f"u_max={utility.max():.1f} "
                  f"wall={1e3 * (time.monotonic() - wall0):.0f}ms", flush=True)

    def _replan_priority_dp(self, t):
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

    def _maybe_save_state(self, force=False):
        """走行間永続化 (kkf_state_save)。定期 + 終了時のベストエフォート保存。

        アトミック上書きのため、ノードが SIGKILL されても直近スナップショット
        (既定 2s sim 間隔) までは残る (gz teardown segfault に保存を依存させない)。
        """
        cfg = self.cfg
        if not (cfg.state_save and cfg.state_dir
                and hasattr(self.predictor, 'save_state')):
            return
        if force or (self.last_t - self.last_save_t >= cfg.state_save_period_s):
            if self.last_t >= 0.0:
                self.predictor.save_state(self.last_t)
                self.last_save_t = self.last_t

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
                    self._maybe_save_state()

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
        finally:
            try:
                self._maybe_save_state(force=True)
            except Exception as e:
                print(f"[kkf_scheduler] state save on exit failed: {e}", flush=True)


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
        self.backlog = []          # 取り込み待ち (lockstep ではエポック境界で切る)
        self.prev_t = -1.0
        # lockstep ではエポックを 0 から刻む。従来は「最初の観測時刻+周期」
        self.next_plan_t = 0.0 if cfg.lockstep else -1e18
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
        # lockstep ではプラグインが毎エポック応答を待つので、言うことが無くても
        # 必ず配信する (受信側は空メッセージを合図として扱い割当に触れない)
        if published_any or self.cfg.lockstep:
            self.pub.publish(sched)

    def spin(self):
        """取込・計画・配信ループ (メインスレッド)。

        lockstep=false (従来): 実時間ループ。状態変化時は即時、無変化でも周期配信。
          プラグインは届いた時点のステップで適用するため、**同じ入力でも計算機の
          混み具合で結果が変わる**。実測で、KKF ノードを4つ同時に走らせると
          grant 時間が 4.6% 落ち、手法間の差 (5-14%) と同程度の交絡になった。

        lockstep=true: シム時刻のエポック t_k = k·T に同期する。
          エポック k の観測が出揃った時点 (= t_k より後の観測が届いた時点) で
          エポック k 用の計画を作り、t_issued=t_k を付けて配信する。プラグインは
          t_{k+1} で「t_issued >= t_k の計画」が来るまでシム時刻を止めて待つ。
          実時間がいくらかかっても、計画が効き始めるシム時刻は必ず同じになる。
        """
        last_status_wall = time.monotonic()
        try:
            while True:
                # lockstep ではシムがこの応答を待って止まっている。
                # ポーリング間隔がそのまま実時間コストになる
                time.sleep(0.0005 if self.cfg.lockstep else 0.01)
                with self.lock:
                    pending, self.inbox = self.inbox, []
                # 到着順は配送タイミング次第で変わるので、シム時刻と車両名で
                # 決定的に並べ替えてから取り込む (取り込み順が推定に効くため)
                self.backlog.extend(pending)
                self.backlog.sort(key=lambda m: (m.t_sim, m.vehicle))

                if not self.cfg.lockstep:
                    for msg in self.backlog:
                        self._process_report(msg)
                    self.backlog = []
                    if self.prev_t >= 0.0:
                        signature = tuple((c.serving, c.probe) for c in self.ctrls.values())
                        if signature != self.last_signature or self.prev_t >= self.next_plan_t:
                            self._publish(self.prev_t)
                            self.last_signature = signature
                            self.next_plan_t = self.prev_t + self.cfg.replan_period_s
                else:
                    # エポック k の締切は「t_k より後の観測が届いたこと」で判定する。
                    # 全プラグインは同じシムステップで観測を出すので、t_k を超える
                    # 観測が1つでも来れば t_k 以前の観測は出揃っている
                    while self.backlog and self.backlog[-1].t_sim > self.next_plan_t:
                        epoch = self.next_plan_t
                        keep = []
                        for msg in self.backlog:
                            if msg.t_sim <= epoch:
                                self._process_report(msg)
                            else:
                                keep.append(msg)
                        self.backlog = keep
                        self._publish(epoch)          # 必ず配信 (無変化でも待たせない)
                        self.next_plan_t = epoch + self.cfg.replan_period_s

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
