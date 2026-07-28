#!/usr/bin/env python3
"""
kkf_scheduler_node 統合テスト (本番仕様 §5〜§7):
  - SchedulerConfig の新パラメータ読取
  - kkf_mpc (RBF基底 + σ_ν²マップ + 永続化) の学習→保存→復元→忘却
  - trend / oracle / ts_kf(trendフォールバック) の情報制約どおりの挙動
  - hungarian 割当のBS排他・idle・ヒステリシス (gz I/O 込みの _replan)
コンテナ内で実行:
  docker compose exec -T sim bash -c \
    "source /workspace/install/setup.bash && python3 /workspace/tools/tests/scheduler_arms_test.py"
"""
import math
import os
import shutil
import sys
import tempfile

import numpy as np
import yaml

sys.path.insert(0, '/workspace/src/comms_sim_pkg/comms_sim_pkg')
import kkf_scheduler_node as node  # noqa: E402


def make_config(tmpdir, noise_std=2.0, **link_overrides):
    """3BS×3車の最小 sim_params を生成して SchedulerConfig を返す。"""
    link = {
        'control_plane': 'kkf_mpc',
        'kkf_use_prior': False,
        'kkf_basis': 'rbf',
        'kkf_rbf_num_bases': 20,
        'kkf_varmap_enabled': True,
        'kkf_varmap_grid_m': 2.0,
        'kkf_assigner': 'hungarian',
        'kkf_switch_bonus_db': 3.0,
        'kkf_kappa': 1.0,
        'kkf_idle_lcb_db': -110.0,
        'kkf_tracker_enabled': False,
        'kkf_replan_period_s': 0.2,
    }
    link.update(link_overrides)
    cfg_dict = {
        'link_controller_node': {'ros__parameters': link},
        'comms_simulator_node': {'ros__parameters': {
            'measurement_report': {'noise_std_db': noise_std,
                                   'observe_all_pairs': True},
        }},
        'vehicles': [
            {'name': f'car_{k}', 'pose': [-150.0 - 18.0 * k, 0, 0, 0, 0, 0],
             'waypoints': [[-150.0 - 18.0 * k, 0.0, 0.0, 16.67],
                           [150.0, 0.0, 0.0, 16.67]],
             'antennas': [{'name': f'car_{k}_ant', 'offset': [0.0, 0.0, 1.35],
                           'relative_rpy': [0.0, 0.0, 0.26]}]}
            for k in range(1, 4)
        ],
        'spawn_entities': {
            f'antenna_{i}': {'pose': [-60.0 + 60.0 * i, 8.0, 0.0, 0.0, 0.0, -1.309],
                             'antenna_offset': [0.0, 0.0, 2.5],
                             'antenna_relative_rpy': [0.0, 0.0, 0.0]}
            for i in range(3)
        },
        'simulation': {'output_subdir': ''},
    }
    path = os.path.join(tmpdir, 'sim_params.yaml')
    with open(path, 'w') as f:
        yaml.dump(cfg_dict, f)
    return node.SchedulerConfig(path)


def true_field(s, bs_x):
    """指向性ウィンドウ風の合成場 (ピークは BS の 10m 上流)。"""
    return -80.0 + 20.0 * math.exp(-0.5 * ((s - (bs_x + 140.0)) / 10.0) ** 2)


def main():
    failures = []
    tmpdir = tempfile.mkdtemp(prefix='sched_arms_')
    state_dir = os.path.join(tmpdir, 'rem_state')
    try:
        # --- 1) SchedulerConfig 新パラメータ ---
        cfg = make_config(tmpdir, kkf_state_dir=state_dir, kkf_state_save=True)
        if cfg.assigner != 'hungarian' or not cfg.varmap_enabled:
            failures.append("SchedulerConfig が新パラメータを読めていない")
        road = node.RoadCoordinate(cfg.road_points)

        # --- 2) kkf_mpc: RBF学習 → 保存 → 復元 (kkf_conv 相当) ---
        pred = node.make_predictor(cfg, road)
        rng = np.random.default_rng(1)
        # 1走行分: car_1 が s=0→300 を掃引、全BSを観測
        t, s = 0.0, 0.0
        while s < 300.0:
            entries = [(0, b, true_field(s, -60.0 + 60.0 * b) + rng.normal(0, 1.0))
                       for b in range(3)]
            pred.ingest('car_1', t, [s], entries)
            t += 0.05
            s = 16.67 * t
        pred.save_state(t)
        files = sorted(os.listdir(state_dir))
        if files != ['bs0.npz', 'bs1.npz', 'bs2.npz']:
            failures.append(f"save_state のファイル構成が不正: {files}")

        # 復元: 平均場が引き継がれている (観測ゼロで真値近傍を予測)
        cfg2 = make_config(tmpdir, kkf_state_dir=state_dir, kkf_state_save=False)
        pred2 = node.make_predictor(cfg2, road)
        s_q = 80.0 + 60.0  # bs1 のピーク (bs_x=0 → s=140)
        m_cold = node.make_predictor(make_config(tmpdir), road).lcb(
            'car_1', 0, 1, s_q, 999.0, 0.0)
        m_conv = pred2.lcb('car_1', 0, 1, s_q, 999.0, 0.0)
        err_conv = abs(m_conv - true_field(s_q, 0.0))
        print(f"  kkf_conv 復元: 予測 {m_conv:.1f} (真値 {true_field(s_q, 0.0):.1f}, "
              f"cold {m_cold:.1f})")
        if err_conv > 3.0:
            failures.append(f"復元後の平均場誤差 {err_conv:.1f} > 3dB")
        if abs(m_cold - true_field(s_q, 0.0)) < 5.0:
            failures.append("cold 開始が学習済みの値を返している (状態リーク)")
        if pred2.run_count != 1:
            failures.append(f"run_count 継承が不正: {pred2.run_count}")

        # 忘却: P が load 時に膨らむ
        # 地図キーは (BS index, 進行方向)。本テストの車は全て +x 方向なので dir=1
        p_saved = pred.maps[(1, 1)].P
        if not (np.diag(pred2.maps[(1, 1)].P) > np.diag(p_saved) + 0.049).all():
            failures.append("q_forget が load 時に適用されていない")

        # --- 3) σ_ν²(s): 遮蔽帯の残差が予測分散に現れる ---
        cfg3 = make_config(tmpdir)
        pred3 = node.make_predictor(cfg3, road)
        rng = np.random.default_rng(2)
        for run in range(15):
            t, s = 0.0, 0.0
            while s < 300.0:
                z = true_field(s, 0.0) + rng.normal(0, 1.0)
                if 100.0 <= s <= 120.0 and rng.random() < 0.3:
                    z -= 26.0  # 確率的遮蔽イベント
                pred3.ingest('car_1', t + run * 100.0, [s], [(0, 1, z)])
                t += 0.05
                s = 16.67 * t
        var_in = pred3.varmaps[(1, 1)].query(110.0)
        var_out = pred3.varmaps[(1, 1)].query(60.0)
        print(f"  σ_ν²: 遮蔽帯 {var_in:.1f} dB² / 帯外 {var_out:.1f} dB²")
        if var_in < 3.0 * max(var_out, 1.0):
            failures.append(f"σ_ν² が遮蔽帯を学習していない ({var_in:.1f} vs {var_out:.1f})")
        lcb_in = pred3.lcb('car_1', 0, 1, 110.0, 9999.0, 1.0)
        lcb_in_no_k = pred3.lcb('car_1', 0, 1, 110.0, 9999.0, 0.0)
        if lcb_in_no_k - lcb_in < math.sqrt(var_in) * 0.5:
            failures.append("σ_ν² が LCB (κσ) に反映されていない")

        # --- 4) trend / oracle / ts_kf フォールバック ---
        # trend: プロファイル必須
        try:
            node.make_predictor(make_config(tmpdir, control_plane='trend'), road)
            failures.append("trend がプロファイル無しで構築できてしまう")
        except ValueError:
            pass
        # oracle: 実測の最新値のみ・staleness
        orc = node.make_predictor(
            make_config(tmpdir, noise_std=0.0, control_plane='oracle'), road)
        orc.ingest('car_1', 10.0, [50.0], [(0, 2, -55.5)])
        if abs(orc.lcb('car_1', 0, 2, 0.0, 10.1, 1.0) - (-55.5)) > 1e-9:
            failures.append("oracle が最新真値を返さない")
        if orc.lcb('car_1', 0, 2, 0.0, 20.0, 1.0) > -140.0:
            failures.append("oracle の staleness が効いていない")
        if orc.lcb('car_1', 0, 0, 0.0, 10.1, 1.0) > -140.0:
            failures.append("oracle が未観測ペアに値を返した")
        # ts_kf: フォールバック無効時は無情報事前
        tskf = node.make_predictor(make_config(tmpdir, control_plane='ts_kf'), road)
        v = tskf.lcb('car_1', 0, 0, 50.0, 1.0, 1.0)
        if abs(v - (-120.0 - 20.0)) > 1e-6:
            failures.append(f"ts_kf 無情報事前の LCB が想定外: {v:.1f}")

        # --- 5) hungarian 割当 (gz I/O 込み) ---
        os.environ['GZ_PARTITION'] = 'sched_arms_test'
        cfg5 = make_config(tmpdir, noise_std=0.0, control_plane='oracle')
        sched = node.MpcScheduler(cfg5)
        # 3車から同時レポート (car_1 が全BSで最良、bs 排他を強制する行列)
        import comms_sim_proto.comms_sim_msgs_pb2 as msgs
        rssi = {('car_1', 0): -50, ('car_1', 1): -52, ('car_1', 2): -54,
                ('car_2', 0): -60, ('car_2', 1): -55, ('car_2', 2): -70,
                ('car_3', 0): -58, ('car_3', 1): -75, ('car_3', 2): -90}
        for vid, x in (('car_1', 0.0), ('car_2', -20.0), ('car_3', -40.0)):
            m = msgs.MeasurementReport(t_sim=1.0, vehicle=vid)
            m.vehicle_pos.x, m.vehicle_pos.y, m.vehicle_pos.z = x, 0.0, 0.0
            for b in range(3):
                m.reports.add(ant=0, bs=b, rssi_dbm=rssi[(vid, b)],
                              link_state='CONNECTED', mode=msgs.DATA)
            sched._process_report(m)
        sched._replan(1.0)
        pairs = {vid: p for vid, p in sched.current_pair.items() if p is not None}
        assigned_bs = [p[1] for p in pairs.values()]
        print(f"  hungarian: {pairs}")
        if len(assigned_bs) != 3 or len(set(assigned_bs)) != 3:
            failures.append(f"hungarian のBS排他/割当数が不正: {pairs}")
        # 総効用最大: {c3:0,c2:1,c1:2} = -58-55-54 = -167 が全順列の最適
        # ({c1:0,c2:1,c3:2} = -195 の「各車が自分の最良を取る」より良い)
        if pairs != {'car_3': (0, 0), 'car_2': (0, 1), 'car_1': (0, 2)}:
            failures.append(f"hungarian が総効用最大を外した: {pairs}")

        # ヒステリシス: 単独車で「改善 < bonus は維持、改善 > bonus は切替」
        sched2 = node.MpcScheduler(cfg5)

        def report_one(t, r0, r1, r2):
            m = msgs.MeasurementReport(t_sim=t, vehicle='car_1')
            m.vehicle_pos.x, m.vehicle_pos.y, m.vehicle_pos.z = 0.0, 0.0, 0.0
            for b, r in enumerate((r0, r1, r2)):
                m.reports.add(ant=0, bs=b, rssi_dbm=r,
                              link_state='CONNECTED', mode=msgs.DATA)
            sched2._process_report(m)

        report_one(1.0, -50.0, -52.0, -54.0)
        sched2._replan(1.0)
        if sched2.current_pair['car_1'] != (0, 0):
            failures.append(f"初回割当が最良BSでない: {sched2.current_pair}")
        report_one(1.2, -50.0, -48.5, -54.0)   # 改善 1.5dB < bonus 3dB → 維持
        sched2._replan(1.2)
        if sched2.current_pair['car_1'] != (0, 0):
            failures.append(f"ヒステリシスが効いていない: {sched2.current_pair}")
        report_one(1.4, -50.0, -45.0, -54.0)   # 改善 5dB > bonus 3dB → 切替
        sched2._replan(1.4)
        if sched2.current_pair['car_1'] != (0, 1):
            failures.append(f"明確な改善でも切替しない: {sched2.current_pair}")
        # idle: 全ペアが idle_lcb 未満の車は割当なし
        u = np.full((2, 3), -120.0)
        u[0, 0] = -60.0
        p, _ = node.solve_assignment(u, min_utility=-110.0)
        if p != {0: 0}:
            failures.append(f"idle 判定が不正: {p}")

    finally:
        shutil.rmtree(tmpdir, ignore_errors=True)

    if failures:
        print("FAIL:")
        for f in failures:
            print(f"  - {f}")
        sys.exit(1)
    print("PASS: scheduler_arms_test 全項目合格")


if __name__ == '__main__':
    main()
