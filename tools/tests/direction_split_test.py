#!/usr/bin/env python3
"""
方向別REM (案A) と一括LCB評価の検証。

  1. 単一方向シナリオでは従来と同一構成に縮退する (road_10car 互換)
  2. 双方向では (RSU, 方向) ごとに地図が分かれ、片方向の学習が他方に漏れない
  3. 弧長 s が +x 向きに単調 = 上り車の v̂ が負になり外挿が正しい
  4. lcb_multi (一括) と lcb (逐次) が数値一致する
  5. 状態ファイルが方向別に分かれ、単一方向では従来名のまま

コンテナ内で実行:
  docker compose exec -T sim bash -c \
    "source /workspace/install/setup.bash && python3 /workspace/tools/tests/direction_split_test.py"
"""
import os
import shutil
import sys
import tempfile

import numpy as np
import yaml

sys.path.insert(0, '/workspace/src/comms_sim_pkg/comms_sim_pkg')
import kkf_scheduler_node as node  # noqa: E402


def make_config(tmpdir, bidirectional, **link_overrides):
    link = {
        'control_plane': 'kkf_mpc', 'kkf_use_prior': False, 'kkf_basis': 'rbf',
        'kkf_rbf_num_bases': 20, 'kkf_varmap_enabled': True,
        'kkf_assigner': 'hungarian', 'kkf_kappa': 1.0, 'kkf_tracker_enabled': False,
    }
    link.update(link_overrides)
    vehicles = [{'name': 'up_1', 'pose': [-90.0, 1.75, 0, 0, 0, 0],
                 'waypoints': [[-90.0, 1.75, 0.0, 16.7], [90.0, 1.75, 0.0, 16.7]],
                 'antennas': [{'name': 'up_1_ant', 'offset': [0.0, 0.0, 1.35],
                               'relative_rpy': [0.0, 0.0, 0.349]}]}]
    if bidirectional:
        vehicles.append({'name': 'dn_1', 'pose': [90.0, -1.75, 0, 0, 0, 3.14159],
                         'waypoints': [[90.0, -1.75, 0.0, 16.7], [-90.0, -1.75, 0.0, 16.7]],
                         'antennas': [{'name': 'dn_1_ant', 'offset': [0.0, 0.0, 1.35],
                                       'relative_rpy': [0.0, 0.0, -0.349]}]})
    cfg_dict = {
        'link_controller_node': {'ros__parameters': link},
        'comms_simulator_node': {'ros__parameters': {
            'measurement_report': {'noise_std_db': 2.0, 'observe_all_pairs': False}}},
        'vehicles': vehicles,
        'spawn_entities': {
            f'antenna_{i}': {'pose': [-15.0 + 10.0 * i, 6.0, 0.0, 0.0, 0.0, -2.79],
                             'antenna_offset': [0.0, 0.0, 2.5],
                             'antenna_relative_rpy': [0.0, 0.0, 0.0]}
            for i in range(4)},
        'simulation': {'output_subdir': ''},
    }
    path = os.path.join(tmpdir, f"p_{bidirectional}.yaml")
    with open(path, 'w') as f:
        yaml.dump(cfg_dict, f)
    return node.SchedulerConfig(path)


def main():
    failures = []
    tmpdir = tempfile.mkdtemp(prefix='dirsplit_')
    try:
        # --- 1) 単一方向は従来構成に縮退 ---
        cfg1 = make_config(tmpdir, False)
        road1 = node.RoadCoordinate(cfg1.road_points)
        p1 = node.make_predictor(cfg1, road1)
        if cfg1.directions != [1] or p1.split:
            failures.append(f"単一方向で分割された: dirs={cfg1.directions} split={p1.split}")
        if len(p1.maps) != 4:
            failures.append(f"単一方向の地図数が不正: {len(p1.maps)} != 4")
        if p1._state_key((2, 1)) != '2':
            failures.append(f"単一方向の状態キーが従来形式でない: {p1._state_key((2, 1))}")

        # --- 2) 双方向は (RSU, 方向) に分かれる ---
        cfg = make_config(tmpdir, True)
        road = node.RoadCoordinate(cfg.road_points)
        pred = node.make_predictor(cfg, road)
        if cfg.directions != [-1, 1] or not pred.split:
            failures.append(f"双方向を検出できていない: {cfg.directions}")
        if len(pred.maps) != 8:
            failures.append(f"双方向の地図数が不正: {len(pred.maps)} != 8 (4RSU×2方向)")
        if cfg.vehicle_dirs != {'up_1': 1, 'dn_1': -1}:
            failures.append(f"進行方向の判定が不正: {cfg.vehicle_dirs}")
        print(f"  方向検出: {cfg.vehicle_dirs}, 地図 {len(pred.maps)}個")

        # --- 3) 道路は +x 単調 (上り車の弧長が増加、下り車は減少) ---
        s_left = road.project(np.array([-80.0, 0.0, 0.0]))
        s_right = road.project(np.array([80.0, 0.0, 0.0]))
        if not s_right > s_left:
            failures.append("弧長 s が +x 向きに増加していない")

        # --- 4) 片方向の学習が他方に漏れない ---
        rng = np.random.default_rng(0)
        for step in range(200):        # up_1 のみが bs2 を観測
            t = step * 0.05
            s = road.project(np.array([-90.0 + 16.7 * t, 1.75, 0.0]))
            pred.ingest('up_1', t, [s], [(0, 2, -55.0 + rng.normal(0, 1.0))])
        s_probe = road.project(np.array([0.0, 0.0, 0.0]))
        v_up = pred.lcb('up_1', 0, 2, s_probe, 1e4, 0.0)
        v_dn = pred.lcb('dn_1', 0, 2, s_probe, 1e4, 0.0)
        print(f"  学習後 bs2: 上り {v_up:.1f} dBm / 下り {v_dn:.1f} dBm (下りは未学習)")
        if abs(v_up - (-55.0)) > 5.0:
            failures.append(f"上り方向の学習が反映されていない: {v_up:.1f}")
        if abs(v_dn - (-55.0)) < 10.0:
            failures.append(f"下り方向に学習が漏れている: {v_dn:.1f}")

        # --- 5) lcb_multi (一括) と lcb (逐次) の数値一致。
        #        lcb_multi は (LCB, 平均) を返す — 接続可能性は平均で判定し、
        #        σ は順位付けにのみ使うため (門番に LCB を使うと繋がる機会を
        #        見送る。競合下で配信ゼロ車 18% を招いた実測あり) ---
        K = 8
        t0 = 10.0
        t_grid = t0 + np.arange(K) * 0.05
        queries = []
        for vid in ('up_1', 'dn_1'):
            s0 = road.project(np.array([-10.0, 0.0, 0.0]))
            v = 16.7 if cfg.vehicle_dirs[vid] > 0 else -16.7
            queries.append((vid, 0, s0 + v * np.arange(K) * 0.05, t_grid))
        for b in range(4):
            multi = pred.lcb_multi(queries, b, 1.0)
            for q, (vid, ant, s_arr, ta) in enumerate(queries):
                lcb_v, mu_v = multi[q]
                seq_lcb = np.array([pred.lcb(vid, ant, b, s, tt, 1.0)
                                    for s, tt in zip(s_arr, ta)])
                seq_mu = np.array([pred.lcb(vid, ant, b, s, tt, 0.0)
                                   for s, tt in zip(s_arr, ta)])
                d1 = np.abs(lcb_v - seq_lcb).max()
                d2 = np.abs(mu_v - seq_mu).max()
                if d1 > 1e-9:
                    failures.append(f"bs{b} {vid}: LCB が逐次と不一致 ({d1:.2e})")
                if d2 > 1e-9:
                    failures.append(f"bs{b} {vid}: 平均が逐次(κ=0)と不一致 ({d2:.2e})")
                if not (mu_v >= lcb_v - 1e-9).all():
                    failures.append(f"bs{b} {vid}: 平均が LCB を下回った (κ>0 で不整合)")
        print("  lcb_multi の (LCB, 平均) が逐次と一致し、平均 >= LCB を確認")

        # --- 5b) 方向集合は交通実現でなく環境が決める (状態キーの安定性) ---
        # 対象車は確率生成なので run によっては片方向しか出ない。車両から
        # 推定していると run ごとに状態キーが変わり、走行間で状態を読めず
        # 学習が一切蓄積しなくなる (実際に 10 走行を無駄にした失敗モード)
        cfg_uni = make_config(tmpdir, False, kkf_directions=[-1, 1])
        if cfg_uni.directions != [-1, 1]:
            failures.append(f"kkf_directions の明示が効いていない: {cfg_uni.directions}")
        p_uni = node.make_predictor(cfg_uni, node.RoadCoordinate(cfg_uni.road_points))
        if not p_uni.split or p_uni._state_key((0, 1)) != '0_p':
            failures.append("片方向の交通でも宣言どおり方向別キーになっていない")
        cfg_bi = make_config(tmpdir, True, kkf_directions=[-1, 1])
        p_bi = node.make_predictor(cfg_bi, node.RoadCoordinate(cfg_bi.road_points))
        if p_uni._state_key((0, 1)) != p_bi._state_key((0, 1)):
            failures.append("交通実現によって状態キーが変わる (学習が蓄積しない)")
        print("  状態キーの安定性: 片方向/双方向どちらの交通でも同一キー")

        # --- 5c) 座標系も環境が決める (地図の意味と基底定義域の安定性) ---
        # 車両の x 範囲から道路を作ると、交通実現ごとに同じ物理位置が違う弧長に
        # なり、地図が走行間で意味を持たない。基底の定義域も変わって状態が
        # 読めなくなる (実測: 道路長 1441〜1844m とばらつき、ハッシュが毎回別物)
        from kkf_core import basis_hash, RbfBasis  # noqa: E402
        hashes, lengths = set(), set()
        for extra_x in (0.0, 500.0, 1200.0):   # staging 位置の違いを模した外れ値
            c = make_config(tmpdir, True, kkf_road_x_range=[-115.0, 115.0],
                            kkf_rbf_s_min=0.0, kkf_rbf_s_max=230.0)
            # 車両を遠方に置いても道路は宣言どおり
            c.vehicles[0]['antenna_offsets'] = c.vehicles[0]['antenna_offsets']
            rd = node.RoadCoordinate(c.road_points)
            lengths.add(round(rd.total_length(), 3))
            smax = c.rbf_s_max if c.rbf_s_max > c.rbf_s_min else rd.total_length()
            hashes.add(basis_hash(RbfBasis(c.rbf_s_min, smax,
                                           c.rbf_num_bases, c.rbf_width_m).config()))
        if len(lengths) != 1 or len(hashes) != 1:
            failures.append(f"座標系が run で変わる: 長さ{lengths} ハッシュ{len(hashes)}種")
        print(f"  座標系の安定性: 道路長 {lengths.pop()}m, 基底ハッシュ 1種で固定")

        # --- 6) 状態ファイルが方向別に分かれる ---
        sd = os.path.join(tmpdir, 'rem_state')
        cfg_s = make_config(tmpdir, True, kkf_state_dir=sd, kkf_state_save=True)
        ps = node.make_predictor(cfg_s, node.RoadCoordinate(cfg_s.road_points))
        ps.save_state(1.0)
        files = sorted(os.listdir(sd))
        expect = sorted([f"bs{b}_{d}.npz" for b in range(4) for d in ('m', 'p')])
        if files != expect:
            failures.append(f"方向別の状態ファイル名が不正: {files}")
        print(f"  状態ファイル: {files[:3]} ... ({len(files)}個)")

    finally:
        shutil.rmtree(tmpdir, ignore_errors=True)

    if failures:
        print("FAIL:")
        for f in failures:
            print(f"  - {f}")
        sys.exit(1)
    print("PASS: direction_split_test 全項目合格")


if __name__ == '__main__':
    main()
