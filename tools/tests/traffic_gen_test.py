#!/usr/bin/env python3
"""
traffic_gen 単体テスト: 決定論展開・CRN・車種構成・到着幾何を検証 (本番仕様 §3.3)。
ホストで実行可能 (stdlib + yaml のみ):
  python3 tools/tests/traffic_gen_test.py
"""
import copy
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from lib import traffic_gen  # noqa: E402


def base_scenario():
    return {
        'scenario': {
            'model_catalog': {
                'SedanBlocker': {'blockage': {'size': [4.5, 1.8, 1.5], 'loss_db': 8.0}},
                'TruckBlocker': {'blockage': {'size': [12, 2.5, 3.8], 'loss_db': 26.0}},
            },
            'traffic': {
                'window_s': [0.0, 30.0],
                'corridor': [-150.0, 150.0],
                'margin_m': 30.0,
                'tx_entry_jitter_s': 0.0,
                'vehicle_mix': [
                    {'model': 'SedanBlocker', 'ratio': 0.7},
                    {'model': 'TruckBlocker', 'ratio': 0.3},
                ],
                'lanes': [
                    {'y': 4.5, 'direction': -1, 'speed_mps': 16.7,
                     'headway_s': {'dist': 'exponential', 'mean': 4.0, 'min': 1.0}},
                    {'y': 2.5, 'direction': 1, 'speed_mps': 5.0,
                     'headway_s': {'dist': 'lognormal', 'mean': 5.0, 'sigma': 0.6,
                                   'min': 1.5}},
                ],
            },
            'entities': [
                {'name': 'car_1', 'model': 'Car', 'role': 'tx',
                 'pose': [-150.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                 'waypoints': [[-150.0, 0.0, 0.0, 16.67], [150.0, 0.0, 0.0, 16.67]]},
            ],
        },
    }


def blockers(scen):
    return [e for e in scen['scenario']['entities'] if e.get('role') == 'blocker']


def main():
    failures = []

    # 1) 決定論性: 同一シードで完全一致
    s1, s2 = base_scenario(), base_scenario()
    n1 = traffic_gen.expand_traffic(s1, 777)
    n2 = traffic_gen.expand_traffic(s2, 777)
    if n1 != n2 or blockers(s1) != blockers(s2):
        failures.append("同一シードで展開結果が一致しない")
    if n1 == 0:
        failures.append("展開が0台 (window/流量の既定が壊れている)")

    # 2) シードが違えば実現が変わる
    s3 = base_scenario()
    traffic_gen.expand_traffic(s3, 778)
    if blockers(s1) == blockers(s3):
        failures.append("シードを変えても交通が変わらない")

    # 3) 到着幾何: 初期位置はコリドー入口の v·t_a 手前 (= 窓内到着) にある
    for e in blockers(s1):
        x0, y = e['pose'][0], e['pose'][1]
        wps = e['waypoints']
        v = wps[0][3]
        lane = next(l for l in s1['scenario']['traffic']['lanes'] if l['y'] == y)
        if lane['direction'] > 0:
            t_arr = (-150.0 - x0) / v      # 入口 = corridor[0]
            if wps[-1][0] < 150.0:
                failures.append(f"{e['name']}: 終端がコリドーを走り抜けない")
        else:
            t_arr = (x0 - 150.0) / v       # 入口 = corridor[1]
            if wps[-1][0] > -150.0:
                failures.append(f"{e['name']}: 終端がコリドーを走り抜けない")
        if not (0.0 <= t_arr <= 30.0 + 1e-9):
            failures.append(f"{e['name']}: 到着時刻 {t_arr:.1f}s が窓外")
    print(f"  展開 {n1} 台、到着幾何 OK")

    # 4) 名前の一意性と役割
    names = [e['name'] for e in blockers(s1)]
    if len(names) != len(set(names)):
        failures.append("blocker 名が重複")

    # 5) 車種構成比 (多数シードの集計で 0.7/0.3 ± 0.1)
    counts = {'SedanBlocker': 0, 'TruckBlocker': 0}
    total = 0
    for seed in range(200):
        s = base_scenario()
        traffic_gen.expand_traffic(s, seed)
        for e in blockers(s):
            counts[e['model']] += 1
            total += 1
    ratio = counts['SedanBlocker'] / total
    print(f"  車種構成: Sedan {ratio:.3f} (設定 0.70, n={total})")
    if abs(ratio - 0.7) > 0.05:
        failures.append(f"車種構成比が設定から乖離: {ratio:.3f}")

    # 6) tx 進入ジッタ: tx のみ移動し、決定論的で、σ に見合う
    s4, s5 = base_scenario(), base_scenario()
    for s in (s4, s5):
        s['scenario']['traffic']['tx_entry_jitter_s'] = 0.5
    traffic_gen.expand_traffic(s4, 900)
    traffic_gen.expand_traffic(s5, 900)
    tx4 = next(e for e in s4['scenario']['entities'] if e.get('role') == 'tx')
    tx5 = next(e for e in s5['scenario']['entities'] if e.get('role') == 'tx')
    if tx4['pose'][0] == -150.0:
        failures.append("ジッタが tx に適用されていない")
    if tx4['pose'][0] != tx5['pose'][0]:
        failures.append("ジッタが決定論的でない")
    if abs(tx4['pose'][0] - tx4['waypoints'][0][0]) > 1e-9:
        failures.append("pose と先頭 waypoint がずれた")

    # 7) traffic: なしは何もしない / catalog 不備はエラー
    s6 = {'scenario': {'entities': []}}
    if traffic_gen.expand_traffic(s6, 1) != 0:
        failures.append("traffic なしで展開された")
    s7 = base_scenario()
    del s7['scenario']['model_catalog']['TruckBlocker']
    try:
        traffic_gen.expand_traffic(s7, 1)
        failures.append("catalog 欠落がエラーにならない")
    except ValueError:
        pass

    if failures:
        print("FAIL:")
        for f in failures:
            print(f"  - {f}")
        sys.exit(1)
    print("PASS: traffic_gen_test 全項目合格")


if __name__ == '__main__':
    main()
