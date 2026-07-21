#!/usr/bin/env python3
"""
A3Controller 単体テスト: P2P制約 (grant中ペアのみ観測) を模擬した閉ループで
スキャン→アタッチ→A3切替 (hysteresis+TTT) を検証。
コンテナ内で実行:
  docker compose exec -T sim bash -c \
    "source /workspace/install/setup.bash && python3 /workspace/tools/tests/a3_controller_test.py"
"""
import sys

import numpy as np

from comms_sim_pkg.kkf_core import A3Params, A3Controller

RATE_HZ = 20.0
NOISE_STD = 1.0
NUM_ANT = 1
NUM_BS = 3


def granted_pair(plan, t):
    """プラグインと同じ規則: t_start <= t の最後のエントリを適用。"""
    active = None
    for t_start, pair, is_measure in plan:
        if t_start <= t:
            active = (pair, is_measure)
    return active


def run_loop(rssi_fn, duration_s, seed=0, params=None):
    """P2Pループ模擬: 制御が grant したペアのみ観測レポートが返る。"""
    rng = np.random.default_rng(seed)
    ctrl = A3Controller(params or A3Params(), NUM_ANT, NUM_BS)
    measure_ticks = 0
    total_ticks = 0
    for k in range(int(duration_s * RATE_HZ)):
        t = k / RATE_HZ
        active = granted_pair(ctrl.plan(t), t)
        entries = []
        if active is not None:
            pair, is_measure = active
            measure_ticks += int(is_measure)
            total_ticks += 1
            z = rssi_fn(t)[pair] + rng.normal(0, NOISE_STD)
            entries.append((pair // NUM_BS, pair % NUM_BS, z))
        ctrl.ingest(t, entries)
    return ctrl, measure_ticks, total_ticks


def main():
    failures = []

    # 1) コールドスタート: 全ペアをスキャンして最良 (pair1=-55) にアタッチ
    ctrl, _, _ = run_loop(lambda t: [-65.0, -55.0, -80.0], 1.0)
    print(f"  スキャン後: {ctrl.status()}")
    if ctrl.serving != 1:
        failures.append(f"アタッチ先 {ctrl.serving} (期待 1=最良ペア)")
    if ctrl.handover_count != 0:
        failures.append(f"定常環境でHO {ctrl.handover_count} 回 (期待 0)")

    # 2) A3切替: pair0 が t=1.5s 以降 +15dB 改善 → hyst+TTT を経て切替
    def improving(t):
        return [-45.0 if t >= 1.5 else -70.0, -60.0, -80.0]
    for seed in range(5):
        ctrl, _, _ = run_loop(improving, 5.0, seed=seed)
        if ctrl.serving != 0:
            failures.append(f"seed{seed}: 改善ペアへ未切替 (serving={ctrl.serving})")
        elif ctrl.handover_count != 1:
            failures.append(f"seed{seed}: HO回数 {ctrl.handover_count} (期待 1)")
    print(f"  A3切替: serving={ctrl.serving} ho={ctrl.handover_count} (期待 0 へ1回)")

    # 3) ヒステリシス: +1.5dB 差 (< hyst 3dB) では切替しない
    for seed in range(5):
        ctrl, _, _ = run_loop(lambda t: [-60.0, -58.5, -80.0], 5.0, seed=seed)
        if ctrl.handover_count != 0:
            failures.append(f"seed{seed}: hyst未満でHO {ctrl.handover_count} 回")
    print(f"  ヒステリシス: hyst未満の差でHOなし (ho={ctrl.handover_count})")

    # 4) 測定コスト: MEASURE専有率がプローブ設定(duty)の2倍以内の妥当な範囲
    params = A3Params()
    ctrl, meas, total = run_loop(lambda t: [-60.0, -65.0, -80.0], 10.0, params=params)
    duty = meas / total
    expected = params.measure_duration_s / params.measure_period_s
    print(f"  MEASURE専有率: {duty:.3f} (設定duty {expected:.3f})")
    if not (0.3 * expected <= duty <= 2.5 * expected + 0.1):
        failures.append(f"MEASURE専有率 {duty:.3f} が設定 {expected:.3f} から乖離")

    if failures:
        print("FAIL:")
        for f in failures:
            print(f"  - {f}")
        sys.exit(1)
    print("PASS: a3_controller_test 全項目合格")


if __name__ == '__main__':
    main()
