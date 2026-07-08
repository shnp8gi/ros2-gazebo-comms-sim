#!/usr/bin/env python3
"""
BlockageTracker 単体テスト: 合成残差ストリームで移動遮蔽帯の追跡精度を検証。
コンテナ内で実行:
  docker compose exec -T sim bash -c \
    "source /workspace/install/setup.bash && python3 /workspace/tools/tests/blockage_tracker_test.py"
"""
import sys

import numpy as np

from comms_sim_pkg.kkf_core import TrackerParams, BlockageTracker

TRUE_V = 12.0       # 遮蔽帯の真の移動速度 [m/s]
TRUE_HW = 10.0      # 真の半幅 [m]
C0 = -50.0
RATE_HZ = 20.0
DURATION_S = 3.0


def run_case(noise_seed):
    rng = np.random.default_rng(noise_seed)
    tracker = BlockageTracker(TrackerParams())
    for k in range(int(DURATION_S * RATE_HZ)):
        t = k / RATE_HZ
        c = C0 + TRUE_V * t
        # 車載アンテナ相当の3点サンプル (中心±12m) + 位置ノイズ
        samples = []
        for offset in (-12.0, 0.0, 12.0):
            s = c + offset + rng.normal(0, 1.0)
            residual = -25.0 if abs(s - c) <= TRUE_HW else rng.normal(0, 2.0)
            samples.append((s, residual))
        tracker.ingest(t, samples)
    return tracker


def main():
    failures = []
    for seed in range(5):
        tracker = run_case(seed)
        confirmed = [tr for tr in tracker.tracks if tr.is_confirmed()]
        if len(confirmed) != 1:
            failures.append(f"seed{seed}: confirmed tracks = {len(confirmed)} (期待 1)")
            continue
        tr = confirmed[0]
        t_end = DURATION_S
        c_true_now = C0 + TRUE_V * t_end
        c_true_future = C0 + TRUE_V * (t_end + 1.0)

        v_err = abs(tr.velocity - TRUE_V)
        zones = tracker.zones(t_end + 1.0)
        in_zone = any(lo <= c_true_future <= hi for lo, hi in zones)
        print(f"  seed{seed}: v̂={tr.velocity:+.2f} (真値 {TRUE_V}), "
              f"ĉ(now)={tr.predict_center(t_end):.1f} (真値 {c_true_now:.1f}), "
              f"1秒先ゾーン={['%.1f..%.1f' % z for z in zones]} 命中={in_zone}")
        if v_err > 3.0:
            failures.append(f"seed{seed}: 速度誤差 {v_err:.2f} > 3.0 m/s")
        if not in_zone:
            failures.append(f"seed{seed}: 1秒先の真の中心がゾーン外")

    # 遮蔽なしストリームで誤検出しないこと
    rng = np.random.default_rng(99)
    tracker = BlockageTracker(TrackerParams())
    for k in range(60):
        samples = [(k * 4.0 + o, rng.normal(0, 2.0)) for o in (-12.0, 0.0, 12.0)]
        tracker.ingest(k / RATE_HZ, samples)
    ghost = [tr for tr in tracker.tracks if tr.is_confirmed()]
    print(f"  no-blockage: confirmed tracks = {len(ghost)} (期待 0)")
    if ghost:
        failures.append("遮蔽なしで誤検出")

    if failures:
        print("TRACKER TEST FAILED:")
        for f in failures:
            print("  " + f)
        sys.exit(1)
    print("TRACKER TEST PASSED")


if __name__ == '__main__':
    main()
