#!/usr/bin/env python3
"""
ゴールデンテスト: C++リファレンス (kkf_golden_dump) と Python kkf_core の数値一致検証。
コンテナ内で実行する:
  docker compose exec -T sim bash -c \
    "source /workspace/install/setup.bash && python3 /workspace/tools/tests/golden_kkf_test.py"
"""
import math
import subprocess
import sys

import numpy as np

from comms_sim_pkg.kkf_core import (
    RoadCoordinate, LogDistanceBasis, KkfParams, Observation,
    KrigedKalmanFilter, solve_handover_plan,
)

DUMP_BIN = "/workspace/install/comms_sim_pkg/lib/comms_sim_pkg/kkf_golden_dump"
ATOL = 1e-8


def python_reference():
    """C++ 側と同一のフィクスチャを kkf_core で計算する。"""
    road = RoadCoordinate([[0, 0, 0], [1000, 0, 0]])
    params = KkfParams(process_noise_q=1e-4, initial_state_var=100.0, sigma_nu=4.0,
                       corr_length_s_m=20.0, corr_length_t_s=5.0,
                       residual_buffer_size=64, initial_state_mean=[-30.0, -20.0])
    basis = LogDistanceBasis(road, [150.0, 10.0, 1.0])
    f = KrigedKalmanFilter(basis, params)

    for k in range(40):
        t = 0.05 * (k + 1)
        obs = []
        for s in (5.0 * k, 5.0 * k + 8.0):
            z = -60.0 + 5.0 * math.sin(0.2 * s) + 2.0 * math.sin(1.3 * t)
            obs.append(Observation(s=s, t=t, z=z, noise_var=4.0))
        f.update(t, obs)

    preds = {}
    for i in range(11):
        s = 25.0 * i
        preds[s] = f.predict_at(s, 2.5)

    lcb = np.array([[10.0 * math.sin(1.7 * a + 0.31 * k) for k in range(15)]
                    for a in range(4)])
    assignments, total = solve_handover_plan(lcb, 0.3, 5.0, 1)
    return preds, assignments, total


def main():
    dump = subprocess.run([DUMP_BIN], capture_output=True, text=True, check=True).stdout
    preds_py, assign_py, total_py = python_reference()

    failures = []
    for line in dump.strip().splitlines():
        parts = line.split(',')
        if parts[0] == 'P':
            s = float(parts[1])
            mean_cpp, var_cpp = float(parts[2]), float(parts[3])
            mean_py, var_py = preds_py[s]
            if abs(mean_cpp - mean_py) > ATOL or abs(var_cpp - var_py) > ATOL:
                failures.append(
                    f"KKF s={s}: C++ ({mean_cpp:.12e}, {var_cpp:.12e}) "
                    f"vs Py ({mean_py:.12e}, {var_py:.12e})")
            else:
                print(f"  OK KKF s={s:6.1f}: mean={mean_py:+.6f} var={var_py:.6f}")
        elif parts[0] == 'D':
            assign_cpp = [int(v) for v in parts[1].split()]
            total_cpp = float(parts[2])
            if assign_cpp != assign_py:
                failures.append(f"DP assignments: C++ {assign_cpp} vs Py {assign_py}")
            elif abs(total_cpp - total_py) > ATOL:
                failures.append(f"DP total: C++ {total_cpp:.12e} vs Py {total_py:.12e}")
            else:
                print(f"  OK DP: assignments={assign_py} total={total_py:.6f}")

    if failures:
        print("GOLDEN TEST FAILED:")
        for f_ in failures:
            print("  " + f_)
        sys.exit(1)
    print("GOLDEN TEST PASSED (atol=1e-8)")


if __name__ == '__main__':
    main()
