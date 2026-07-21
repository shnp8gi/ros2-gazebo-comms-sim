#!/usr/bin/env python3
"""
ScalarRssiKF 単体テスト: 時系列KFベースライン予測器の推定・外挿を検証。
コンテナ内で実行:
  docker compose exec -T sim bash -c \
    "source /workspace/install/setup.bash && python3 /workspace/tools/tests/scalar_kf_test.py"
"""
import sys

import numpy as np

from comms_sim_pkg.kkf_core import ScalarKfParams, ScalarRssiKF

RATE_HZ = 20.0
NOISE_STD = 2.0
NOISE_VAR = NOISE_STD ** 2


def feed(kf, rng, level_fn, duration_s):
    t = 0.0
    for k in range(int(duration_s * RATE_HZ)):
        t = k / RATE_HZ
        kf.update(t, level_fn(t) + rng.normal(0, NOISE_STD), NOISE_VAR)
    return t


def main():
    failures = []

    # 1) 無観測ペアは事前分布を返す
    params = ScalarKfParams(prior_mean_dbm=-120.0, prior_var=400.0)
    kf = ScalarRssiKF(params)
    mean, var = kf.predict_at(10.0)
    print(f"  無観測: mean={mean:.1f} var={var:.1f}")
    if mean != -120.0 or var != 400.0:
        failures.append("無観測ペアが事前分布を返さない")

    # 2) 定常信号への収束 (レベル誤差 < 1dB)
    for seed in range(5):
        rng = np.random.default_rng(seed)
        kf = ScalarRssiKF(ScalarKfParams())
        t_end = feed(kf, rng, lambda t: -60.0, 3.0)
        mean, _ = kf.predict_at(t_end)
        if abs(mean - (-60.0)) > 1.0:
            failures.append(f"seed{seed}: 定常信号のレベル誤差 {abs(mean + 60):.2f} > 1dB")
    print(f"  定常信号: 最終推定 {mean:.2f} dBm (真値 -60)")

    # 3) ランプ信号の追従と1秒先外挿 (真値 -40-8t、外挿誤差 < 3dB)
    for seed in range(5):
        rng = np.random.default_rng(100 + seed)
        kf = ScalarRssiKF(ScalarKfParams())
        t_end = feed(kf, rng, lambda t: -40.0 - 8.0 * t, 3.0)
        pred, _ = kf.predict_at(t_end + 1.0)
        true_future = -40.0 - 8.0 * (t_end + 1.0)
        err = abs(pred - true_future)
        if err > 3.0:
            failures.append(f"seed{seed}: ランプ1秒先外挿誤差 {err:.2f} > 3dB")
    print(f"  ランプ外挿: 1秒先予測 {pred:.2f} (真値 {true_future:.2f}, 誤差 {err:.2f}dB)")

    # 4) 外挿分散はリード時間とともに単調増加
    variances = [kf.predict_at(t_end + dt)[1] for dt in (0.0, 0.5, 1.0, 2.0)]
    print(f"  分散成長: {['%.1f' % v for v in variances]}")
    if not all(v2 > v1 for v1, v2 in zip(variances, variances[1:])):
        failures.append("外挿分散がリード時間に対して単調増加しない")

    # 5) 構造的限界の確認: 過去に観測のない急変 (前方遮蔽) は外挿に現れない
    rng = np.random.default_rng(42)
    kf = ScalarRssiKF(ScalarKfParams())
    t_end = feed(kf, rng, lambda t: -60.0, 3.0)  # 遮蔽前の定常区間のみ観測
    pred, _ = kf.predict_at(t_end + 0.5)          # 真値はこの先 -90 に急落する想定
    print(f"  急変非予測: 遮蔽直前の0.5秒先予測 {pred:.2f} (定常値 -60 近傍のまま)")
    if abs(pred - (-60.0)) > 5.0:
        failures.append("定常観測のみで外挿が -60 近傍から乖離 (KFとして不健全)")

    if failures:
        print("FAIL:")
        for f in failures:
            print(f"  - {f}")
        sys.exit(1)
    print("PASS: scalar_kf_test 全項目合格")


if __name__ == '__main__':
    main()
