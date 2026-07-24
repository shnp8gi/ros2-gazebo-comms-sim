#!/usr/bin/env python3
"""
RbfBasis 単体テスト: KKF と組み合わせて「s に非単調な平均場 (指向性ウィンドウ)」を
学習できることを検証する。本番仕様 §5.1 の最大リスク (基底解像度) の回帰テスト。
コンテナ内で実行:
  docker compose exec -T sim bash -c \
    "source /workspace/install/setup.bash && python3 /workspace/tools/tests/rbf_basis_test.py"
"""
import sys

import numpy as np

from comms_sim_pkg.kkf_core import RbfBasis, KkfParams, Observation, KrigedKalmanFilter

S_MIN, S_MAX = 0.0, 200.0


def true_profile(s):
    """距離減衰 + ボアサイト対向ウィンドウ (s=100 にピーク) を模した非単調場。"""
    return -85.0 + 25.0 * np.exp(-0.5 * ((s - 100.0) / 8.0) ** 2) \
        - 0.02 * np.abs(s - 100.0)


def main():
    failures = []

    basis = RbfBasis(S_MIN, S_MAX, num_bases=20)

    # 1) 基本性質
    if basis.dimension() != 20:
        failures.append(f"dimension() = {basis.dimension()} != 20")
    phi = basis.evaluate(100.0)
    if phi.shape != (20,) or not np.all(phi >= 0.0) or phi.max() > 1.0 + 1e-12:
        failures.append("evaluate() の形状・値域が不正")
    cfg = basis.config()
    if cfg['type'] != 'rbf' or cfg['num_bases'] != 20:
        failures.append(f"config() が不正: {cfg}")

    # 2) 非単調プロファイルの学習 (1走行分の掃引観測、雑音 σ=1dB)
    rng = np.random.default_rng(7)
    kkf = KrigedKalmanFilter(basis, KkfParams(sigma_nu=4.0))
    speed, rate_hz = 16.67, 20.0
    t, s = 0.0, S_MIN
    while s < S_MAX:
        z = true_profile(s) + rng.normal(0.0, 1.0)
        kkf.update(t, [Observation(s=s, t=t, z=z, noise_var=1.0)])
        t += 1.0 / rate_hz
        s = speed * t

    # 残差クリギングの時間相関 (5s) が切れた「未来の予測」= 平均場のみで評価する
    t_future = t + 100.0
    s_eval = np.linspace(20.0, 180.0, 81)
    means, _ = kkf.predict_batch(s_eval, np.full_like(s_eval, t_future))
    err = means - true_profile(s_eval)
    rms = float(np.sqrt(np.mean(err ** 2)))
    peak_err = float(abs(means[np.argmin(np.abs(s_eval - 100.0))]
                         - true_profile(100.0)))
    print(f"  平均場学習: RMS={rms:.2f}dB ピーク誤差={peak_err:.2f}dB")
    if rms > 2.0:
        failures.append(f"平均場 RMS 誤差 {rms:.2f} > 2.0dB (基底解像度不足)")
    if peak_err > 2.5:
        failures.append(f"ボアサイトピーク誤差 {peak_err:.2f} > 2.5dB")

    # 3) ピーク位置 (argmax) が正しく再現される = 「距離順≠品質順」を学習できる
    s_peak = float(s_eval[np.argmax(means)])
    print(f"  ピーク位置: 推定 s={s_peak:.1f} (真値 100.0)")
    if abs(s_peak - 100.0) > 5.0:
        failures.append(f"ピーク位置誤差 {abs(s_peak - 100.0):.1f} > 5m")

    if failures:
        print("FAIL:")
        for f in failures:
            print(f"  - {f}")
        sys.exit(1)
    print("PASS: rbf_basis_test 全項目合格")


if __name__ == '__main__':
    main()
