#!/usr/bin/env python3
"""
AleatoricVarianceMap 単体テスト: 確率的遮蔽イベントの「場所の変わりやすさ」が
σ_ν²(s) として自己組織化されることを検証 (本番仕様 §5.2)。
コンテナ内で実行:
  docker compose exec -T sim bash -c \
    "source /workspace/install/setup.bash && python3 /workspace/tools/tests/variance_map_test.py"
"""
import sys

import numpy as np

from comms_sim_pkg.kkf_core import VarianceMapParams, AleatoricVarianceMap

ZONE = (80.0, 100.0)      # 遮蔽リスク帯
BLOCK_LOSS = -26.0        # トラック遮蔽の残差 [dB]
P_BLOCK = 0.12            # 遮蔽発生率
NOISE_STD = 2.0


def run_pass(vmap, rng):
    """1走行分: s を 1m 刻みで掃引し、リスク帯では確率的に大残差を落とす。"""
    for s in np.arange(0.0, 200.0, 1.0):
        r = rng.normal(0.0, NOISE_STD)
        if ZONE[0] <= s <= ZONE[1] and rng.random() < P_BLOCK:
            r += BLOCK_LOSS
        vmap.update(s, r)


def main():
    failures = []
    params = VarianceMapParams(s_min=0.0, s_max=200.0, grid_m=2.0,
                               prior_var_db2=9.0, min_weight=0.5)
    vmap = AleatoricVarianceMap(params)

    # 1) 未観測時は事前分散
    if abs(vmap.query(50.0) - 9.0) > 1e-9:
        failures.append(f"未観測の query が事前分散でない: {vmap.query(50.0)}")

    # 2) 50走行でリスク帯の分散が浮き上がる
    rng = np.random.default_rng(3)
    for _ in range(50):
        run_pass(vmap, rng)
    var_in = float(np.mean(vmap.query_batch(np.arange(84.0, 97.0, 1.0))))
    var_out = float(np.mean(vmap.query_batch(np.arange(20.0, 60.0, 1.0))))
    # 理論値: 帯内 ≈ σ² + p·loss² = 4 + 0.12·676 ≈ 85、帯外 ≈ 4
    print(f"  リスク帯 {var_in:.1f} dB² / 帯外 {var_out:.1f} dB² "
          f"(理論 ≈85 / 4)")
    if var_in < 3.0 * var_out:
        failures.append(f"リスク帯のコントラスト不足 ({var_in:.1f} vs {var_out:.1f})")
    if not (2.0 < var_out < 8.0):
        failures.append(f"帯外分散が計測雑音レベルから乖離: {var_out:.1f}")

    # 3) contrast() がコントラストを検出する
    c = vmap.contrast()
    print(f"  contrast = {c:.2f}")
    if c < 2.0:
        failures.append(f"contrast() = {c:.2f} < 2.0")

    # 4) discount は σ_ν²(s) を変えず W だけ減らす (EMA と違いイベントを忘れない)
    before = vmap.query(90.0)
    w_before = vmap.W.copy()
    vmap.discount(0.9)
    after = vmap.query(90.0)
    if abs(before - after) > 1e-9:
        failures.append(f"discount が σ_ν² を変えた: {before:.3f} → {after:.3f}")
    if not np.allclose(vmap.W, w_before * 0.9):
        failures.append("discount が W を γ 倍していない")

    # 5) 割引後は新しい証拠が効きやすい (γ を強くかけて清浄走行 → 分散低下)
    for _ in range(10):
        vmap.discount(0.5)
    rng2 = np.random.default_rng(11)
    for _ in range(20):
        for s in np.arange(0.0, 200.0, 1.0):
            vmap.update(s, rng2.normal(0.0, NOISE_STD))  # 遮蔽イベントなし
    var_in_clean = float(np.mean(vmap.query_batch(np.arange(84.0, 97.0, 1.0))))
    print(f"  強割引+清浄20走行後のリスク帯: {var_in_clean:.1f} dB²")
    if var_in_clean > var_in * 0.5:
        failures.append("割引後も古い遮蔽イベントが支配し続けている (経年変化に不追従)")

    # 6) 状態の復元 (RemStore 連携面)
    vmap2 = AleatoricVarianceMap(params)
    vmap2.load_state(vmap.S2, vmap.W)
    if abs(vmap2.query(90.0) - vmap.query(90.0)) > 1e-12:
        failures.append("load_state 後の query が一致しない")
    try:
        bad = AleatoricVarianceMap(VarianceMapParams(s_min=0, s_max=100, grid_m=2.0))
        bad.load_state(vmap.S2, vmap.W)
        failures.append("格子形状不一致の load_state がエラーにならない")
    except ValueError:
        pass

    if failures:
        print("FAIL:")
        for f in failures:
            print(f"  - {f}")
        sys.exit(1)
    print("PASS: variance_map_test 全項目合格")


if __name__ == '__main__':
    main()
