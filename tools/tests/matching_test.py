#!/usr/bin/env python3
"""
solve_assignment 単体テスト: ハンガリアン割当の最適性 (総当たり照合)・
矩形行列・禁止ペア・min_utility を検証 (本番仕様 §6)。
コンテナ内で実行:
  docker compose exec -T sim bash -c \
    "source /workspace/install/setup.bash && python3 /workspace/tools/tests/matching_test.py"
"""
import itertools
import sys

import numpy as np

from comms_sim_pkg.kkf_core import solve_assignment, FORBIDDEN_UTILITY


def enumerate_matchings(U, floor):
    """有効ペア (非禁止・floor 以上) のみからなる全ての部分マッチングを列挙。"""
    n_veh, n_bs = U.shape
    for r in range(min(n_veh, n_bs) + 1):
        for vehs in itertools.combinations(range(n_veh), r):
            for bss in itertools.permutations(range(n_bs), r):
                if all(U[v, b] > FORBIDDEN_UTILITY and U[v, b] >= floor
                       for v, b in zip(vehs, bss)):
                    yield list(zip(vehs, bss))


def brute_force_no_floor(U):
    """min_utility=None の意味論: 実行可能割当数を最大化しつつ効用和を最大化。"""
    best = (-1, -np.inf)
    for m in enumerate_matchings(U, -np.inf):
        val = sum(U[v, b] for v, b in m)
        best = max(best, (len(m), val))
    return best


def brute_force_floor(U, floor):
    """min_utility あり: idle 込みの総効用 Σ(U−floor) を最大化する部分マッチング。"""
    best = -np.inf
    for m in enumerate_matchings(U, floor):
        best = max(best, sum(U[v, b] - floor for v, b in m))
    return best


def main():
    failures = []

    # 1) 既知の最適解 (対角優位)
    U = np.array([[10.0, 1.0, 1.0],
                  [1.0, 10.0, 1.0],
                  [1.0, 1.0, 10.0]])
    pairs, total = solve_assignment(U)
    if pairs != {0: 0, 1: 1, 2: 2} or abs(total - 30.0) > 1e-9:
        failures.append(f"対角最適を外した: {pairs} total={total}")

    # 2) 需要超過 (5台×3BS): ちょうど3対、idle 2台
    rng = np.random.default_rng(1)
    U = rng.uniform(-80, -40, size=(5, 3))
    pairs, _ = solve_assignment(U)
    if len(pairs) != 3:
        failures.append(f"5×3 で割当数 {len(pairs)} != 3")
    if len(set(pairs.values())) != len(pairs):
        failures.append(f"BS 排他違反: {pairs}")

    # 3) 禁止ペアの回避 (他に選択肢がある限り採らない、残れば idle)
    U = np.array([[10.0, FORBIDDEN_UTILITY - 1.0],
                  [FORBIDDEN_UTILITY - 1.0, FORBIDDEN_UTILITY - 1.0]])
    pairs, total = solve_assignment(U)
    if pairs != {0: 0}:
        failures.append(f"禁止ペアが採用された: {pairs}")

    # 4) min_utility で不採算ペアを idle 化
    U = np.array([[-50.0, -200.0],
                  [-201.0, -60.0]])
    pairs, _ = solve_assignment(U, min_utility=-100.0)
    if pairs != {0: 0, 1: 1}:
        failures.append(f"min_utility 前の基準解が不正: {pairs}")
    pairs, _ = solve_assignment(U, min_utility=-55.0)
    if pairs != {0: 0}:
        failures.append(f"min_utility 未満が採用された: {pairs}")

    # 5) 総当たり照合 (4×4 以下のランダム行列 × 禁止/フロア混在)
    for seed in range(30):
        rng = np.random.default_rng(100 + seed)
        m, n = rng.integers(1, 5), rng.integers(1, 5)
        U = rng.uniform(-100, -40, size=(m, n))
        mask = rng.random(size=(m, n)) < 0.25
        U[mask] = FORBIDDEN_UTILITY - 1.0
        floor = -90.0 if seed % 2 else None
        pairs, total = solve_assignment(U, min_utility=floor)
        if floor is None:
            best_card, best_val = brute_force_no_floor(U)
            if len(pairs) != best_card or total < best_val - 1e-6:
                failures.append(f"seed{seed}: 最適 ({best_card}, {best_val:.3f}) "
                                f"に対し ({len(pairs)}, {total:.3f})")
        else:
            got = total - len(pairs) * floor
            best = brute_force_floor(U, floor)
            if got < best - 1e-6:
                failures.append(f"seed{seed}: idle込み最適 {best:.3f} に対し {got:.3f}")
        for v, b in pairs.items():
            if U[v, b] <= FORBIDDEN_UTILITY or (floor is not None and U[v, b] < floor):
                failures.append(f"seed{seed}: 無効ペア採用 {v}->{b}")
        if len(set(pairs.values())) != len(pairs):
            failures.append(f"seed{seed}: BS 排他違反 {pairs}")
    print("  総当たり照合 30 ケース一致")

    # 6) 空入力
    if solve_assignment(np.zeros((0, 3))) != ({}, 0.0):
        failures.append("空入力の扱いが不正")

    if failures:
        print("FAIL:")
        for f in failures:
            print(f"  - {f}")
        sys.exit(1)
    print("PASS: matching_test 全項目合格")


if __name__ == '__main__':
    main()
