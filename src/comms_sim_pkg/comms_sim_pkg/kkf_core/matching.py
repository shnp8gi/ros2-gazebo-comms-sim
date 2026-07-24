"""
P2P 排他制約下のリスク調整効用マッチング (本番仕様 §6)。

各再計画時刻に効用行列 U[i][j] = μ_ij − κ·σ_ij (車 i × BS j) を作り、
ハンガリアン法で割り当てる。余剰の車は自然に idle となり、逐次優先度DP の
占有マスク (MASKED_LCB) 起因の構造的飢餓が原理的に消える。

責務: 効用行列 → 割当のみ。効用の作り方 (LCB)・切替ヒステリシス・時系列は
呼び出し側 (スケジューラ) の責務。
"""
import numpy as np
from scipy.optimize import linear_sum_assignment

# 割当禁止 (CONNECTED 不能・未進入等) を表す番兵。これ以下の効用は採用しない。
FORBIDDEN_UTILITY = -1.0e9


def solve_assignment(utility, min_utility=None):
    """
    効用総和最大の P2P 割当を解く。

    min_utility は「idle の効用」: これ未満のペアを掴むくらいなら車を idle に
    する。事後フィルタではなく仮想 idle 列 (効用 = min_utility) を車ごとに
    追加して解くため、負効用 (dBm) でも厳密に最適 (掴んだ方が良いか idle が
    良いかを含めて総和最大)。None のときは割当可能な限り掴む (idle なし)。

    Args:
        utility: (n_veh, n_bs) 行列。禁止ペアは FORBIDDEN_UTILITY 以下にする。
        min_utility: idle の効用しきい値。None = idle オプションなし。
    Returns:
        (pairs, total): pairs = {veh_idx: bs_idx} (採用ペアのみ)、
        total = 採用ペアの効用和。空入力は ({}, 0.0)。
    """
    U = np.asarray(utility, dtype=float)
    if U.ndim != 2 or U.size == 0:
        return {}, 0.0
    m, n = U.shape

    if min_utility is None:
        # 禁止ペアは十分小さい番兵なので、ハンガリアン法は「禁止を含む対が
        # 最少 = 実行可能割当が最大」の解を選ぶ。形式上残った禁止対を落とす。
        rows, cols = linear_sum_assignment(U, maximize=True)
        pairs = {int(r): int(c) for r, c in zip(rows, cols)
                 if U[r, c] > FORBIDDEN_UTILITY}
    else:
        # 車ごとに idle 列 (効用 min_utility) を持つ拡大行列。idle 列は m 本
        # あるため全車 idle も表現でき、「floor 未満の実ペアを強制される」
        # ことがない。
        floor = float(min_utility)
        aug = np.concatenate([U, np.full((m, m), floor)], axis=1)
        rows, cols = linear_sum_assignment(aug, maximize=True)
        pairs = {int(r): int(c) for r, c in zip(rows, cols)
                 if c < n and U[r, c] > FORBIDDEN_UTILITY and U[r, c] >= floor}

    total = float(sum(U[r, c] for r, c in pairs.items()))
    return pairs, total
