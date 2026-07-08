"""ビタビDPによるハンドオーバー計画 (C++ HandoverPlanner.hpp と同一仕様、文書 式(13))。"""
import numpy as np


def solve_handover_plan(lcb_series, stage_dt_s, switch_cost, initial_state):
    """
    max Σ_k γ̃[a(k)][k]·Δt − λ Σ_k 1[a(k) ≠ a(k−1)] を厳密に解く。

    Args:
        lcb_series: (A, K) 行列。lcb_series[state][stage] = γ̃ (LCB)
        stage_dt_s: ステージ幅 Δt [s]
        switch_cost: 切替コスト λ
        initial_state: 計画開始時点の接続状態 (-1 = 未接続)
    Returns:
        (assignments: list[int] 長さK, total_value: float) / 空入力は ([], 0.0)
    """
    lcb = np.asarray(lcb_series, dtype=float)
    if lcb.size == 0:
        return [], 0.0
    A, K = lcb.shape

    backptr = np.full((K, A), -1, dtype=int)
    value = lcb[:, 0] * stage_dt_s
    if initial_state >= 0:
        value = value - np.where(np.arange(A) != initial_state, switch_cost, 0.0)

    for k in range(1, K):
        # trans[prev, a] = value[prev] - switch_cost·1[a≠prev]
        trans = value[:, None] - switch_cost * (1.0 - np.eye(A))
        backptr[k] = np.argmax(trans, axis=0)
        value = trans[backptr[k], np.arange(A)] + lcb[:, k] * stage_dt_s

    best_last = int(np.argmax(value))
    assignments = [best_last] * K
    for k in range(K - 1, 0, -1):
        assignments[k - 1] = int(backptr[k][assignments[k]])
    return assignments, float(value[best_last])
