"""
Kriged Kalman Filter (C++ KrigedKalmanFilter.hpp と同一仕様)。
潜在状態 α の KF 更新 (文書 式(2)〜(6), Λ=I) と、更新後残差の
リングバッファに対する時空間分離型指数共分散クリギング (式(7),(8))。
"""
from collections import deque
from dataclasses import dataclass, field

import numpy as np


@dataclass
class KkfParams:
    process_noise_q: float = 1e-4       # Q = q·I·Δt
    initial_state_var: float = 100.0    # P0 = var·I
    sigma_nu: float = 4.0               # 残差場の標準偏差 σ_ν [dB]
    corr_length_s_m: float = 20.0       # 空間相関長 L_s [m]
    corr_length_t_s: float = 5.0        # 時間相関長 L_t [s]
    residual_buffer_size: int = 64
    initial_state_mean: list = field(default_factory=list)  # 空なら零ベクトル


@dataclass
class Observation:
    s: float = 0.0
    t: float = 0.0
    z: float = 0.0
    noise_var: float = 1.0


class KrigedKalmanFilter:
    """責務: 単一RSUの受信品質場 Z(s,t) の時空間推定のみ。"""

    def __init__(self, basis, params: KkfParams):
        self.basis = basis
        self.params = params
        p = basis.dimension()
        if len(params.initial_state_mean) == p:
            self.alpha = np.asarray(params.initial_state_mean, dtype=float).copy()
        else:
            self.alpha = np.zeros(p)
        self.P = np.eye(p) * params.initial_state_var
        self.last_update_t = -1.0
        self.residuals = deque()          # (s, t, value, noise_var)
        self._C = None
        self._kriging_weights = None

    def update(self, t, observations):
        """
        観測集合による1ステップ更新 (時間更新+観測更新+残差登録)。

        Returns:
            今回の観測に対する更新後残差 [(s, residual_db)] (文書 式(9))。
            第2層 (遮蔽トラッカー) の観測入力となる。
        """
        dt = max(0.0, t - self.last_update_t) if self.last_update_t >= 0.0 else 0.0
        self.last_update_t = t
        p = self.basis.dimension()
        self.P = self.P + np.eye(p) * (self.params.process_noise_q * dt)

        fresh_residuals = []
        if observations:
            n = len(observations)
            Phi = np.vstack([self.basis.evaluate(o.s) for o in observations])
            z = np.array([o.z for o in observations])
            R = np.diag([o.noise_var for o in observations])

            S = Phi @ self.P @ Phi.T + R
            K = self.P @ Phi.T @ np.linalg.solve(S, np.eye(n))
            self.alpha = self.alpha + K @ (z - Phi @ self.alpha)
            self.P = (np.eye(p) - K @ Phi) @ self.P

            for o in observations:
                residual = o.z - float(self.basis.evaluate(o.s) @ self.alpha)
                fresh_residuals.append((o.s, residual))
                self.residuals.append((o.s, o.t, residual, o.noise_var))
            while len(self.residuals) > self.params.residual_buffer_size:
                self.residuals.popleft()

        self._refresh_kriging()
        return fresh_residuals

    def predict_at(self, s, t):
        """(s,t) における予測 (mean, variance) — 式(7) + 基底成分の不確かさ。"""
        phi = self.basis.evaluate(s)
        mean = float(phi @ self.alpha)
        variance = float(phi @ self.P @ phi) + self.params.sigma_nu ** 2

        n = len(self.residuals)
        if n > 0:
            c0 = np.array([self._covariance(s, t, r[0], r[1]) for r in self.residuals])
            mean += float(c0 @ self._kriging_weights)
            reduction = float(c0 @ np.linalg.solve(self._C, c0))
            variance -= min(reduction, self.params.sigma_nu ** 2)
            variance = max(variance, 1e-6)
        return mean, variance

    def observation_count(self):
        return len(self.residuals)

    def _covariance(self, s1, t1, s2, t2):
        """時空間分離型指数共分散 (式(8))。"""
        return (self.params.sigma_nu ** 2 *
                np.exp(-abs(s1 - s2) / self.params.corr_length_s_m) *
                np.exp(-abs(t1 - t2) / self.params.corr_length_t_s))

    def _refresh_kriging(self):
        n = len(self.residuals)
        if n == 0:
            return
        res = list(self.residuals)
        C = np.empty((n, n))
        nu = np.empty(n)
        for j in range(n):
            for k in range(n):
                C[j, k] = self._covariance(res[j][0], res[j][1], res[k][0], res[k][1])
            C[j, j] += res[j][3]  # ナゲット (観測雑音)
            nu[j] = res[j][2]
        self._C = C
        self._kriging_weights = np.linalg.solve(C, nu)
