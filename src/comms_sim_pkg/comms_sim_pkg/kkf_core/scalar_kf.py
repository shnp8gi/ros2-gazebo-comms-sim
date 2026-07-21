"""
ペア単位スカラーKalmanフィルタ (時系列予測ハンドオーバー・ベースライン用)。

先行研究ファミリ「単一リンクRSS時系列のKalman/AR外挿による予測HO」の
コア機構の再実装。状態 x = [RSSIレベル (dBm), 変化率 (dB/s)] の
等速 (constant-velocity) モデルで、空間構造・遮蔽追跡を一切持たない。
KKF (kkf.py) との差分が「時間のみの予測」対「時空間予測」の寄与を分離する。
"""
from dataclasses import dataclass

import numpy as np


@dataclass
class ScalarKfParams:
    process_noise_q: float = 25.0    # 加速度白色雑音PSD q [dB^2/s^3]
    initial_level_var: float = 400.0  # 初回観測時のレベル分散 [dB^2]
    initial_rate_var: float = 100.0   # 初回観測時の変化率分散 [(dB/s)^2]
    prior_mean_dbm: float = -120.0    # 無観測ペアの事前平均 [dBm]
    prior_var: float = 400.0          # 無観測ペアの事前分散 [dB^2]


class ScalarRssiKF:
    """責務: 単一 (アンテナ, BS) ペアの RSSI 時系列の推定・外挿のみ。"""

    def __init__(self, params: ScalarKfParams):
        self.params = params
        self.x = np.zeros(2)          # [level_dbm, rate_db_per_s]
        self.P = np.zeros((2, 2))
        self.last_t = -1.0

    def initialized(self):
        return self.last_t >= 0.0

    def update(self, t, z, noise_var):
        """時刻 t の観測 z [dBm] (雑音分散 noise_var) による1ステップ更新。"""
        if not self.initialized():
            self.x = np.array([z, 0.0])
            self.P = np.diag([self.params.initial_level_var,
                              self.params.initial_rate_var])
            self.last_t = t
            return
        dt = max(0.0, t - self.last_t)
        self.x, self.P = self._propagate(dt)
        self.last_t = max(t, self.last_t)

        # 観測更新 (H = [1, 0])
        s = self.P[0, 0] + noise_var
        k = self.P[:, 0] / s
        self.x = self.x + k * (z - self.x[0])
        self.P = self.P - np.outer(k, self.P[0, :])

    def predict_at(self, t):
        """時刻 t への外挿 (mean, variance)。無観測なら事前分布を返す。"""
        if not self.initialized():
            return self.params.prior_mean_dbm, self.params.prior_var
        dt = max(0.0, t - self.last_t)
        x, P = self._propagate(dt)
        return float(x[0]), max(float(P[0, 0]), 1e-6)

    def _propagate(self, dt):
        """等速モデルの時間発展 (連続時間白色加速度雑音の厳密離散化)。"""
        F = np.array([[1.0, dt], [0.0, 1.0]])
        q = self.params.process_noise_q
        Q = q * np.array([[dt ** 3 / 3.0, dt ** 2 / 2.0],
                          [dt ** 2 / 2.0, dt]])
        return F @ self.x, F @ self.P @ F.T + Q
