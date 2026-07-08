"""
第2層: 残差マップに基づく遮蔽トラッカー (文書 4章)。

責務: KKF更新後残差からの遮蔽領域の即時検出 (式(9))、遮蔽帯中心の
等速カルマンフィルタ追跡 (式(10))、未来遮蔽区間 B(t+Δ) の外挿のみ。
第1層への還流 (式(11)) と第3層ペナルティ (式(12)) は呼び出し側が行う。
numpy のみ依存の純粋モジュール。
"""
from dataclasses import dataclass

import numpy as np


@dataclass
class TrackerParams:
    threshold_db: float = -10.0        # 遮蔽判定の残差しきい値 ν_th (式(9))
    cluster_gap_m: float = 25.0        # 1次元クラスタリングの分割ギャップ
    zone_halfwidth_min_m: float = 8.0  # 遮蔽帯半幅の下限
    gate_m: float = 40.0               # トラック関連付けゲート
    accel_psd: float = 4.0             # 等速モデルのプロセス雑音 PSD [m²/s³]
    meas_noise_var: float = 9.0        # クラスタ中心の観測分散 [m²]
    track_timeout_s: float = 1.5       # 未観測でトラックを破棄するまでの時間
    confirm_hits: int = 3              # ゾーン出力に必要な最小ヒット数


class BlockageTrack:
    """単一遮蔽帯の等速KF (状態 [中心 c, 速度 v_b]、式(10))。"""

    def __init__(self, t, center, halfwidth, params: TrackerParams):
        self.params = params
        self.x = np.array([center, 0.0])
        self.P = np.diag([params.meas_noise_var, 400.0])  # 速度は無情報で開始
        self.last_t = t
        self.hits = 1
        self.halfwidth = max(halfwidth, params.zone_halfwidth_min_m)

    def _fq(self, dt):
        F = np.array([[1.0, dt], [0.0, 1.0]])
        q = self.params.accel_psd
        Q = q * np.array([[dt ** 3 / 3.0, dt ** 2 / 2.0],
                          [dt ** 2 / 2.0, dt]])
        return F, Q

    def predict_center(self, t):
        dt = max(0.0, t - self.last_t)
        return float(self.x[0] + self.x[1] * dt)

    def update(self, t, center_meas, halfwidth_meas):
        dt = max(0.0, t - self.last_t)
        F, Q = self._fq(dt)
        x_pred = F @ self.x
        P_pred = F @ self.P @ F.T + Q

        H = np.array([[1.0, 0.0]])
        S = float(H @ P_pred @ H.T) + self.params.meas_noise_var
        K = (P_pred @ H.T / S).ravel()
        self.x = x_pred + K * (center_meas - float(H @ x_pred))
        self.P = (np.eye(2) - np.outer(K, H)) @ P_pred
        self.last_t = t
        self.hits += 1
        # 半幅は緩やかに追従 (急拡大の誤検出を抑える)
        self.halfwidth = 0.7 * self.halfwidth + 0.3 * max(
            halfwidth_meas, self.params.zone_halfwidth_min_m)

    @property
    def velocity(self):
        return float(self.x[1])

    def is_confirmed(self):
        return self.hits >= self.params.confirm_hits

    def is_stale(self, t):
        return (t - self.last_t) > self.params.track_timeout_s


class BlockageTracker:
    """残差サンプル列から複数遮蔽帯を検出・追跡する。"""

    def __init__(self, params: TrackerParams = None):
        self.params = params or TrackerParams()
        self.tracks = []

    def ingest(self, t, residual_samples):
        """
        Args:
            residual_samples: [(s, residual_db)] — 第1層の更新後残差 (式(9))
        """
        p = self.params
        blocked = sorted(s for s, r in residual_samples if r < p.threshold_db)

        # 1次元クラスタリング (ギャップ分割)
        clusters = []
        if blocked:
            start = prev = blocked[0]
            for s in blocked[1:]:
                if s - prev > p.cluster_gap_m:
                    clusters.append((start, prev))
                    start = s
                prev = s
            clusters.append((start, prev))

        # トラック関連付け (最近傍ゲート)
        for lo, hi in clusters:
            center = 0.5 * (lo + hi)
            halfwidth = 0.5 * (hi - lo)
            best, best_d = None, p.gate_m
            for track in self.tracks:
                d = abs(track.predict_center(t) - center)
                if d < best_d:
                    best, best_d = track, d
            if best is not None:
                best.update(t, center, halfwidth)
            else:
                self.tracks.append(BlockageTrack(t, center, halfwidth, p))

        self.tracks = [tr for tr in self.tracks if not tr.is_stale(t)]

    def zones(self, t_future):
        """未来時刻の遮蔽区間 B(t_future) を [(s_min, s_max)] で返す (確定トラックのみ)。"""
        result = []
        for tr in self.tracks:
            if tr.is_confirmed():
                c = tr.predict_center(t_future)
                result.append((c - tr.halfwidth, c + tr.halfwidth))
        return result

    def is_blocked(self, s, t_future):
        return any(lo <= s <= hi for lo, hi in self.zones(t_future))
