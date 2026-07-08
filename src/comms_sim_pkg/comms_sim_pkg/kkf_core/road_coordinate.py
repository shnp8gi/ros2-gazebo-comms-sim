"""経路折れ線に対する世界座標⇔弧長座標の相互変換 (C++ RoadCoordinate.hpp と同一仕様)。"""
import numpy as np


class RoadCoordinate:
    """責務: 折れ線経路の弧長座標変換のみ。通信・時刻の知識を持たない純粋幾何。"""

    def __init__(self, points):
        self.points = np.asarray(points, dtype=float)
        if self.points.ndim != 2 or self.points.shape[1] != 3:
            raise ValueError("points must be (N, 3)")
        seg = np.diff(self.points, axis=0)
        seg_len = np.linalg.norm(seg, axis=1)
        self.cum_lengths = np.concatenate([[0.0], np.cumsum(seg_len)])

    def is_valid(self):
        return len(self.points) >= 2

    def total_length(self):
        return float(self.cum_lengths[-1])

    def project(self, world_pos):
        """世界座標を経路へ射影し弧長 s を返す。"""
        p = np.asarray(world_pos, dtype=float)
        best_s, best_d2 = 0.0, np.inf
        for i in range(len(self.points) - 1):
            a, b = self.points[i], self.points[i + 1]
            ab = b - a
            len_sq = float(ab @ ab)
            t = float((p - a) @ ab) / len_sq if len_sq > 1e-12 else 0.0
            t = min(max(t, 0.0), 1.0)
            proj = a + t * ab
            d2 = float((p - proj) @ (p - proj))
            if d2 < best_d2:
                best_d2 = d2
                best_s = self.cum_lengths[i] + t * np.sqrt(len_sq)
        return float(best_s)

    def position_at(self, s):
        """弧長 s の経路上の世界座標 (端点外はクランプ)。"""
        if s <= 0.0:
            return self.points[0].copy()
        if s >= self.total_length():
            return self.points[-1].copy()
        idx = int(np.searchsorted(self.cum_lengths, s, side='right')) - 1
        idx = min(idx, len(self.points) - 2)
        seg_len = self.cum_lengths[idx + 1] - self.cum_lengths[idx]
        t = (s - self.cum_lengths[idx]) / seg_len if seg_len > 1e-12 else 0.0
        return self.points[idx] + t * (self.points[idx + 1] - self.points[idx])

    def tangent_yaw_at(self, s):
        """弧長 s における経路接線方向の yaw [rad] (車両姿勢の推定に使用)。"""
        idx = int(np.searchsorted(self.cum_lengths, min(max(s, 0.0), self.total_length()),
                                  side='right')) - 1
        idx = min(max(idx, 0), len(self.points) - 2)
        d = self.points[idx + 1] - self.points[idx]
        return float(np.arctan2(d[1], d[0]))
