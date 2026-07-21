"""基底関数 φ(s) (C++ BasisFunction.hpp と同一仕様)。"""
import numpy as np


class ConstantBasis:
    """
    定数基底 φ(s) = [1]。α = [バイアス] に対応する。
    事前地図 (決定論RSSIプロファイル) を平均関数とする偏差学習モードで使う
    (偏差場のトレンドは距離依存を持たないため定数のみ)。
    """

    def dimension(self):
        return 1

    def evaluate(self, s):
        return np.array([1.0])


class LogDistanceBasis:
    """
    対数距離基底 φ(s) = [1, log10(max(d(s), d_min))]^T。
    α = [基準受信レベル, 距離減衰の傾き] に対応する (文書 式(1))。
    """

    def __init__(self, road, rsu_position, min_distance_m=1.0):
        self.road = road
        self.rsu_position = np.asarray(rsu_position, dtype=float)
        self.min_distance_m = float(min_distance_m)

    def dimension(self):
        return 2

    def evaluate(self, s):
        d = float(np.linalg.norm(self.road.position_at(s) - self.rsu_position))
        d = max(d, self.min_distance_m)
        return np.array([1.0, np.log10(d)])
