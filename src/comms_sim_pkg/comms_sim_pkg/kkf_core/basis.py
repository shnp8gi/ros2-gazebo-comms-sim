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

    def config(self):
        """RemStore の basis ハッシュ用の正規化設定。"""
        return {'type': 'constant'}


class RbfBasis:
    """
    ガウスRBF基底 φ_i(s) = exp(−(s−c_i)²/2w²)。
    中心 c_i は [s_min, s_max] に等間隔 num_bases 個。幅 w の既定は中心間隔
    (隣接基底が重なり、指向性ウィンドウのような s に非単調な平均場を表現できる。
    LogDistanceBasis 単独では表現不能だった構造への回答 = 本番仕様 §5.1)。
    """

    def __init__(self, s_min, s_max, num_bases=20, width_m=0.0):
        num_bases = int(num_bases)
        if num_bases < 2:
            raise ValueError(f"RbfBasis: num_bases は2以上が必要 ({num_bases})")
        if not s_max > s_min:
            raise ValueError(f"RbfBasis: s_max > s_min が必要 ({s_min}, {s_max})")
        self.centers = np.linspace(float(s_min), float(s_max), num_bases)
        spacing = (float(s_max) - float(s_min)) / (num_bases - 1)
        self.width_m = float(width_m) if width_m > 0 else spacing

    def dimension(self):
        return len(self.centers)

    def evaluate(self, s):
        d = (float(s) - self.centers) / self.width_m
        return np.exp(-0.5 * d * d)

    def config(self):
        """RemStore の basis ハッシュ用の正規化設定。"""
        return {'type': 'rbf',
                's_min': float(self.centers[0]),
                's_max': float(self.centers[-1]),
                'num_bases': int(len(self.centers)),
                'width_m': float(self.width_m)}


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

    def config(self):
        """RemStore の basis ハッシュ用の正規化設定。"""
        return {'type': 'log_distance',
                'rsu_position': [round(float(v), 3) for v in self.rsu_position],
                'min_distance_m': float(self.min_distance_m)}
