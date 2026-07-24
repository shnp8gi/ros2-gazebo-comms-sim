"""
位置依存の偶然的分散 σ_ν²(s) の自己組織化マップ (本番仕様 §5.2)。

走行ごとにランダムな動的遮蔽は平均場では学習できないが、「その場所の
変わりやすさ (分散)」は場所に持続する。KKF 更新後残差の二乗を格子上の
割引なし十分統計量 (S2, W) に累積し、σ_ν²(s) = S2/W を予測分散にのみ
加算することで、遮蔽リスクマップを人手なしに形成する。

単純 EMA を使わないのは、確率的な遮蔽イベント (発生率 数%〜17%) を
イベント間隔の間に忘れてしまうため。忘却は走行境界の discount() のみ
(γ ≈ 0.997 の遅い割引で交通の経年変化に追従する)。
"""
from dataclasses import dataclass

import numpy as np


@dataclass
class VarianceMapParams:
    s_min: float = 0.0
    s_max: float = 200.0
    grid_m: float = 2.0              # 格子間隔
    kernel_width_m: float = 0.0      # 残差を配るガウスカーネル幅 (0 = grid_m)
    kernel_cutoff_sigma: float = 3.0
    prior_var_db2: float = 0.0       # 重み不足の格子が返す事前分散 [dB²]
    min_weight: float = 0.5          # これ未満の W は「未観測」扱い


class AleatoricVarianceMap:
    """責務: σ_ν²(s) の保持・更新・照会・割引のみ。KKF の状態には触れない。

    予測分散への合成 (var_kkf + σ_ν²(s)) は呼び出し側 (KkfMapPredictor) の
    責務であり、本クラスは kkf.py を知らない。Kalman ゲイン・観測ノイズには
    決して使わないこと (平均場学習を殺すため。本番仕様 §5.2)。
    """

    def __init__(self, params: VarianceMapParams):
        if not params.s_max > params.s_min:
            raise ValueError(f"VarianceMap: s_max > s_min が必要 "
                             f"({params.s_min}, {params.s_max})")
        self.params = params
        n = max(2, int(round((params.s_max - params.s_min) / params.grid_m)) + 1)
        self.grid = np.linspace(params.s_min, params.s_max, n)
        self.S2 = np.zeros(n)
        self.W = np.zeros(n)
        self._h = (params.kernel_width_m if params.kernel_width_m > 0
                   else params.grid_m)
        self._var_cache = None

    def update(self, s, residual_db):
        """更新後残差1点を累積: S2[g] += k(s,g)·r², W[g] += k(s,g)。"""
        cutoff = self.params.kernel_cutoff_sigma * self._h
        lo = int(np.searchsorted(self.grid, s - cutoff, side='left'))
        hi = int(np.searchsorted(self.grid, s + cutoff, side='right'))
        if lo >= hi:
            return
        d = (self.grid[lo:hi] - float(s)) / self._h
        k = np.exp(-0.5 * d * d)
        self.S2[lo:hi] += k * float(residual_db) ** 2
        self.W[lo:hi] += k
        self._var_cache = None

    def update_batch(self, residuals):
        """residuals: [(s, residual_db)] (KrigedKalmanFilter.update の戻り値)。"""
        for s, r in residuals:
            self.update(s, r)

    def _var_grid(self):
        if self._var_cache is None:
            var = np.full(len(self.grid), self.params.prior_var_db2)
            ok = self.W >= self.params.min_weight
            var[ok] = self.S2[ok] / self.W[ok]
            self._var_cache = var
        return self._var_cache

    def query(self, s):
        """σ_ν²(s) [dB²]。未観測域は prior_var_db2。"""
        return float(np.interp(float(s), self.grid, self._var_grid()))

    def query_batch(self, s_arr):
        return np.interp(np.asarray(s_arr, dtype=float), self.grid,
                         self._var_grid())

    def discount(self, gamma):
        """走行境界の忘却 (carryover 時のみ呼ぶ)。σ_ν²=S2/W は不変で重みが減る
        = 新しい走行の証拠が効きやすくなる。"""
        self.S2 *= float(gamma)
        self.W *= float(gamma)
        self._var_cache = None

    def contrast(self, q_hi=0.9):
        """収束確認用: 観測済み格子の分散の上位分位点/中央値 比。
        遮蔽リスク帯が形成されると 1 から離れて飽和する (学習曲線の指標)。"""
        ok = self.W >= self.params.min_weight
        if not np.any(ok):
            return 1.0
        var = self.S2[ok] / self.W[ok]
        med = float(np.median(var))
        if med <= 1e-12:
            return 1.0
        return float(np.quantile(var, q_hi)) / med

    def load_state(self, S2, W):
        """RemStore からの復元 (形状不一致は設定変更なのでエラー)。"""
        S2 = np.asarray(S2, dtype=float)
        W = np.asarray(W, dtype=float)
        if S2.shape != self.S2.shape or W.shape != self.W.shape:
            raise ValueError(
                f"VarianceMap: 格子形状不一致 (state {S2.shape}/{W.shape}, "
                f"map {self.S2.shape}) — grid 設定が保存時と異なる")
        self.S2 = S2.copy()
        self.W = W.copy()
        self._var_cache = None
