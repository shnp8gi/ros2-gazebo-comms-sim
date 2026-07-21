"""
A3イベント型ハンドオーバー・ベースライン (3GPP TS 38.331 の測定イベント A3 の翻案)。

先行研究ファミリ「hysteresis + Time-to-Trigger による測定イベント駆動HO」の
コア機構の再実装。予測を一切持たず、L3フィルタ済み測定値の現在値比較のみで
切替を判断する (比較表: 知識=測定のみ / 予測なし / 空間構造なし / 遮蔽追跡なし)。

P2P制約 (802.15.3e) への翻案: 近傍セルの常時測定は不可能なので、周期的に
MEASURE スロットを挿入して近傍ペアを1つずつラウンドロビンでプローブする。
プローブ中はサービングリンクのデータが止まり、復帰時にペアネット再確立を
要する — この構造的コストの定量化が本ベースラインの存在意義である。
"""
from dataclasses import dataclass

import numpy as np


@dataclass
class A3Params:
    hysteresis_db: float = 3.0        # A3 ヒステリシス [dB]
    time_to_trigger_s: float = 0.1    # TTT: 条件連続成立の要求時間 [s]
    l3_filter_beta: float = 0.5       # L3フィルタ EMA 係数 (旧値の重み)
    measure_period_s: float = 0.4     # 近傍プローブの開始周期 [s]
    measure_duration_s: float = 0.06  # プローブ窓長 [s] (レポート周期以上必要)
    estimate_timeout_s: float = 4.0   # これより古い近傍推定は A3 判定に使わない [s]


class A3Controller:
    """責務: A3イベント則によるサービングペア決定とプローブ窓の管理のみ。

    状態機械:
      SCAN     — 全ペアを1巡プローブし、最良ペアにアタッチ
      TRACKING — サービングで DATA、周期的に近傍を1ペアずつ MEASURE、
                 A3条件 (nbr > serving + hyst が TTT 継続) で切替
    """

    def __init__(self, params: A3Params, num_ant, num_bs):
        self.params = params
        self.num_ant = num_ant
        self.num_bs = num_bs
        self.num_pairs = num_ant * num_bs

        self.rsrp = np.full(self.num_pairs, -np.inf)   # L3フィルタ済み推定 [dBm]
        self.last_meas_t = np.full(self.num_pairs, -np.inf)

        self.serving = -1              # ペアindex (ant*num_bs+bs)、-1=未アタッチ
        self.probe = None              # (t_start, t_end, pair) | None
        self.next_probe_t = 0.0
        self.scan_idx = 0              # SCAN: 次にプローブするペア
        self.rr_idx = 0                # TRACKING: 近傍ラウンドロビン位置
        self.ttt_pair = -1             # A3条件成立中の候補ペア
        self.ttt_since = 0.0
        self.handover_count = 0

    # --- 観測受信 ---
    def ingest(self, t, entries):
        """測定エントリ [(ant, bs, rssi)] を取り込み、状態機械を進める。"""
        for ant, bs, rssi in entries:
            p = ant * self.num_bs + bs
            beta = self.params.l3_filter_beta
            self.rsrp[p] = (beta * self.rsrp[p] + (1.0 - beta) * rssi
                            if np.isfinite(self.rsrp[p]) else rssi)
            self.last_meas_t[p] = t

        if self.serving < 0:
            self._step_scan(t)
        else:
            self._step_tracking(t)

    def _step_scan(self, t):
        if self.probe is not None and t < self.probe[1]:
            return
        if self.probe is not None:
            self.scan_idx += 1
        if self.scan_idx >= self.num_pairs:
            # スキャン完了: 測定できたペアの最良にアタッチ
            if np.any(np.isfinite(self.rsrp)):
                self.serving = int(np.nanargmax(
                    np.where(np.isfinite(self.rsrp), self.rsrp, -np.inf)))
                self.probe = None
                self.next_probe_t = t + self.params.measure_period_s
                return
            self.scan_idx = 0  # 1つも測れなければ再スキャン
        self.probe = (t, t + self.params.measure_duration_s, self.scan_idx)

    def _step_tracking(self, t):
        # プローブ窓の終了処理
        if self.probe is not None and t >= self.probe[1]:
            self.probe = None
        # プローブ開始 (近傍をラウンドロビン)
        if self.probe is None and t >= self.next_probe_t:
            pair = self._next_neighbor()
            if pair >= 0:
                self.probe = (t, t + self.params.measure_duration_s, pair)
                self.next_probe_t = t + self.params.measure_period_s

        self._evaluate_a3(t)

    def _next_neighbor(self):
        for _ in range(self.num_pairs):
            self.rr_idx = (self.rr_idx + 1) % self.num_pairs
            if self.rr_idx != self.serving:
                return self.rr_idx
        return -1

    def _evaluate_a3(self, t):
        """A3: 最良近傍が serving + hyst を TTT の間 上回り続けたら切替。"""
        fresh = ((t - self.last_meas_t) <= self.params.estimate_timeout_s) \
            & np.isfinite(self.rsrp)
        fresh[self.serving] = False
        if not np.any(fresh) or not np.isfinite(self.rsrp[self.serving]):
            self.ttt_pair = -1
            return
        cand = int(np.argmax(np.where(fresh, self.rsrp, -np.inf)))
        entered = (self.rsrp[cand] >
                   self.rsrp[self.serving] + self.params.hysteresis_db)
        if not entered:
            self.ttt_pair = -1
            return
        if cand != self.ttt_pair:
            self.ttt_pair = cand
            self.ttt_since = t
            return
        if t - self.ttt_since >= self.params.time_to_trigger_s:
            self.serving = cand
            self.ttt_pair = -1
            self.handover_count += 1
            # 切替直後のプローブは通常周期に任せる (probe中の切替はprobe破棄)
            self.probe = None
            self.next_probe_t = t + self.params.measure_period_s

    # --- スケジュール生成 ---
    def plan(self, t):
        """現時点の割当計画 [(t_start, pair, is_measure)] を返す。

        再計画周期ごとに呼ばれる前提の逐次計画 (プローブは ingest 側で開始
        されるため、ここでは現在の状態を写すだけ)。
        """
        if self.serving < 0:
            if self.probe is None:
                return []
            return [(t, self.probe[2], True)]
        if self.probe is not None and t < self.probe[1]:
            return [(t, self.probe[2], True),
                    (self.probe[1], self.serving, False)]
        return [(t, self.serving, False)]

    def status(self):
        n_fresh = int(np.sum(np.isfinite(self.rsrp)))
        return (f"serving={self.serving} ho={self.handover_count} "
                f"meas={n_fresh}/{self.num_pairs}")
