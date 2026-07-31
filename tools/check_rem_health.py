#!/usr/bin/env python3
"""
学習した REM が「知らないことを知っている」かを測る。

REM は自分が観測していない場所についても値を返す。そこで何を返すかは
安全性に直結する:

  楽観的に返すと   実在しないリンクを掴む。802.15.3e は排他接続 (1 BS =
                   1 車) なので、幻の grant は実在する車を締め出す純粋な害
  悲観的に返すと   掴み損ねるだけで、他車の機会は奪わない

LCB (下側信頼限界) は本来そのための道具だが、平均場の事前値が 0 dBm の
ままだと未観測の位置で「極めて強い信号」と評価され、悲観のはずが最も
楽観になる。この転倒を検出する。

  python3 tools/check_rem_health.py sim_results/urban_learn/rem_state
  python3 tools/check_rem_health.py <state_dir> --kappa 1.0 --threshold -68.5
"""
import argparse
import glob
import json
import os
import sys

import numpy as np


def rbf_design(s, num_bases, s_min, s_max, width_m=0.0):
    """RbfBasis と同一の設計行列 (kkf_core に依存せず再現)。"""
    centers = np.linspace(s_min, s_max, num_bases)
    spacing = (s_max - s_min) / (num_bases - 1)
    w = width_m if width_m > 0 else spacing
    d = (s[:, None] - centers[None, :]) / w
    return np.exp(-0.5 * d * d)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('state_dir')
    ap.add_argument('--s-min', type=float, default=0.0)
    ap.add_argument('--s-max', type=float, default=210.0,
                    help='弧長の範囲 [m] (シナリオの kkf_rbf_s_max と揃える)')
    ap.add_argument('--kappa', type=float, default=1.0,
                    help='LCB の保守度 (シナリオの kkf_kappa と揃える)')
    ap.add_argument('--threshold', type=float, default=-68.5,
                    help='接続閾値 [dBm] (シナリオの kkf_idle_lcb_db と揃える)')
    ap.add_argument('--mean-prior-dbm', type=float, default=None,
                    help='平均場の事前値 [dBm]。地図は偏差を学習しているので '
                         '予測に足し戻す (省略時はシナリオ既定の -110)')
    ap.add_argument('--w-min', type=float, default=0.5,
                    help='この観測重み未満を「未観測」とみなす')
    ap.add_argument('--max-phantom-pct', type=float, default=2.0,
                    help='許容する幻リンク率 [%%]。超えたら終了コード 1')
    a = ap.parse_args()

    files = sorted(glob.glob(os.path.join(a.state_dir, 'bs*.npz')))
    if not files:
        sys.exit(f"状態ファイルがありません: {a.state_dir}")
    mu0 = -110.0 if a.mean_prior_dbm is None else a.mean_prior_dbm

    s = np.linspace(a.s_min, a.s_max, 800)
    print(f"事前値 {mu0:.1f} dBm / kappa {a.kappa} / 接続閾値 {a.threshold} dBm\n")
    print(f"{'キー':10s} {'未観測率':>8s} {'未観測LCB中央':>14s} "
          f"{'幻リンク率':>10s} {'観測域μ範囲':>20s}")

    worst = 0.0
    for f in files:
        with np.load(f) as d:
            alpha, P, S2, W = d['alpha'], d['P'], d['S2'], d['W']
            run_count = json.loads(str(d['meta'])).get('run_count', '?')
        Phi = rbf_design(s, len(alpha), a.s_min, a.s_max)
        mu = Phi @ alpha + mu0
        var_kf = np.einsum('ij,jk,ik->i', Phi, P, Phi)
        grid = np.linspace(a.s_min, a.s_max, len(W))
        Wi = np.interp(s, grid, W)
        var_nu = np.interp(s, grid,
                           np.where(W >= a.w_min, S2 / np.maximum(W, 1e-12), 0.0))
        lcb = mu - a.kappa * np.sqrt(np.maximum(var_kf + var_nu, 0.0))

        unobs = Wi < a.w_min
        # 幻リンク = 観測が無いのに「繋がる」と判定される位置 (道路全体に対する率)
        phantom = float((unobs & (lcb > a.threshold)).mean() * 100.0)
        worst = max(worst, phantom)
        key = os.path.basename(f)[2:-4]
        med = float(np.median(lcb[unobs])) if unobs.any() else float('nan')
        obs = ~unobs
        rng = (f"{mu[obs].min():7.1f}..{mu[obs].max():6.1f}" if obs.any()
               else "     (観測なし)")
        print(f"{key:10s} {unobs.mean() * 100:7.1f}% {med:13.1f} "
              f"{phantom:9.1f}% {rng:>20s}")

    print(f"\n学習走行数 {run_count} / 幻リンク率の最大 {worst:.1f}% "
          f"(許容 {a.max_phantom_pct:.1f}%)")
    if worst > a.max_phantom_pct:
        print("NG: 未観測領域を楽観的に評価している。平均場の事前値 "
              "(kkf_mean_prior_dbm) を見直すこと")
        return 1
    print("OK: 知らない場所では繋がらないと判断できている")
    return 0


if __name__ == '__main__':
    sys.exit(main())
