#!/usr/bin/env python3
"""
学習した REM (電波環境地図) の可視化。

保存された状態 (rem_state/bs<key>.npz) から、地図が何を学習したかを描く:

  1. 平均場 μ(s)      RBF基底 × α。学習した「その位置でのRSSI」
  2. リスク σ_ν²(s)   S2/W。遮蔽の起きやすさ (自己組織化した偶然的分散)
  3. 予測分散         φᵀPφ + σ_ν²。どこが不確かか
  4. 観測重み W(s)    どこを実際に観測できたか (学習の裏付けの濃さ)

方向別に地図が分かれている場合 (bs<j>_p / bs<j>_m) は上り・下りを並べる。

  python3 tools/plot_rem.py sim_results/urban_learn/rem_state
  python3 tools/plot_rem.py sim_results/urban_learn/rem_state --out figures/rem.png
  python3 tools/plot_rem.py sim_results/urban_learn/rem_state_snapshots/after_run_10
"""
import argparse
import glob
import json
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'comms_sim_pkg', 'comms_sim_pkg'))


def load_states(state_dir):
    """rem_state ディレクトリから (キー, 状態) を読む。"""
    out = []
    for f in sorted(glob.glob(os.path.join(state_dir, 'bs*.npz'))):
        key = os.path.basename(f)[2:-4]
        with np.load(f) as d:
            out.append((key, {
                'alpha': d['alpha'], 'P': d['P'], 'S2': d['S2'], 'W': d['W'],
                'meta': json.loads(str(d['meta'])),
            }))
    return out


def rbf_design(s_grid, num_bases, s_min, s_max, width_m=0.0):
    """RbfBasis と同一の設計行列 Φ (kkf_core に依存せず再現)。"""
    centers = np.linspace(s_min, s_max, num_bases)
    spacing = (s_max - s_min) / (num_bases - 1)
    w = width_m if width_m > 0 else spacing
    d = (s_grid[:, None] - centers[None, :]) / w
    return np.exp(-0.5 * d * d)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('state_dir', help='rem_state ディレクトリ')
    ap.add_argument('--out', default=None, help='出力PNG (既定: <state_dir>/rem_map.png)')
    ap.add_argument('--s-min', type=float, default=0.0)
    ap.add_argument('--s-max', type=float, default=230.0,
                    help='弧長の範囲 [m] (シナリオの kkf_rbf_s_max と揃える)')
    ap.add_argument('--x-offset', type=float, default=-115.0,
                    help='弧長 0 に対応する world x [m] (RSU位置を重ねるため)')
    ap.add_argument('--rsu-x', type=float, nargs='*', default=[-15, -5, 5, 15],
                    help='RSU の world x [m] (縦線で表示)')
    ap.add_argument('--mu-ylim', type=float, nargs=2, default=[-130.0, -40.0],
                    help='平均場の表示範囲 [dBm]。観測域外では端点基底の悪条件で '
                         'μ が発散するため、既定で通信が成立する範囲に切る')
    ap.add_argument('--w-min', type=float, default=0.5,
                    help='この観測重み未満の領域を「未観測」として網掛けする')
    a = ap.parse_args()

    states = load_states(a.state_dir)
    if not states:
        sys.exit(f"状態ファイルが見つかりません: {a.state_dir}")

    n_bases = len(states[0][1]['alpha'])
    s = np.linspace(a.s_min, a.s_max, 600)
    Phi = rbf_design(s, n_bases, a.s_min, a.s_max)
    x_world = s + a.x_offset

    # 方向別キー (0_p / 0_m) なら方向で列を分ける
    split = any('_' in k for k, _ in states)
    if split:
        cols = [('p', 'direction +1 (near lane)'), ('m', 'direction -1 (far lane)')]
        groups = {c: [(k, v) for k, v in states if k.endswith('_' + c)]
                  for c, _ in cols}
    else:
        cols = [('', 'all vehicles')]
        groups = {'': states}

    run_count = states[0][1]['meta'].get('run_count', '?')
    fig, axes = plt.subplots(4, len(cols), figsize=(7.0 * len(cols), 12.0),
                             squeeze=False, sharex=True)

    for ci, (suffix, title) in enumerate(cols):
        items = groups[suffix]
        ax_mu, ax_risk, ax_var, ax_w = (axes[r][ci] for r in range(4))
        for key, st in items:
            bs = key.split('_')[0]
            mu = Phi @ st['alpha']
            grid_w = np.linspace(a.s_min, a.s_max, len(st['W']))
            ok = st['W'] >= 0.5
            var_nu = np.where(ok, st['S2'] / np.maximum(st['W'], 1e-12), 0.0)
            var_nu_i = np.interp(s, grid_w, var_nu)
            var_kf = np.einsum('ij,jk,ik->i', Phi, st['P'], Phi)

            ax_mu.plot(x_world, mu, label=f'RSU{bs}')
            ax_risk.plot(x_world, var_nu_i, label=f'RSU{bs}')
            ax_var.plot(x_world, var_kf + var_nu_i, label=f'RSU{bs}')
            ax_w.plot(np.linspace(a.s_min, a.s_max, len(st['W'])) + a.x_offset,
                      st['W'], label=f'RSU{bs}')

        for ax in (ax_mu, ax_risk, ax_var, ax_w):
            for rx in a.rsu_x:
                ax.axvline(rx, color='gray', ls=':', lw=0.8)
            ax.grid(alpha=0.3)
        # 観測が薄い領域を網掛け (そこの μ は外挿であり信用しない)
        wsum = np.zeros(len(items[0][1]['W'])) if items else np.zeros(1)
        for _k, st in items:
            wsum = wsum + st['W']
        gx = np.linspace(a.s_min, a.s_max, len(wsum)) + a.x_offset
        thin = wsum < a.w_min
        for ax in (ax_mu, ax_risk, ax_var, ax_w):
            ax.fill_between(gx, *ax.get_ylim(), where=thin, color='gray',
                            alpha=0.12, step='mid', lw=0)
        ax_mu.set_ylim(*a.mu_ylim)
        ax_mu.set_title(f'{title}   (learned from {run_count} runs)')
        ax_mu.set_ylabel('mean field  mu(s) [dBm]')
        ax_risk.set_ylabel('risk  sigma_nu^2(s) [dB^2]')
        ax_var.set_ylabel('predictive variance [dB^2]')
        ax_w.set_ylabel('observation weight W(s)')
        ax_w.set_xlabel('world x [m]   (dotted = RSU)')
        if ci == 0:
            ax_mu.legend(fontsize=8, ncol=2)

    fig.suptitle('Learned REM (radio environment map)', y=0.995)
    fig.tight_layout()
    out = a.out or os.path.join(a.state_dir, 'rem_map.png')
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    fig.savefig(out, dpi=130, bbox_inches='tight')
    print(f"wrote {out}")

    # 数値サマリ (図と併せて読む)
    print("\nキー   run  |μ|max   σ_ν²(中央)  σ_ν²(90%)  コントラスト  W合計")
    for key, st in states:
        ok = st['W'] >= 0.5
        v = st['S2'][ok] / np.maximum(st['W'][ok], 1e-12) if ok.any() else np.array([0.0])
        med = max(float(np.median(v)), 1e-12)
        print(f"{key:5s} {st['meta'].get('run_count','?'):>4} "
              f"{np.abs(st['alpha']).max():7.1f} {med:11.2f} "
              f"{float(np.quantile(v, 0.9)):10.2f} {float(np.quantile(v,0.9))/med:12.2f} "
              f"{st['W'].sum():8.0f}")


if __name__ == '__main__':
    main()
