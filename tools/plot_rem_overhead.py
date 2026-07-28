#!/usr/bin/env python3
"""
学習した REM の俯瞰図 (道路平面 x-y への投影)。

2枚を並べる:
  左  担当領域   各車線の各位置で「平均場 μ が最良の RSU」を色分け。
                 どの RSU がどこを受け持つかが一目で分かる
  右  遮蔽リスク 各車線を σ_ν²(s) で着色。どこが遮蔽されやすいかの地図

車線は `--lanes <y>:<キー接尾辞>` で任意本数を指定できる。現在の REM は
(RSU, 進行方向) ごとの 1 次元場なので接尾辞は p/m だが、2 次元 REM (s,d) や
車線別 REM へ移行しても、キーの対応を変えるだけで同じ図が描ける。

  python3 tools/plot_rem_overhead.py sim_results/urban_learn/rem_state
  python3 tools/plot_rem_overhead.py <state_dir> --lanes 1.75:p -1.75:m
  python3 tools/plot_rem_overhead.py <state_dir> --lanes 5.25:l0 1.75:l1 -1.75:l2 -5.25:l3
"""
import argparse
import glob
import json
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt          # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402
import numpy as np                        # noqa: E402


def load_states(state_dir):
    out = {}
    for f in sorted(glob.glob(os.path.join(state_dir, 'bs*.npz'))):
        key = os.path.basename(f)[2:-4]           # "0_p" / "0" / "0_l1"
        with np.load(f) as d:
            out[key] = {'alpha': d['alpha'], 'P': d['P'], 'S2': d['S2'],
                        'W': d['W'], 'meta': json.loads(str(d['meta']))}
    return out


def rbf_design(s_grid, num_bases, s_min, s_max, width_m=0.0):
    centers = np.linspace(s_min, s_max, num_bases)
    spacing = (s_max - s_min) / (num_bases - 1)
    w = width_m if width_m > 0 else spacing
    d = (s_grid[:, None] - centers[None, :]) / w
    return np.exp(-0.5 * d * d)


def lane_fields(states, suffix, s, Phi, s_min, s_max, w_min):
    """その車線に属する全 RSU の μ(s), σ_ν²(s), W(s) を集める。"""
    keys = sorted(k for k in states
                  if (k.split('_', 1)[1] if '_' in k else '') == suffix)
    mus, risks, ws, ids = [], [], [], []
    for k in keys:
        st = states[k]
        grid = np.linspace(s_min, s_max, len(st['W']))
        ok = st['W'] >= w_min
        risk = np.where(ok, st['S2'] / np.maximum(st['W'], 1e-12), np.nan)
        mus.append(Phi @ st['alpha'])
        risks.append(np.interp(s, grid, np.nan_to_num(risk, nan=0.0)))
        ws.append(np.interp(s, grid, st['W']))
        ids.append(int(k.split('_')[0]))
    if not keys:
        return None
    return (np.array(mus), np.array(risks), np.array(ws), ids)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('state_dir')
    ap.add_argument('--out', default=None)
    ap.add_argument('--lanes', nargs='*', default=['1.75:p', '-1.75:m'],
                    help='<車線y>:<キー接尾辞> の並び (車線数は任意)')
    ap.add_argument('--lane-width', type=float, default=3.5)
    ap.add_argument('--s-min', type=float, default=0.0)
    ap.add_argument('--s-max', type=float, default=230.0)
    ap.add_argument('--x-offset', type=float, default=-115.0,
                    help='弧長0に対応する world x [m]')
    ap.add_argument('--rsu-x', type=float, nargs='*', default=[-15, -5, 5, 15])
    ap.add_argument('--rsu-y', type=float, default=6.0)
    ap.add_argument('--rsu-tilt-deg', type=float, nargs='*',
                    default=[-70, 70, -70, 70],
                    help='RSU の傾き (車線正対から下流+ / 上流−)。矢印で描く')
    ap.add_argument('--xlim', type=float, nargs=2, default=[-90.0, 90.0])
    ap.add_argument('--w-min', type=float, default=0.5)
    ap.add_argument('--mu-min', type=float, default=-68.5,
                    help='接続閾値 [dBm]。学習した μ がこれ未満の領域は '
                         '「どの RSU にも繋がらない」として塗らない')
    ap.add_argument('--risk-max', type=float, default=0.0,
                    help='リスク色スケールの上限 (0=データの95%分位)')
    a = ap.parse_args()

    states = load_states(a.state_dir)
    if not states:
        sys.exit(f"状態ファイルがありません: {a.state_dir}")
    lanes = []
    for spec in a.lanes:
        y_s, suf = spec.split(':')
        lanes.append((float(y_s), suf))

    n_bases = len(next(iter(states.values()))['alpha'])
    s = np.linspace(a.s_min, a.s_max, 700)
    Phi = rbf_design(s, n_bases, a.s_min, a.s_max)
    x = s + a.x_offset
    run_count = next(iter(states.values()))['meta'].get('run_count', '?')

    # 車線ごとの場を集める
    per_lane = {}
    for y, suf in lanes:
        got = lane_fields(states, suf, s, Phi, a.s_min, a.s_max, a.w_min)
        if got is not None:
            per_lane[(y, suf)] = got
    if not per_lane:
        sys.exit("指定した接尾辞に対応する地図がありません: "
                 f"{[suf for _, suf in lanes]} / 実在キー {sorted(states)}")

    rsu_ids = sorted({i for v in per_lane.values() for i in v[3]})
    cmap_rsu = plt.get_cmap('tab10')
    risk_all = np.concatenate([v[1].ravel() for v in per_lane.values()])
    risk_all = risk_all[np.isfinite(risk_all) & (risk_all > 0)]
    vmax = a.risk_max if a.risk_max > 0 else (
        float(np.quantile(risk_all, 0.95)) if risk_all.size else 1.0)

    fig, axes = plt.subplots(1, 2, figsize=(17.0, 5.0), sharey=True)
    y_lo = min(y for y, _ in lanes) - a.lane_width
    y_hi = a.rsu_y + 1.5

    for ax, mode in zip(axes, ('serving', 'risk')):
        # 道路の地物
        ax.axhspan(min(y for y, _ in lanes) - a.lane_width / 2,
                   max(y for y, _ in lanes) + a.lane_width / 2,
                   color='#eeeeee', zorder=0)
        ax.axhline(0.0, color='#999999', ls='--', lw=1.0, zorder=1)  # 中央線

        for (y, suf), (mus, risks, ws, ids) in per_lane.items():
            # 観測が無い所と、どの RSU も接続閾値に届かない所は塗らない
            mu_masked = np.where(ws >= a.w_min, mus, -np.inf)
            obs = (ws.max(axis=0) >= a.w_min) & (mu_masked.max(axis=0) >= a.mu_min)
            if mode == 'serving':
                best = np.argmax(mu_masked, axis=0)
                col = np.array([cmap_rsu(ids[b] % 10) for b in best])
                col[~obs] = (1, 1, 1, 0)
                img = col.reshape(1, -1, 4)
                ax.imshow(img, extent=[x[0], x[-1],
                                       y - a.lane_width / 2, y + a.lane_width / 2],
                          aspect='auto', origin='lower', zorder=2,
                          interpolation='nearest')
            else:
                r = np.nanmax(np.where(ws >= a.w_min, risks, np.nan), axis=0)
                r = np.where(obs, r, np.nan)
                ax.imshow(r.reshape(1, -1), extent=[x[0], x[-1],
                                                    y - a.lane_width / 2,
                                                    y + a.lane_width / 2],
                          aspect='auto', origin='lower', cmap='inferno',
                          vmin=0.0, vmax=vmax, zorder=2, interpolation='nearest')
            ax.text(a.xlim[0] + 2, y, f'lane y={y:+.2f} ({suf})', va='center',
                    fontsize=8, zorder=5,
                    bbox=dict(fc='white', ec='none', alpha=0.65, pad=1.5))

        # RSU とボアサイト
        for i, rx in enumerate(a.rsu_x):
            ax.plot(rx, a.rsu_y, marker='v', ms=11, color=cmap_rsu(i % 10),
                    mec='black', mew=0.6, zorder=6)
            ax.text(rx, a.rsu_y + 0.55, f'RSU{i}', ha='center', fontsize=8, zorder=6)
            if i < len(a.rsu_tilt_deg):
                th = np.radians(-90.0 + a.rsu_tilt_deg[i])   # 車線正対 = -y
                ax.arrow(rx, a.rsu_y, 9.0 * np.cos(th), 9.0 * np.sin(th),
                         head_width=0.5, head_length=1.6, fc=cmap_rsu(i % 10),
                         ec=cmap_rsu(i % 10), alpha=0.75, zorder=6, length_includes_head=True)
        ax.set_xlim(*a.xlim)
        ax.set_ylim(y_lo, y_hi)
        ax.set_xlabel('world x [m]')
        ax.grid(alpha=0.25, zorder=0)

    axes[0].set_ylabel('world y [m]')
    axes[0].set_title(f'serving RSU (best learned mean field, mu >= {a.mu_min:.1f} dBm)'
                      f'   [{run_count} runs]')
    axes[1].set_title(f'blockage risk  sigma_nu^2  [dB^2]  (0-{vmax:.0f})')
    handles = [plt.Line2D([], [], marker='s', ls='', color=cmap_rsu(i % 10),
                          label=f'RSU{i}') for i in rsu_ids]
    axes[0].legend(handles=handles, loc='lower right', fontsize=8, ncol=len(rsu_ids))
    fig.colorbar(plt.cm.ScalarMappable(cmap='inferno',
                                       norm=plt.Normalize(0, vmax)),
                 ax=axes[1], fraction=0.03, pad=0.01, label='sigma_nu^2 [dB^2]')
    fig.suptitle('Learned REM — overhead view', y=1.02)
    fig.tight_layout()

    out = a.out or os.path.join(a.state_dir, 'rem_overhead.png')
    fig.savefig(out, dpi=140, bbox_inches='tight')
    print(f"wrote {out}")

    # 数値サマリ: 車線ごとに各 RSU が担当する区間長
    print("\n車線   RSU  担当長[m]  平均リスク[dB²]")
    for (y, suf), (mus, risks, ws, ids) in per_lane.items():
        mu_masked = np.where(ws >= a.w_min, mus, -np.inf)
        best = np.argmax(mu_masked, axis=0)
        obs = (ws.max(axis=0) >= a.w_min) & (mu_masked.max(axis=0) >= a.mu_min)
        dx = (x[-1] - x[0]) / (len(x) - 1)
        for j, rid in enumerate(ids):
            sel = obs & (best == j)
            if sel.sum() == 0:
                continue
            print(f"y={y:+6.2f} RSU{rid}  {sel.sum() * dx:8.1f}  "
                  f"{np.nanmean(risks[j][sel]):12.1f}")


if __name__ == '__main__':
    main()
