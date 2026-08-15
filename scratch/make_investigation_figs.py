#!/usr/bin/env python3
"""内部調査資料の図を描く。

数値はすべて sim_results の集計 CSV から読む (make_investigation_docx.py と同じ)。
図は scratch/review/figs/ に PNG で出力し、文書側から貼り込む。

日本語フォントについて
----------------------
コンテナには CJK フォントが入っていないので、ホストの Noto Sans CJK を
1 度だけ入れておく必要がある (コンテナを作り直したら再実行すること):

  docker cp /usr/share/fonts/google-noto-cjk/NotoSansCJK-Regular.ttc \\
      comms_sim:/usr/share/fonts/NotoSansCJK-Regular.ttc
  docker compose exec -T sim bash -c "fc-cache -f && rm -rf /root/.cache/matplotlib"

  python3 scratch/make_investigation_figs.py
"""
import os
import sys

import matplotlib
matplotlib.use('Agg')
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager as fm  # noqa: E402
from scipy import stats  # noqa: E402

ROOT = '/workspace'
R = f'{ROOT}/sim_results'
OUT = f'{ROOT}/scratch/review/figs'
os.makedirs(OUT, exist_ok=True)
sys.path.insert(0, f'{ROOT}/tools')
from lib import eval_metrics  # noqa: E402

FONT = '/usr/share/fonts/NotoSansCJK-Regular.ttc'
if not os.path.exists(FONT):
    sys.exit('日本語フォントがない。docstring の docker cp を実行すること')
fm.fontManager.addfont(FONT)
plt.rcParams.update({
    'font.family': 'Noto Sans CJK JP',
    'font.size': 12,
    'axes.edgecolor': '#5A6672',
    'axes.labelcolor': '#0E2841',
    'text.color': '#0E2841',
    'xtick.color': '#5A6672',
    'ytick.color': '#5A6672',
    'axes.spines.top': False,
    'axes.spines.right': False,
    'figure.facecolor': 'white',
})

# 濃紺を主、強調をオレンジにする (青/橙は色覚多様性でも分離できる組)。
# 系列の識別は位置と直接ラベルが担い、色は「提案手法かどうか」だけを示す
NAVY, ORANGE, GRAY, LIGHT, INK = '#1F4E79', '#C55A11', '#A6B1BB', '#C9D6E3', '#0E2841'


def save(fig, name):
    p = os.path.join(OUT, name)
    fig.savefig(p, dpi=200, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    print('  ', p)
    return p


# ------------------------------------------------------------------ データ
def load(tag):
    runs = pd.read_csv(f'{R}/{tag}/analysis/runs.csv')
    agg = pd.read_csv(f'{R}/{tag}/analysis/agg.csv').set_index('method')
    return runs, agg


def ci(x):
    x = pd.Series(x).dropna()
    if len(x) < 2 or x.std(ddof=1) == 0:
        return 0.0
    return float(stats.t.ppf(0.975, len(x) - 1) * x.std(ddof=1) / np.sqrt(len(x)))


def diff_vs(runs, m, ref='assoc_hold'):
    a = runs[runs.method == ref].set_index('run').total_data_MB
    x = runs[runs.method == m].set_index('run').total_data_MB
    return (x - a).dropna(), (100 * (x - a) / a).dropna()


runs, agg = load('abl_d70_n50')
o_runs, o_agg = load('rec_cm_d70_ablation')
N = int(agg.loc['assoc_hold', 'n'])
N_OLD = int(o_agg.loc['assoc_hold', 'n'])
pk = eval_metrics.paired(runs, 'kkf_conv', 'method', ['load'], ['total_data_MB'])
pk = pk[pk.metric == 'total_data_MB'].set_index('baseline')

LBL = {'oracle': 'oracle\n(全ペアの受信電力が既知)', 'kkf_cold_probe': 'kkf_cold_probe\n(全ペア観測・地図なし)',
       'kkf_conv': 'kkf_conv\n(提案手法)', 'kkf_nolcb': 'kkf_nolcb\n(LCB なし)',
       'kkf_novar': 'kkf_novar\n(遮蔽リスク地図なし)', 'assoc_hold': 'assoc_hold\n(受動接続)'}


# ------------------------------------------------ 図1 受動接続に対する優位性
def fig_gain():
    ms = ['kkf_novar', 'kkf_nolcb', 'kkf_conv', 'kkf_cold_probe', 'oracle']
    vals, errs, labels, mbs = [], [], [], []
    for m in ms:
        dmb, drel = diff_vs(runs, m)
        vals.append(drel.mean())
        errs.append(ci(drel))
        mbs.append(dmb.mean())
        labels.append(LBL[m])
    fig, ax = plt.subplots(figsize=(9.6, 4.6))
    y = np.arange(len(ms))
    colors = [ORANGE if m == 'kkf_conv' else NAVY for m in ms]
    ax.barh(y, vals, xerr=errs, color=colors, height=0.62,
            error_kw=dict(ecolor=INK, capsize=4, lw=1.2), zorder=3)
    for i, (v, e, mb) in enumerate(zip(vals, errs, mbs)):
        ax.text(v + e + 0.5, i, f'+{v:.1f}%  ({mb:+,.0f} MB)', va='center',
                ha='left', fontsize=11.5, color=INK)
    ax.axvline(0, color=INK, lw=1.4, zorder=4)
    ax.text(0.15, len(ms) - 0.35, 'assoc_hold (受動接続) = 0', fontsize=11, color=INK)
    ax.set_yticks(y)
    ax.set_yticklabels(labels, fontsize=11.5)
    ax.set_xlabel('受動接続に対する走行ごと相対差 [%]（誤差棒は 95% 信頼区間）')
    ax.set_xlim(0, max(v + e for v, e in zip(vals, errs)) * 1.42)
    ax.xaxis.grid(True, color=LIGHT, lw=0.8)
    ax.set_axisbelow(True)
    return save(fig, 'fig1_gain.png')


# ------------------------------------------------ 図2 接続時間と grant 時間
def fig_time():
    """掴んだ時間のうち実際に送れた割合。棒ではなく点で描く (0 起点でないため)。"""
    ms = ['assoc_hold', 'kkf_cold_probe', 'kkf_conv', 'oracle']
    y = np.arange(len(ms))
    conn = np.array([float(agg.loc[m, 'connected_time_s_mean']) for m in ms])
    gr = np.array([float(agg.loc[m, 'grant_time_s_mean']) for m in ms])
    ratio = 100 * conn / gr
    fig, ax = plt.subplots(figsize=(9.6, 3.6))
    lo = 90
    for i, (r, c, g, m) in enumerate(zip(ratio, conn, gr, ms)):
        col = ORANGE if m == 'kkf_conv' else NAVY
        ax.plot([lo, r], [i, i], color=LIGHT, lw=2.4, zorder=2, solid_capstyle='round')
        ax.scatter([r], [i], s=150, color=col, zorder=4, edgecolor='white', lw=1.0)
        ax.text(r + 0.25, i, f'{r:.1f}%　（接続 {c:.0f} s / grant {g:.0f} s）',
                va='center', fontsize=11.5, color=INK)
    ax.set_yticks(y)
    ax.set_yticklabels([LBL[m] for m in ms], fontsize=11.5)
    ax.set_xlim(lo, 105)
    ax.set_xticks([90, 92, 94, 96, 98, 100])
    ax.set_xlabel('掴んでいた時間 (grant) のうち実際に送れた時間の割合 [%]')
    ax.xaxis.grid(True, color=LIGHT, lw=0.8)
    ax.set_axisbelow(True)
    ax.spines['left'].set_visible(False)
    return save(fig, 'fig3_time.png')


# ------------------------------------------------ 図3 走行ごとの分布 (N=13 / N=48)
def fig_runs():
    _, rel_new = diff_vs(runs, 'kkf_conv')
    _, rel_old = diff_vs(o_runs, 'kkf_conv')
    fig, ax = plt.subplots(figsize=(9.6, 3.6))
    rng = np.random.default_rng(0)
    for i, (r, lab, col) in enumerate([(rel_old, f'N = {N_OLD}（前回）', '#7E8C99'),
                                       (rel_new, f'N = {N}（今回）', NAVY)]):
        yy = i + rng.uniform(-0.13, 0.13, len(r))
        ax.scatter(r, yy, s=34, color=col, alpha=0.75, edgecolor='white', lw=0.6, zorder=3)
        m, e = r.mean(), ci(r)
        ax.errorbar(m, i + 0.30, xerr=e, fmt='o', color=ORANGE, ms=9, lw=2.4,
                    capsize=5, zorder=5)
        ax.text(m, i + 0.52, f'平均 +{m:.1f}%  ±{e:.1f} pt', ha='center',
                fontsize=11.5, color=ORANGE)
        ax.text(-0.5, i, lab, ha='right', va='center', fontsize=11.5, color=INK)
    ax.axvline(0, color=INK, lw=1.4)
    ax.set_xlabel('走行ごと相対差 [%]（点は 1 走行、橙は平均と 95% 信頼区間）')
    ax.set_yticks([])
    ax.set_ylim(-0.55, 1.85)
    ax.set_xlim(-4, max(rel_new.max(), rel_old.max()) * 1.1)
    ax.spines['left'].set_visible(False)
    ax.xaxis.grid(True, color=LIGHT, lw=0.8)
    ax.set_axisbelow(True)
    return save(fig, 'fig2_runs.png')


# ------------------------------------------------ 図4 機構ごとの寄与
def fig_ablation():
    items = [('kkf_cold_probe', '事前学習を外し\n全ペア観測を許す'),
             ('kkf_novar', '遮蔽リスク地図 σν²\nを外す'),
             ('kkf_nolcb', '不確実性の利用\n(LCB, κσ) を外す')]
    v = [float(pk.loc[m, 'diff_mean']) for m, _ in items]
    e = [float(pk.loc[m, 'diff_ci95']) for m, _ in items]
    fig, ax = plt.subplots(figsize=(9.6, 3.6))
    y = np.arange(len(items))
    ax.barh(y, v, xerr=e, height=0.55, color=[NAVY if x > 0 else GRAY for x in v],
            error_kw=dict(ecolor=INK, capsize=4, lw=1.2), zorder=3)
    for i, (val, err) in enumerate(zip(v, e)):
        ax.text(val + np.sign(val) * (err + 60), i,
                f'{val:+,.0f} MB  (p = {float(pk.loc[items[i][0], "wilcoxon_p"]):.2g})',
                va='center', ha='left' if val > 0 else 'right', fontsize=11.5, color=INK)
    ax.axvline(0, color=INK, lw=1.4)
    ax.set_yticks(y)
    ax.set_yticklabels([lab for _, lab in items], fontsize=11.5)
    ax.invert_yaxis()
    ax.set_xlabel('kkf_conv との対差 [MB]（正 = 提案手法が上回る、誤差棒は 95% 信頼区間）')
    ax.set_xlim(min(v) - 1500, max(v) + 1000)
    ax.xaxis.grid(True, color=LIGHT, lw=0.8)
    ax.set_axisbelow(True)
    return save(fig, 'fig4_ablation.png')


# ------------------------------------------------ 図5 公平性
def fig_fair():
    ms = ['assoc_hold', 'kkf_cold_probe', 'oracle', 'kkf_conv']
    v = [float(agg.loc[m, 'min_car_data_MB_mean']) for m in ms]
    z = [float(agg.loc[m, 'zero_car_pct_mean']) for m in ms]
    fig, ax = plt.subplots(figsize=(9.6, 3.4))
    y = np.arange(len(ms))
    ax.barh(y, v, height=0.55, color=[ORANGE if m == 'kkf_conv' else NAVY for m in ms], zorder=3)
    for i, (val, zz) in enumerate(zip(v, z)):
        ax.text(val + 4, i, f'{val:,.0f} MB　（配信量 0 の車両 {zz:.2f}%）',
                va='center', fontsize=11.5, color=INK)
    ax.set_yticks(y)
    ax.set_yticklabels([LBL[m] for m in ms], fontsize=11.5)
    ax.set_xlabel('最も恵まれない車両の配信量 [MB]（走行平均）')
    ax.set_xlim(0, max(v) * 1.75)
    ax.xaxis.grid(True, color=LIGHT, lw=0.8)
    ax.set_axisbelow(True)
    return save(fig, 'fig5_fairness.png')


# ------------------------------------------------ 図6 天井 vs 選択余地 / カバレッジ
def fig_ceiling():
    cm = pd.read_csv(f'{R}/choice_margin_summary.csv')
    cf = pd.read_csv(f'{R}/confound_summary.csv')
    cm['群'], cf['群'] = '掃引 (δ_rsu を振る)', '直交 (本数 × 間隔)'
    t = pd.concat([cm[cm['索引OK'] == True], cf[cf['索引OK'] == True]], ignore_index=True)
    t['名'] = t['条件'].str.replace('rec_cm_', '', regex=False).str.replace('rec_cf_', '直交 ', regex=False)
    THR = 0.155
    fig, axes = plt.subplots(1, 2, figsize=(11.0, 4.4))
    for ax, xcol, xlabel in [
            (axes[0], '選択余地率', '選択余地率（2 基以上が同時に接続可能な時刻の割合）'),
            (axes[1], 'assoc_hold_MB', 'カバレッジ = 受動接続の総配信量 [MB]')]:
        for g, mk, col in [('掃引 (δ_rsu を振る)', 'o', NAVY), ('直交 (本数 × 間隔)', '^', ORANGE)]:
            s = t[t['群'] == g]
            ax.scatter(s[xcol], s['天井_pct'], s=90, marker=mk, color=col,
                       edgecolor='white', lw=0.8, label=g, zorder=4)
        for _, r in t.iterrows():
            ax.annotate(r['名'], (r[xcol], r['天井_pct']), textcoords='offset points',
                        xytext=(7, 5), fontsize=10.5, color='#5A6672')
        rr, pp = stats.pearsonr(t[xcol], t['天井_pct'])
        hi = t[t['選択余地率'] >= THR]
        rh, _ = stats.pearsonr(hi[xcol], hi['天井_pct'])
        ax.set_xlabel(xlabel, fontsize=11.5)
        ax.set_ylim(-2, 34)
        ax.margins(x=0.16)
        ax.yaxis.grid(True, color=LIGHT, lw=0.8)
        ax.set_axisbelow(True)
        # 左の枠には網掛けと閾値の注記があるので、r の注記は上に逃がす
        pos = (0.02, 0.98, 'left', 'top') if xcol == '選択余地率' else (0.97, 0.05, 'right', 'bottom')
        ax.text(pos[0], pos[1],
                f'r = {rr:+.2f} (全 {len(t)} 条件, p = {pp:.2f})\n'
                f'r = {rh:+.2f} (余地 {THR} 以上の {len(hi)} 条件)',
                transform=ax.transAxes, fontsize=10.5, color='#5A6672',
                va=pos[3], ha=pos[2], linespacing=1.4)
    axes[0].axvline(THR, color=GRAY, ls='--', lw=1.4, zorder=2)
    axes[0].text(THR + 0.012, 8.5, f'余地 {THR}', fontsize=11, color='#5A6672')
    axes[0].axhspan(-2, 6, color=LIGHT, alpha=0.45, zorder=1)
    axes[0].text(0.36, 2.4, '天井がほぼ無い領域', fontsize=11, color='#5A6672')
    axes[0].set_ylabel('天井 = oracle の assoc_hold 比 [%]', fontsize=11.5)
    axes[1].set_yticklabels([])
    axes[0].legend(loc='upper right', frameon=False, fontsize=11)
    fig.tight_layout(w_pad=1.2)
    return save(fig, 'fig6_ceiling.png')


if __name__ == '__main__':
    print('図を出力:')
    fig_gain()
    fig_time()
    fig_runs()
    fig_ablation()
    fig_fair()
    fig_ceiling()
