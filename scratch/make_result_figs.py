#!/usr/bin/env python3
"""内部調査資料の結果図を描く。数値はすべて sim_results の集計から読む。

フォント: 英数字 Liberation Serif (Times New Roman 互換)、日本語 Noto Serif CJK JP
(MS 明朝相当)。コンテナに CJK 明朝を入れておくこと:
  docker cp /usr/share/fonts/google-noto-cjk/NotoSerifCJK-Regular.ttc \
      comms_sim:/usr/share/fonts/NotoSerifCJK-Regular.ttc
"""
import os, glob
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
from matplotlib import font_manager as fm
import matplotlib.pyplot as plt
from scipy import stats

R, OUT = '/workspace/sim_results', '/workspace/scratch/review/figs'
os.makedirs(OUT, exist_ok=True)
fm.fontManager.addfont('/usr/share/fonts/NotoSerifCJK-Regular.ttc')
plt.rcParams.update({
    'font.family': ['Liberation Serif', 'Noto Serif CJK JP'],
    'font.size': 12, 'axes.edgecolor': '#333333', 'axes.labelcolor': 'black',
    'text.color': 'black', 'xtick.color': '#333333', 'ytick.color': '#333333',
    'axes.spines.top': False, 'axes.spines.right': False,
    'figure.facecolor': 'white', 'axes.grid': True, 'grid.alpha': 0.25,
    'grid.linestyle': '-', 'grid.color': '#BBBBBB',
})
NAVY, ORANGE, GRAY, LIGHT = '#1F4E79', '#C55A11', '#8C959E', '#D6DEE7'


def save(fig, name):
    p = f'{OUT}/{name}'
    fig.savefig(p, dpi=200, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    print('  ', name)


def runs(tag):
    return pd.read_csv(f'{R}/{tag}/analysis/runs.csv')


def piv(tag, val='total_data_MB'):
    return runs(tag).pivot_table(index='run', columns='method', values=val)


def paired(p, a, b='assoc_hold'):
    x = (p[a] - p[b]).dropna()
    t = stats.t.ppf(0.975, len(x) - 1)
    ci = t * x.std(ddof=1) / np.sqrt(len(x))
    try:
        pv = stats.wilcoxon(x)[1]
    except Exception:
        pv = float('nan')
    return x, x.mean(), ci, pv


MAIN = 'ablation_n50'
LBL = {'assoc_hold': '受動接続\n(802.15.3e 準拠)', 'kkf_conv': '提案手法',
       'oracle': '理想反応型\n(全ペア既知)', 'kkf_cold_probe': '全ペア観測\n(規格の枠外)',
       'kkf_cold': '地図なし', 'kkf_nolcb': 'LCB なし', 'kkf_novar': 'σν² なし',
       'kkf_nokrig': '残差クリギングなし', 'kkf_frozen': '走行内更新なし'}
print('図を描いています:')

# ---------------------------------------------------------------- 図H 主結果
p = piv(MAIN)
arms = ['kkf_conv', 'kkf_cold_probe', 'oracle']
fig, ax = plt.subplots(figsize=(8.4, 4.4))
rows = []
for a in arms:
    x, m, ci, pv = paired(p, a)
    rows.append((LBL[a], m, ci, 100 * m / p['assoc_hold'].mean(), pv))
y = np.arange(len(rows))[::-1]
cols = [ORANGE if r[0] == '提案手法' else NAVY for r in rows]
ax.barh(y, [r[1] for r in rows], xerr=[r[2] for r in rows], color=cols,
        height=0.52, error_kw=dict(ecolor='#333333', capsize=5, lw=1.2))
for yy, r in zip(y, rows):
    ax.text(r[1] + r[2] + 90, yy, f'{r[1]:+,.0f} MB  ({r[3]:+.1f}%)',
            va='center', fontsize=12)
ax.set_yticks(y); ax.set_yticklabels([r[0] for r in rows], fontsize=12)
ax.set_xlabel('受動接続との対差 [MB]（誤差棒は 95% 信頼区間）')
ax.set_xlim(0, max(r[1] for r in rows) * 1.42)
ax.set_title(f'図H　規格準拠の受動接続に対する優位（{len(p)} 走行の対比較）', loc='left',
             fontsize=13.5, pad=12)
save(fig, 'H_gain.png')

# ---------------------------------------------------------------- 図I 走行ごと
x, m, ci, pv = paired(p, 'kkf_conv')
rel = (100 * x / p['assoc_hold']).sort_values().values
fig, ax = plt.subplots(figsize=(8.4, 4.0))
ax.bar(np.arange(len(rel)), rel, color=ORANGE, width=0.82)
ax.axhline(0, color='#333333', lw=1.0)
ax.axhline(rel.mean(), color=NAVY, lw=1.4, ls='--')
ax.text(len(rel) - 0.5, rel.mean() + 0.6, f'平均 {rel.mean():+.1f}%',
        ha='right', color=NAVY, fontsize=12)
ax.annotate(f'最小 {rel.min():+.1f}%', xy=(0, rel.min()), xytext=(3.5, rel.min() + 3.4),
            arrowprops=dict(arrowstyle='->', color='#333333', lw=1.0), fontsize=11.5)
ax.set_xlabel(f'走行（相対差の昇順、{len(rel)} 走行）')
ax.set_ylabel('受動接続に対する相対差 [%]')
ax.set_xlim(-1, len(rel)); ax.set_ylim(0, rel.max() * 1.15)
ax.set_title(f'図I　{len(rel)} 走行すべてで提案手法が上回る', loc='left',
             fontsize=13.5, pad=12)
save(fig, 'I_runs.png')

# ---------------------------------------------------------------- 図J grant効率
d = runs(MAIN)
g = d.groupby('method')[['connected_time_s', 'grant_time_s']].mean()
order = ['assoc_hold', 'kkf_conv', 'oracle']
eff = [100 * g.loc[a, 'connected_time_s'] / g.loc[a, 'grant_time_s'] for a in order]
fig, ax = plt.subplots(figsize=(7.4, 4.0))
cols = [GRAY, ORANGE, NAVY]
b = ax.bar(range(3), eff, color=cols, width=0.52)
for i, a in enumerate(order):
    ax.text(i, eff[i] + 0.35, f'{eff[i]:.1f}%', ha='center', fontsize=13)
    ax.text(i, 81.0, f'接続 {g.loc[a, "connected_time_s"]:.0f} s\n'
                     f'grant {g.loc[a, "grant_time_s"]:.0f} s',
            ha='center', fontsize=10.5, color='#333333')
ax.set_xticks(range(3))
ax.set_xticklabels([LBL[a].replace('\n', ' ') for a in order], fontsize=11.5)
ax.set_ylim(80, 101); ax.set_ylabel('割当のうち実際に送れた割合 [%]')
ax.set_title('図J　利得の源泉は「繋がらない相手を選ばない」こと', loc='left',
             fontsize=13.5, pad=12)
save(fig, 'J_grant.png')

# ---------------------------------------------------------------- 図K アブレーション
gain = paired(p, 'kkf_conv')[1]
abl = ['kkf_novar', 'kkf_nolcb', 'kkf_nokrig', 'kkf_frozen']
res = []
for a in abl:
    x2 = (p['kkf_conv'] - p[a]).dropna()
    t = stats.t.ppf(0.975, len(x2) - 1)
    res.append((LBL[a], x2.mean(), t * x2.std(ddof=1) / np.sqrt(len(x2)),
                stats.wilcoxon(x2)[1]))
fig, ax = plt.subplots(figsize=(10.2, 4.2))
y = np.arange(len(res))[::-1]
cols = [NAVY if r[3] < 0.05 else GRAY for r in res]
ax.barh(y, [r[1] for r in res], xerr=[r[2] for r in res], color=cols, height=0.5,
        error_kw=dict(ecolor='#333333', capsize=5, lw=1.2))
ax.axvline(0, color='#333333', lw=1.0)
# 注記は棒の右端より外側の共通位置に左揃えで置く (軸ラベルと衝突させない)
lab_x = max(r[1] + r[2] for r in res) * 1.10
for yy, r in zip(y, res):
    sig = f'p = {r[3]:.2g}' if r[3] >= 0.001 else 'p < 0.001'
    ax.text(lab_x, yy, f'{r[1]:+.0f} MB　利得の {100*r[1]/gain:+.1f}%　{sig}',
            va='center', ha='left', fontsize=11.5)
ax.set_yticks(y); ax.set_yticklabels([r[0] for r in res], fontsize=12)
ax.set_xlabel('提案手法との対差 [MB]　正の値はその機構が効いていることを示す')
ax.set_xlim(-120, lab_x * 2.55)
ax.set_title('図K　機構を1つずつ外したときの変化（灰色は有意でない）', loc='left',
             fontsize=13.5, pad=12)
save(fig, 'K_ablation.png')

# ---------------------------------------------------------------- 図M 公平性
a = pd.read_csv(f'{R}/{MAIN}/analysis/agg.csv').set_index('method')
order = ['assoc_hold', 'kkf_conv', 'oracle']
fig, axes = plt.subplots(1, 2, figsize=(10.4, 4.0))
cols = [GRAY, ORANGE, NAVY]
v = [a.loc[m, 'min_car_data_MB_mean'] for m in order]
e = [a.loc[m, 'min_car_data_MB_ci95'] for m in order]
axes[0].bar(range(3), v, yerr=e, color=cols, width=0.52,
            error_kw=dict(ecolor='#333333', capsize=5, lw=1.2))
for i, x2 in enumerate(v):
    axes[0].text(i, x2 + e[i] + 1.2, f'{x2:.1f}', ha='center', fontsize=12.5)
axes[0].set_ylabel('最も少ない車両の配信量 [MB]')
axes[0].set_title('(a) 最も恵まれない車両', fontsize=12.5, loc='left')
v2 = [a.loc[m, 'zero_car_pct_mean'] for m in order]
e2 = [a.loc[m, 'zero_car_pct_ci95'] for m in order]
axes[1].bar(range(3), v2, yerr=e2, color=cols, width=0.52,
            error_kw=dict(ecolor='#333333', capsize=5, lw=1.2))
for i, x2 in enumerate(v2):
    axes[1].text(i, x2 + e2[i] + 0.12, f'{x2:.2f}%', ha='center', fontsize=12.5)
axes[1].set_ylabel('配信量が 0 の車両の割合 [%]')
axes[1].set_title('(b) 取りこぼし', fontsize=12.5, loc='left')
for ax2 in axes:
    ax2.set_xticks(range(3))
    ax2.set_xticklabels([LBL[m].replace('\n', ' ') for m in order], fontsize=10.5)
fig.suptitle('図M　車両間の公平性', x=0.09, ha='left', fontsize=13.5)
fig.tight_layout(rect=[0, 0, 1, 0.94])
save(fig, 'M_fairness.png')

# ---------------------------------------------------------------- 図G 変更の分解
steps = [('旧版\n閾値 −78.5 dBm', 'abl_d70_n50'),
         ('接続閾値を規格値へ\n−68.5 dBm', 'ablation_snrmin265'),
         ('記録の欠落を解消\n＋走り出し起点の窓', 'ablation_clean'),
         ('全走行車両を\n通信対象に', 'ablation_n50')]
vals = []
for lab, tag in steps:
    q = piv(tag)
    vals.append((lab, (100 * (q['kkf_conv'] - q['assoc_hold']) / q['assoc_hold']).mean(),
                 len(q)))
fig, ax = plt.subplots(figsize=(9.0, 4.4))
xs = np.arange(len(vals))
cols = [GRAY, '#8C2F1E', NAVY, ORANGE]
ax.bar(xs, [v[1] for v in vals], color=cols, width=0.55)
for i, v in enumerate(vals):
    ax.text(i, v[1] + 0.55, f'{v[1]:+.1f}%', ha='center', fontsize=13.5)
    ax.text(i, 0.6, f'N = {v[2]}', ha='center', fontsize=10.5, color='white')
for i in range(len(vals) - 1):
    ax.annotate('', xy=(i + 0.72, vals[i + 1][1]), xytext=(i + 0.28, vals[i][1]),
                arrowprops=dict(arrowstyle='->', color='#333333', lw=1.1))
ax.set_xticks(xs); ax.set_xticklabels([v[0] for v in vals], fontsize=11)
ax.set_ylabel('受動接続に対する相対差 [%]')
ax.set_ylim(0, max(v[1] for v in vals) * 1.22)
ax.set_title('図G　本版で数値が変わった理由', loc='left', fontsize=13.5, pad=12)
save(fig, 'G_changes.png')

# ---------------------------------------------------------------- 図N 需要と利得
# 需要の比較は同一の交通実現どうしで対にする必要があるため、
# 対象 80% 版と同じ記録から作った ablation_all を使う (ablation_n50 は
# 補充した 2 走行を含み、80% 版に対応する走行が無い)
pa, pc = piv('ablation_all'), piv('ablation_clean')
na = runs('ablation_all').groupby('run').n_cars.max()
nc = runs('ablation_clean').groupby('run').n_cars.max()
common = sorted(set(pa.index) & set(pc.index))
ra = (100 * (pa['kkf_conv'] - pa['assoc_hold']) / pa['assoc_hold']).loc[common]
rc = (100 * (pc['kkf_conv'] - pc['assoc_hold']) / pc['assoc_hold']).loc[common]
fig, axes = plt.subplots(1, 2, figsize=(10.6, 4.2))
axes[0].scatter(nc.loc[common], rc, s=26, color=GRAY, label='対象 80%')
axes[0].scatter(na.loc[common], ra, s=26, color=ORANGE, label='全車対象')
z = np.polyfit(list(nc.loc[common]) + list(na.loc[common]),
               list(rc) + list(ra), 1)
xx = np.linspace(min(nc.min(), na.min()), max(nc.max(), na.max()), 20)
axes[0].plot(xx, np.polyval(z, xx), color=NAVY, lw=1.3, ls='--')
axes[0].set_xlabel('走行あたりの通信対象車数'); axes[0].set_ylabel('相対差 [%]')
axes[0].legend(frameon=False, fontsize=11)
axes[0].set_title('(a) 需要が多い走行ほど利得が大きい', fontsize=12.5, loc='left')
dd = (ra - rc)
axes[1].hist(dd, bins=14, color=ORANGE, edgecolor='white')
axes[1].axvline(0, color='#333333', lw=1.0)
axes[1].axvline(dd.mean(), color=NAVY, lw=1.4, ls='--')
axes[1].text(dd.mean(), axes[1].get_ylim()[1] * 0.92, f'  平均 {dd.mean():+.1f} pt',
             color=NAVY, fontsize=11.5)
axes[1].set_xlabel('相対差の変化 [pt]（同一の交通で対にした差）')
axes[1].set_ylabel('走行数')
axes[1].set_title(f'(b) {int((dd>0).sum())} / {len(dd)} 走行で増加', fontsize=12.5,
                  loc='left')
fig.suptitle('図N　通信対象を増やすと利得が増える', x=0.075, ha='left', fontsize=13.5)
fig.tight_layout(rect=[0, 0, 1, 0.93])
save(fig, 'N_demand.png')

# ---------------------------------------------------------------- 図O 学習曲線
lc = pd.read_csv(f'{R}/urban_cm_d70_learn/learning_curve.csv')
fig, ax = plt.subplots(figsize=(8.0, 4.0))
c1 = [c for c in lc.columns if 'p_trace' in c][0]
c2 = [c for c in lc.columns if 'alpha' in c and 'delta' in c][0]
xr = lc.index + 1 if 'run' not in lc.columns else lc['run']
ax.plot(xr, lc[c1], color=NAVY, lw=1.8, marker='o', ms=3.6, label='係数の不確かさ（P のトレース）')
ax.set_xlabel('学習走行数'); ax.set_ylabel('P のトレース', color=NAVY)
ax.tick_params(axis='y', labelcolor=NAVY)
ax2 = ax.twinx(); ax2.grid(False)
ax2.plot(xr, lc[c2], color=ORANGE, lw=1.8, marker='s', ms=3.6,
         label='1 走行あたりの係数変化 [dB]')
ax2.set_ylabel('係数の変化量 [dB]', color=ORANGE)
ax2.tick_params(axis='y', labelcolor=ORANGE)
ax2.spines['right'].set_visible(True)
h1, l1 = ax.get_legend_handles_labels(); h2, l2 = ax2.get_legend_handles_labels()
ax.legend(h1 + h2, l1 + l2, frameon=False, fontsize=11, loc='upper right')
ax.set_title('図O　地図の学習の推移（30 走行）', loc='left', fontsize=13.5, pad=12)
save(fig, 'O_learning.png')

# ---------------------------------------------------------------- 図F 評価窓
rows = []
for sd in sorted(glob.glob('/workspace/scratch/rec50/seed_*')):
    po = pd.read_csv(f'{sd}/poses.csv', usecols=['t_s', 'model', 'x']).dropna()
    v = po[po.model.str.startswith(('tx', 'tf'), na=False)].sort_values('t_s')
    mv = (v.groupby('model')['x'].diff().abs() > 1e-4).fillna(False)
    if not mv.any():
        continue
    rows.append((float(v.loc[mv, 't_s'].min()), float(po.t_s.max())))
rows.sort()
t0 = np.array([r[0] for r in rows]); te = np.array([r[1] for r in rows])
fig, ax = plt.subplots(figsize=(8.6, 4.6))
yy = np.arange(len(rows))
ax.barh(yy, te - t0, left=t0, color=LIGHT, height=0.72, label='記録された区間')
ax.barh(yy, np.minimum(35.0, te - t0), left=t0, color=ORANGE, height=0.72,
        label='評価に使う 35 秒')
ax.scatter(t0, yy, s=9, color=NAVY, zorder=3, label='走り出し')
ax.set_xlabel('シミュレーション時刻 [s]'); ax.set_ylabel('走行（走り出しの昇順）')
ax.legend(frameon=False, fontsize=11, loc='lower right')
ax.set_title('図F　走り出しを起点に評価窓を揃える', loc='left', fontsize=13.5, pad=12)
save(fig, 'F_window.png')

# ---------------------------------------------------------------- 図D 地図が返すもの
c = np.linspace(0, 210, 40); w = 210 / 39
# 学習済み地図は道路端で悪条件になり非現実的な値を返す (係数が大きく振動する)。
# 図には、窓全体が物理的に妥当な範囲に収まる区間のうち、最も強い場所を選ぶ。
HALF = 22.0
best = None
for f in sorted(glob.glob(f'{R}/urban_cm_d70_learn/rem_state/*_p_a*.npz')):
    dd = np.load(f); a2 = dd['alpha']
    for ctr in np.arange(HALF, 210 - HALF, 2.0):
        ss = np.linspace(ctr - HALF, ctr + HALF, 120)
        m2 = -110.0 + np.exp(-0.5 * ((ss[:, None] - c[None, :]) / w) ** 2) @ a2
        if m2.min() < -135 or m2.max() > -45:
            continue
        if best is None or m2.max() > best[0]:
            best = (m2.max(), f, ctr)
if best is None:
    raise SystemExit('妥当な区間が見つかりません')
_, fbest, ctr = best
print(f'    図D: {os.path.basename(fbest)} の s = {ctr-HALF:.0f}〜{ctr+HALF:.0f} m を描画')
d = np.load(fbest); al, P = d['alpha'], d['P']
s = np.linspace(ctr - HALF, ctr + HALF, 500)
Phi = np.exp(-0.5 * ((s[:, None] - c[None, :]) / w) ** 2)
mu = -110.0 + Phi @ al
sig = np.sqrt(np.maximum(np.einsum('ij,jk,ik->i', Phi, P, Phi), 0) + 4.0 ** 2)
lcb = mu - 1.0 * sig

fig, ax = plt.subplots(figsize=(8.8, 4.8))
ax.fill_between(s, mu - sig, mu + sig, color=LIGHT, label='± σ（予測の不確かさ）')
ax.plot(s, mu, color=NAVY, lw=2.0, label='予測値 μ')
ax.plot(s, lcb, color=ORANGE, lw=1.7, ls='--', label='LCB = μ − κσ（κ = 1）')
ax.axhline(-68.5, color='#333333', lw=1.3, ls=':')
lo = min(lcb.min(), -68.5) - 6
hi = max(mu.max() + sig.max(), -68.5) + 7
ax.set_ylim(lo, hi)
ax.text(s[0] + 0.5, -68.5 + 1.0, '接続閾値 −68.5 dBm', ha='left', fontsize=11.5)
# 接続可能と判定される区間を帯で示す
conn = mu >= -68.5
if conn.any():
    ax.fill_between(s, lo, hi, where=conn, color=ORANGE, alpha=0.10, lw=0)
    ax.text(s[conn].mean(), hi - 2.6, '接続可能と判定される区間', ha='center',
            fontsize=11, color=ORANGE)
# 基底の中心 (軸下端に固定)
tr = ax.get_xaxis_transform()
for cc in c:
    if s[0] <= cc <= s[-1]:
        ax.plot([cc], [0.0], marker='^', color=GRAY, ms=5.5, transform=tr,
                clip_on=False)
ax.set_xlabel('道路に沿った位置 [m]　▲ は RBF 基底の中心（間隔 5.4 m）')
ax.set_ylabel('受信電力 [dBm]')
ax.set_xlim(s[0], s[-1])
ax.legend(frameon=False, fontsize=11, loc='upper left', bbox_to_anchor=(0.0, 0.86))
ax.set_title('図D　地図が返すもの（学習済みの実データ）', loc='left', fontsize=13.5,
             pad=12)
save(fig, 'D_map.png')
print('完了')


# ================================================================ 図L 判定精度
print('図L を描いています:')
AUD = f'{R}/rem_audit_n50'
THR = -68.5


def load_samples(mode, frac=0.02, seed=0):
    """監査標本を間引いて読む (全 2,232 万行あるため)。"""
    rng = np.random.default_rng(seed)
    parts = []
    for ch in pd.read_csv(f'{AUD}/samples_{mode}.csv.gz', chunksize=1_000_000,
                          usecols=['mu_dbm', 'sigma_db', 'truth_dbm']):
        parts.append(ch.iloc[rng.random(len(ch)) < frac])
    return pd.concat(parts, ignore_index=True)


s0 = pd.read_csv(f'{AUD}/summary_frozen.csv').iloc[0]
sm = load_samples('frozen')
print(f'    標本 {len(sm):,} 行を使用')

fig, axes = plt.subplots(1, 2, figsize=(11.4, 4.7))

# --- (a) 予測と真値
ax = axes[0]
d2 = sm[(sm.truth_dbm > -140) & (sm.truth_dbm < -30) &
        (sm.mu_dbm > -140) & (sm.mu_dbm < -30)]
hb = ax.hexbin(d2.truth_dbm, d2.mu_dbm, gridsize=58, bins='log',
               cmap='Blues', mincnt=1, linewidths=0)
ax.plot([-140, -30], [-140, -30], color='#333333', lw=1.0)
ax.axhline(THR, color=ORANGE, lw=1.3, ls='--')
ax.axvline(THR, color=ORANGE, lw=1.3, ls='--')
ax.text(-66, -134, f'取りこぼし {s0.miss_pct:.1f}%', color=ORANGE, fontsize=11.5)
ax.text(-137, -62, f'幻リンク {s0.false_alarm_pct:.2f}%', color=ORANGE,
        fontsize=11.5)
ax.text(-137, -37, '破線は接続閾値 −68.5 dBm', fontsize=10, color=ORANGE)
ax.set_xlim(-140, -30); ax.set_ylim(-140, -30)
ax.set_xlabel('真値 [dBm]'); ax.set_ylabel('地図の予測 μ [dBm]')
ax.set_title(f'(a) 予測と真値（接続可能標本の RMSE '
             f'{s0.rmse_db_on_connectable:.1f} dB）', fontsize=12, loc='left')
cb = fig.colorbar(hb, ax=ax, fraction=0.045, pad=0.03)
cb.ax.set_title('標本数', fontsize=9.5, pad=6)

# --- (b) 不確かさの較正
ax = axes[1]
ok = sm[(sm.sigma_db > 0) & (sm.truth_dbm >= THR)]
z = ((ok.mu_dbm - ok.truth_dbm) / ok.sigma_db).clip(-4, 4)
ax.hist(z, bins=60, density=True, color=NAVY, alpha=0.85, edgecolor='none')
xx = np.linspace(-3, 3, 300)
ax.plot(xx, np.exp(-xx ** 2 / 2) / np.sqrt(2 * np.pi), color=ORANGE, lw=1.8)
ax.annotate('標準正規分布\n（σ が誤差と一致する場合）', xy=(1.35, 0.16),
            xytext=(1.5, 0.62), color=ORANGE, fontsize=10,
            arrowprops=dict(arrowstyle='->', color=ORANGE, lw=1.1))
ax.axvline(0, color='#333333', lw=1.0)
ax.set_xlim(-3, 3)
ax.text(0.02, 0.96, f'σ と |誤差| の順位相関 {s0.spearman_sigma_abserr:.2f}\n'
        f'（大きさは過大だが順序は合う）',
        transform=ax.transAxes, fontsize=10.5, va='top')
ax.set_xlabel('z = (μ − 真値) / σ'); ax.set_ylabel('確率密度')
ax.set_title('(b) 不確かさの較正（接続可能標本）', fontsize=12, loc='left')

fig.suptitle('図L　地図は接続可否を当てられるか（50 走行）', x=0.055, ha='left',
             fontsize=13.5)
fig.tight_layout(rect=[0, 0, 1, 0.91])
save(fig, 'L_calibration.png')
print('図L 完了')
