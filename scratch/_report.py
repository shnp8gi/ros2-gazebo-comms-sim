import sys
import numpy as np, pandas as pd
from scipy.stats import wilcoxon, t as tdist
tag = sys.argv[1] if len(sys.argv) > 1 else 'ablation_n50'
d = pd.read_csv(f'sim_results/{tag}/analysis/runs.csv')
p = d.pivot_table(index='run', columns='method', values='total_data_MB')
def pr(a, b='assoc_hold'):
    x = (p[a] - p[b]).dropna()
    ci = tdist.ppf(0.975, len(x)-1) * x.std(ddof=1) / np.sqrt(len(x))
    return x, x.mean(), ci, wilcoxon(x)[1]
print(f'=== {tag} :  N = {len(p)} 走行 / 閾値 -68.5 dBm ===\n')
print('■ 受動接続に対する優位')
for a in ['oracle', 'kkf_cold_probe', 'kkf_conv']:
    x, m, ci, pv = pr(a)
    rel = 100 * (x / p['assoc_hold'])
    print(f'  {a:15s} {m:+9.1f} ±{ci:6.1f} MB / 走行ごと相対差 {rel.mean():+6.2f}% / p={pv:.2g}')
x, gain, ci, pv = pr('kkf_conv'); orc = pr('oracle')[1]
rel = 100 * (x / p['assoc_hold'])
print(f'\n  到達率 {100*gain/orc:.1f}%   全走行で優位: {(x>0).all()}   '
      f'最小 {rel.min():+.2f}% / 最大 {rel.max():+.2f}%')
print('\n■ 機構を1つずつ外したときの対差 (正 = 効いている)')
for a, lab in [('kkf_novar','遮蔽リスク地図 σν²'), ('kkf_nolcb','LCB (κσ)'),
               ('kkf_nokrig','残差クリギング'), ('kkf_frozen','走行内の地図更新')]:
    y = (p['kkf_conv'] - p[a]).dropna()
    c = tdist.ppf(0.975, len(y)-1) * y.std(ddof=1) / np.sqrt(len(y))
    print(f'  {lab:22s} {y.mean():+8.1f} ±{c:5.1f} MB / 利得の {100*y.mean()/gain:+5.1f}% / p={wilcoxon(y)[1]:.2g}')
a = pd.read_csv(f'sim_results/{tag}/analysis/agg.csv').set_index('method')
print('\n■ 公平性')
for m in ['assoc_hold','kkf_conv','oracle']:
    print(f'  {m:15s} 最小車両 {a.loc[m,"min_car_data_MB_mean"]:6.1f} MB / '
          f'0 MB 車両 {a.loc[m,"zero_car_pct_mean"]:5.2f}%')
g = d.groupby('method')[['connected_time_s','grant_time_s']].mean()
print('\n■ grant 効率')
for m in ['assoc_hold','kkf_conv','oracle']:
    print(f'  {m:15s} {100*g.loc[m,"connected_time_s"]/g.loc[m,"grant_time_s"]:.1f}%')
print(f'\n  対象車数: 中央 {d[d.method=="assoc_hold"].n_cars.median():.0f} / '
      f'範囲 {d[d.method=="assoc_hold"].n_cars.min():.0f}-{d[d.method=="assoc_hold"].n_cars.max():.0f}')
