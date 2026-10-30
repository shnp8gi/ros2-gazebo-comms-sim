"""確定した測定条件と、全区間を重み付けした受信曲線を描画する。"""
import csv
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parent
plt.rcParams['font.family'] = 'Noto Sans CJK JP'
fig, axes = plt.subplots(1, 2, figsize=(12, 4.8), layout='constrained')
for metric, ax in zip(['per', 'ber'], axes):
    with (root / f'{metric}_relative.csv').open() as stream:
        rows = sorted(csv.DictReader(stream), key=lambda r: -float(r['relative_power_db']))
    x = [-float(r['relative_power_db']) for r in rows]
    y = [float(r[metric]) for r in rows]
    ax.plot(x, y, 'o-', ms=4)
    ax.axvspan(12, 14, color='orange', alpha=.15)
    ax.set_xlabel('外部減衰量 [dB]（絶対受信電力は未校正）')
    ax.set_ylabel('検出数で重み付けした近似PER' if metric == 'per' else '検出数で重み付けしたBER')
    if metric == 'ber':
        ax.set_yscale('symlog', linthresh=1e-10)
        ax.set_ylim(bottom=0)
    else:
        ax.set_ylim(-.03, 1.03)
    ax.grid(True, alpha=.25)
fig.suptitle('下り60.48 GHz（Q）・MCS 8・100 kB・clear write\n34条件／全4,900行を使用。PERのロスト数、BERのFEC前後は不明。')
fig.savefig(root / 'receiver_curves.png', dpi=170)
fig.savefig(root / 'receiver_curves.pdf')
