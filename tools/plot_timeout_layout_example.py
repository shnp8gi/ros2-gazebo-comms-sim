"""速度×開始距離のレポート図を、実験結果と分離した架空データで例示する。"""
import csv
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm
from matplotlib.patches import Patch, Rectangle
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'sim_results/model12_speed_campaign_20261029/figure_layout_example'
TITLES = ['並行・固定', '並行・追従', '十字・固定｜従来の無線条件',
          '十字・追従', '十字・固定｜追加の無線条件']
COLORS = ['#64b9a5', '#fff0bc', '#fbd18b', '#f99b64', '#d85b58', '#8d304d']
GRAY = '#dce2e9'


def make_example(model, pattern, positions=None):
    """見せ方のための数値。電波伝搬・プロトコルを計算した値ではない。"""
    x, v = np.meshgrid(np.arange(1, 61) if positions is None else positions, np.arange(1, 6))
    missing = np.zeros_like(x, dtype=bool)
    mixed = np.zeros_like(x, dtype=bool)
    if pattern == 0:
        mean = np.where(x <= 5, 0, np.maximum(0, 4 - v)).astype(float)
        missing = (x > 53) | ((x >= 7) & (x <= 10) & (v <= 2))
        if model == 2:
            mean = np.maximum(0, mean - .5)
            mixed = (x >= 50) & (x <= 53)
    elif pattern in [1, 3]:
        boundary = 17 if pattern == 1 else 26
        if model == 2:
            boundary = boundary + .5 * v
        excess = np.maximum(0, x - boundary)
        mean = np.ceil(excess / (v * .5)).astype(float)
        if model == 2:
            mean = np.where(excess > 0, np.maximum(.15, mean - .25), 0)
    elif pattern == 2:
        mean = np.zeros_like(x, dtype=float)
        missing[:] = True
    else:
        boundary = 9 + 2 * v + (2 if model == 2 else 0)
        mean = np.ceil(np.maximum(0, x - boundary) / (2 * v)).astype(float)
        missing = x > 40 + 2 * v
        if model == 2:
            mean = np.where(mean > 0, np.maximum(.1, mean - .4), 0)
            mixed = (x > 37 + 2 * v) & ~missing
    return x, v, mean, missing, mixed


def draw(model):
    fig, axes = plt.subplots(2, 3, figsize=(15, 8.6))
    fig.patch.set_facecolor('#fafbfc')
    fig.subplots_adjust(left=.065, right=.98, top=.82, bottom=.16, wspace=.24, hspace=.42)
    name = '① しきい値モデル' if model == 1 else '② 確率モデル'
    fig.text(.065, .945, f'{name}｜速度と開始距離から見るTimeout', fontsize=21, weight='bold', color='#233248')
    fig.text(.065, .897, '図の構成例・すべて架空データです。再計算結果や通信性能の予測ではありません。',
             fontsize=12, color='#983535', bbox=dict(boxstyle='round,pad=.5', fc='#fff0ed', ec='#e9c2bb'))
    cmap = ListedColormap(COLORS)
    cmap.set_bad(GRAY)
    norm = BoundaryNorm(np.arange(-.5, 6.5), cmap.N)
    records = []
    for pattern, ax in enumerate(axes.flat[:5]):
        x, v, mean, missing, mixed = make_example(model, pattern)
        classes = np.where(mean == 0, 0, np.where(mean <= 1, 1, np.where(mean <= 2, 2,
                  np.where(mean <= 5, 3, np.where(mean <= 10, 4, 5)))))
        ax.pcolormesh(np.arange(.5, 61.5), np.arange(.5, 6.5),
                      np.ma.array(classes, mask=missing), cmap=cmap, norm=norm, shading='flat')
        # 試行の一部だけ未接続のセルには、平均の色の上から斜線を重ねる。
        for j, i in np.argwhere(mixed):
            ax.add_patch(Rectangle((i+.5, j+.5), 1, 1, fill=False, hatch='////',
                                   edgecolor='#46566a', linewidth=0, zorder=4))
        zero = (mean == 0) & ~missing & ~mixed
        segments = []
        for j in range(5):
            for i in range(59):
                if zero[j, i] != zero[j, i+1]:
                    segments.append([(i+1.5, j+.5), (i+1.5, j+1.5)])
        ax.add_collection(LineCollection(segments, colors='#163f3c', linewidths=2, zorder=5))
        for y in np.arange(1.5, 5.5):
            ax.axhline(y, color='white', lw=.8, alpha=.8)
        ax.set_title(TITLES[pattern], loc='left', fontsize=12.5, weight='bold', pad=12, color='#233248')
        ax.set(xlim=(.5, 60.5), ylim=(.5, 5.5), xticks=[1, 10, 20, 30, 40, 50, 60], yticks=range(1, 6),
               xlabel='各機の最接近地点までの開始距離 [m]', ylabel='各機の速度 [m/s]')
        ax.tick_params(length=0, labelsize=10, pad=6)
        for spine in ax.spines.values():
            spine.set_color('#a6b0bc')
        if missing.all():
            ax.text(.5, .5, '全試行で未接続の表示例\nTimeout 0回の接続成功とは区別',
                    transform=ax.transAxes, ha='center', va='center', fontsize=11, color='#526071')
        for j, i in np.ndindex(x.shape):
            records.append(dict(illustrative_only=True, model=model, pattern=TITLES[pattern],
                speed_mps=int(v[j, i]), start_path_m=int(x[j, i]),
                mean_timeouts='' if missing[j, i] else float(mean[j, i]),
                connection_status='none' if missing[j, i] else 'mixed' if mixed[j, i] else 'all'))
    legend = axes.flat[5]
    legend.set_axis_off()
    legend.text(0, 1.04, '①②で共通の色・軸を使用', transform=legend.transAxes,
                fontsize=13, weight='bold', color='#233248')
    labels = ['0回で接続', '0回超〜1回', '1回超〜2回', '2回超〜5回', '5回超〜10回', '10回超']
    handles = [Patch(facecolor=c, edgecolor='#b3bbc4', label=s) for c, s in zip(COLORS, labels)]
    handles += [Patch(facecolor=GRAY, label='全試行で未接続'),
                Patch(facecolor='white', hatch='////', edgecolor='#46566a', label='一部の試行で未接続（②）')]
    legend.legend(handles=handles, loc='upper left', bbox_to_anchor=(-.025, .94),
                  ncol=2, frameon=False, fontsize=10.2, handleheight=1.6, labelspacing=1.05,
                  columnspacing=1.1, title='色：初回リンク確立までの平均Timeout回数', title_fontsize=10.5)
    legend.text(0, .12, '太線：各速度での0回接続領域の端\n細かな数値・試行数は別表／CSVへ',
                transform=legend.transAxes, fontsize=10.5, color='#526071', linespacing=1.6)
    fig.text(.065, .038, '表示例は各機1〜5 m/s・開始距離1〜60 m。各速度の計算点を区画で示し、速度間を滑らかに補間しません。\n'
             '追加の無線条件：双方向60 GHz・−7 dBm、PRDEV相当の回路条件からLNAを除外する計画。全パネルの数値は架空です。',
             fontsize=10.5, color='#526071', linespacing=1.6)
    return fig, records


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({'font.family': 'Noto Sans CJK JP', 'axes.labelsize': 10,
                         'pdf.fonttype': 3, 'svg.fonttype': 'none'})
    records = []
    with PdfPages(OUT / 'timeout_layout_example.pdf') as pdf:
        for model in [1, 2]:
            fig, rows = draw(model)
            records.extend(rows)
            fig.savefig(OUT / f'model_{model}_layout_example.png', dpi=160, bbox_inches='tight')
            pdf.savefig(fig, bbox_inches='tight')
            plt.close(fig)
    with (OUT / 'illustrative_data_NOT_SIMULATION.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)
    (OUT / 'README_ja.txt').write_text(
        '速度×開始位置の図の構成例\n\n全数値は表示確認用の架空データ。シミュレーション結果・推定性能ではありません。\n'
        '実際の計算設定・既存の結果は変更していません。\n'
        '①②それぞれ5パターンを2×3配置にし、最後の区画を共通凡例に使用。\n'
        '両モデルの軸範囲と色の区分は統一。0回接続は緑、正値は段階色、未接続は灰、一部未接続は斜線。\n'
        '未接続を成功0回として描かない。速度間の未計算条件は補間しない。\n'
        '再生成：python3 tools/plot_timeout_layout_example.py\n')
    print(OUT)


if __name__ == '__main__':
    main()
