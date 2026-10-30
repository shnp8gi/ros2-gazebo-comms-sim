"""架空データによる、速度を色分けした従来形式のTimeout散布図例。"""
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.lines import Line2D
from matplotlib.patches import ConnectionPatch, Rectangle
from matplotlib import font_manager
from matplotlib.ticker import MaxNLocator
import numpy as np

from plot_timeout_layout_example import make_example, TITLES


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'sim_results/model12_speed_campaign_20261029/scatter_layout_example'
COLORS = ['#0072b2', '#dc8a00', '#009e73', '#ad69a0', '#d55e00']
MARKERS = ['o', 's', '^', 'D', 'v']
NAMES = ['parallel_fixed', 'parallel_tracking', 'cross_fixed', 'cross_tracking', 'cross_fixed_60ghz']


def draw_points(ax, model, pattern, positions, size):
    x, v, means, missing, mixed = make_example(model, pattern, positions)
    for j, (color, marker) in enumerate(zip(COLORS, MARKERS)):
        # 全試行未接続は接続成功の系列から除き、観測終了までの回数を×で示す。
        y = means[j]
        line = np.where(missing[j], np.nan, y)
        ax.plot(x[j], line, color=color, lw=.75, alpha=.65, zorder=2)
        good = ~missing[j]
        ax.scatter(x[j, good], y[good], marker=marker, s=size,
                   facecolors='none', edgecolors=color, linewidths=1.1, zorder=3)
        if missing[j].any():
            failed_counts = y + (2 if pattern == 0 else 1 if pattern == 4 else 0)
            ax.scatter(x[j, missing[j]], failed_counts[missing[j]], marker='x',
                       s=size+5, color=color, linewidths=1, zorder=4)
        if mixed[j].any():
            ax.scatter(x[j, mixed[j]], y[mixed[j]], marker='o', s=size+45,
                       facecolors='none', edgecolors='#707782', linewidths=.9, zorder=4)
    ax.grid(alpha=.2, color='#637385')
    ax.set_axisbelow(True)
    ax.tick_params(labelsize=10)
    for spine in ax.spines.values():
        spine.set_color('#a4aeb9')


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    available = {f.name for f in font_manager.fontManager.ttflist}
    japanese_font = next((f for f in ['MS Mincho', 'ＭＳ 明朝', 'MS明朝'] if f in available), 'Noto Serif CJK JP')
    latin_font = next((f for f in ['Times New Roman', 'TimesNewRomanPSMT'] if f in available), 'Liberation Serif')
    plt.rcParams.update({'font.family': japanese_font, 'pdf.fonttype': 3, 'axes.labelsize': 11,
                         'mathtext.fontset': 'custom', 'mathtext.rm': latin_font,
                         'mathtext.it': latin_font + ':italic', 'mathtext.bf': latin_font + ':bold'})
    (OUT / 'fonts_used.json').write_text(json.dumps(dict(japanese=japanese_font, latin=latin_font), ensure_ascii=False, indent=2))
    records = []
    with PdfPages(OUT / 'timeout_scatter_examples.pdf') as pdf:
        for pattern, name in enumerate(NAMES):
            fig, axes = plt.subplots(1, 2, figsize=(14, 7.5))
            fig.subplots_adjust(left=.075, right=.97, top=.73, bottom=.12, wspace=.17)
            fig.text(.075, .95, f'{TITLES[pattern]}｜開始距離と'+r'$\mathbf{Timeout}$', fontsize=21, weight='bold', color='#243349')
            fig.text(.075, .895, '表示確認用の仮データです。再計算結果・性能予測ではありません。', fontsize=12,
                     color='#953c33', bbox=dict(boxstyle='round,pad=.45', fc='#fff1ed', ec='#edcdc5'))
            handles = [Line2D([], [], color=c, marker=m, markerfacecolor='none', lw=.9,
                              markersize=6, label=f'{v} m/s') for v, c, m in zip(range(1, 6), COLORS, MARKERS)]
            legend = fig.legend(handles=handles, loc='upper center', bbox_to_anchor=(.52, .855), ncol=5,
                       frameon=False, prop={'family': latin_font, 'size': 12}, columnspacing=2.2,
                       title='各機の速度（①②で共通）', title_fontsize=11)
            legend.get_title().set_fontfamily(japanese_font)
            data = [make_example(m, pattern) for m in [1, 2]]
            top = max(float(d[2].max()) for d in data)
            top = max(6, top*1.13) if pattern in [0, 2, 4] else top*1.12
            if pattern == 0:
                top = max(10, top)  # 未接続の×印が拡大図の背後に隠れない余白。
            if pattern == 4:
                top *= 1.35
            zoom = [(2, 12), (15, 23), None, (24, 32), (9, 23)][pattern]
            for model, ax in enumerate(axes, start=1):
                ax.set_title('① しきい値モデル' if model == 1 else '② 確率モデル', fontsize=15, pad=13)
                draw_points(ax, model, pattern, np.arange(1, 61), 23)
                ax.set(xlim=(0, 61), ylim=(-top*.035, top), xlabel='各機の最接近地点までの開始距離 '+r'$\mathrm{[m]}$')
                ax.set_ylabel(('初回リンク確立までの' if model == 1 else '初回リンク確立までの平均')+r'$\mathrm{Timeout}$'+'回数')
                ax.yaxis.set_major_locator(MaxNLocator(integer=True, nbins=6))
                if zoom is not None:
                    # 小図はデータと重ならない上部に配置。両モデルとも同じ拡大範囲。
                    inset = ax.inset_axes([.085, .62, .53, .30] if pattern == 0 else [.085, .55, .53, .36])
                    inset.set_facecolor('white')
                    lo, hi = zoom
                    fine_x = np.round(np.arange(lo, hi+.01, .2), 1)
                    draw_points(inset, model, pattern, fine_x, 17)
                    inset.set(xlim=(lo, hi), ylim=(-.12, 3.3), yticks=[0, 1, 2, 3])
                    inset.tick_params(labelsize=8)
                    inset.xaxis.set_major_locator(MaxNLocator(nbins=4, integer=True))
                    # 文字の説明に代え、拡大範囲の枠と対応する角同士の線を描く。
                    outline = '#596575'
                    ax.add_patch(Rectangle((lo, -.12), hi-lo, 3.42, facecolor='none',
                                           edgecolor=outline, linewidth=1.3, zorder=4))
                    for source_x, target_x in [(lo, 0), (hi, 1)]:
                        ax.add_artist(ConnectionPatch(xyA=(source_x, 3.3), coordsA=ax.transData,
                            xyB=(target_x, 0), coordsB=inset.transAxes, color=outline,
                            linewidth=.9, alpha=.85, zorder=1.5, clip_on=False))
                    for spine in inset.spines.values():
                        spine.set_color(outline)
                        spine.set_linewidth(1.3)
                else:
                    ax.text(.5, .55, '全条件で未接続の場合の表示例\n×印の0回は、接続成功ではありません',
                            transform=ax.transAxes, ha='center', fontsize=12, color='#526071', linespacing=1.8)
                x, v, mean, missing, mixed = data[model-1]
                for j, i in np.ndindex(x.shape):
                    records.append(dict(illustrative_only=True, model=model, pattern=name,
                        speed_mps=int(v[j, i]), start_path_m=int(x[j, i]),
                        timeouts=float(mean[j, i])+(2 if pattern == 0 else 1 if pattern == 4 else 0) if missing[j, i] else float(mean[j, i]),
                        status='unconnected' if missing[j, i] else 'partly_unconnected' if mixed[j, i] else 'connected'))
            for ax in axes:
                for current in [ax, *ax.child_axes]:
                    for label in [*current.get_xticklabels(), *current.get_yticklabels()]:
                        label.set_fontfamily(latin_font)
            fig.savefig(OUT / f'{name}_example.png', dpi=160, bbox_inches='tight')
            pdf.savefig(fig, bbox_inches='tight')
            plt.close(fig)
    with (OUT / 'illustrative_scatter_data_NOT_SIMULATION.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)
    (OUT / 'README_ja.txt').write_text(
        '速度別の散布図の構成例\n\n全数値は架空データです。実際のシミュレーションは実行していません。\n'
        '1パターンにつき1枚、①②を左右に配置し、速度1〜5 m/sを色と点形状で区別。\n'
        '軸範囲と速度の色は①②で共通。拡大範囲を元図の四角と接続線で示す。下部の灰色注釈は表示しない。\n'
        '未接続は×、一部の試行で未接続は灰色の輪で区別。\n'
        '5パターンのPNG、および5ページのPDFを保存。\n'
        '拡大図も架空値であり、実際の0.1 m刻みの境界を示すものではありません。\n'
        f'使用フォント：日本語 {japanese_font}、英数字 {latin_font}。\n'
        'MS明朝・Times New Romanが利用可能な環境では優先して使用する。\n'
        '再生成：python3 tools/plot_timeout_scatter_example.py\n')
    print(OUT)


if __name__ == '__main__':
    main()
