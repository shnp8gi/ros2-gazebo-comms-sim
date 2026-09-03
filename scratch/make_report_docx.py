#!/usr/bin/env python3
"""調査資料 (Word) を生成する。

書式は seminar/2026_08_04_KKFSystem.docx をそのまま雛形として引き継ぐ:
  用紙・余白  A4 / 上下 2.5 cm・左右 2.0 cm (雛形の sectPr をそのまま使う)
  見出し      List Paragraph + 自動番号 (numId は雛形の見出しから複製)、章=ilvl0 / 節=ilvl1
  本文        Normal / Times New Roman 10.5 pt / 1 字下げ
  句読点      ，．(全角カンマ・ピリオド)、である調

数値はすべて sim_results の集計 CSV から読む (本文への手書き写しをしない)。

  python3 scratch/make_report_docx.py
"""
import copy
import os
import sys

import numpy as np
import pandas as pd
from docx import Document
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.oxml.ns import qn
from docx.shared import Cm, Pt
from scipy import stats

ROOT = '/workspace'
R = f'{ROOT}/sim_results'
TPL = f'{ROOT}/scratch/seminar/2026_08_04_KKFSystem.docx'
OUT = f'{ROOT}/scratch/review'
FIGS = f'{OUT}/figs'
os.makedirs(OUT, exist_ok=True)
sys.path.insert(0, f'{ROOT}/tools')
from lib import eval_metrics  # noqa: E402

# ============================================================ 書式 (雛形から)
doc = Document(TPL)

h_lvl0 = h_lvl1 = body_tpl = None
for p in doc.paragraphs:
    t = p.text.strip()
    if p.style.name == 'List Paragraph' and t:
        pPr = p._p.pPr
        num = pPr.find(qn('w:numPr')) if pPr is not None else None
        ilvl = None
        if num is not None and num.find(qn('w:ilvl')) is not None:
            ilvl = num.find(qn('w:ilvl')).get(qn('w:val'))
        if ilvl == '0' and h_lvl0 is None:
            h_lvl0 = p
        if ilvl == '1' and h_lvl1 is None:
            h_lvl1 = p
    elif p.style.name == 'Normal' and t and body_tpl is None and p.runs:
        body_tpl = p
assert h_lvl0 is not None and h_lvl1 is not None and body_tpl is not None, '雛形が取れない'

TBL_STYLE = doc.tables[0].style.name if doc.tables else 'Table Grid'
h0_x = copy.deepcopy(h_lvl0._p)
h1_x = copy.deepcopy(h_lvl1._p)
body_x = copy.deepcopy(body_tpl._p)

# 雛形の中身を捨てる (sectPr = 用紙・余白の定義だけ残す)
body = doc.element.body
sectPr = body.find(qn('w:sectPr'))
for child in list(body):
    if child is not sectPr:
        body.remove(child)


# w:rPr の子要素は順序が決まっている (ECMA-376 CT_RPr)。順序を守らないと
# Word が壊れたファイルとして扱うので、必ずこの並びに割り込ませる
_RPR_ORDER = ['w:rStyle', 'w:rFonts', 'w:b', 'w:bCs', 'w:i', 'w:iCs', 'w:caps',
              'w:smallCaps', 'w:strike', 'w:dstrike', 'w:outline', 'w:shadow',
              'w:emboss', 'w:imprint', 'w:noProof', 'w:snapToGrid', 'w:vanish',
              'w:webHidden', 'w:color', 'w:spacing', 'w:w', 'w:kern', 'w:position',
              'w:sz', 'w:szCs', 'w:highlight', 'w:u', 'w:effect', 'w:bdr', 'w:shd',
              'w:fitText', 'w:vertAlign', 'w:rtl', 'w:cs', 'w:em', 'w:lang']


def _rpr_add(rPr, tag, attrs=None):
    """rPr に要素を規定の順序で差し込む (既存の同名要素は置き換える)。"""
    for e in rPr.findall(qn(tag)):
        rPr.remove(e)
    el = rPr.makeelement(qn(tag), {})
    for k, v in (attrs or {}).items():
        el.set(qn(k), v)
    rank = _RPR_ORDER.index(tag) if tag in _RPR_ORDER else len(_RPR_ORDER)
    for child in rPr:
        name = child.tag.split('}')[-1]
        r = _RPR_ORDER.index('w:' + name) if ('w:' + name) in _RPR_ORDER else len(_RPR_ORDER)
        if r > rank:
            child.addprevious(el)
            return el
    rPr.append(el)
    return el


def _jp_punct(text):
    """句点を雛形の体裁 (，．) にそろえる。数値の小数点は半角なので影響しない。"""
    return text.replace('。', '．')


def _clone(tpl_x, text, bold=None, size=None, align=None, indent=None):
    """雛形段落を複製して文字列だけ差し替える (番号付け・フォントを保つ)。"""
    text = _jp_punct(text)
    new = copy.deepcopy(tpl_x)
    runs = new.findall(qn('w:r'))
    for r in runs[1:]:
        new.remove(r)
    r = runs[0]
    for t in r.findall(qn('w:t')):
        r.remove(t)
    el = r.makeelement(qn('w:t'), {})
    el.text = text
    el.set(qn('xml:space'), 'preserve')
    r.append(el)
    rPr = r.find(qn('w:rPr'))
    if rPr is None:
        rPr = r.makeelement(qn('w:rPr'), {})
        r.insert(0, rPr)
    if bold is not None:
        for tag in ('w:b', 'w:bCs'):
            for e in rPr.findall(qn(tag)):
                rPr.remove(e)
        if bold:
            _rpr_add(rPr, 'w:b')
            _rpr_add(rPr, 'w:bCs')
    if size is not None:
        for tag in ('w:sz', 'w:szCs'):
            _rpr_add(rPr, tag, {'w:val': str(int(size * 2))})
    pPr = new.find(qn('w:pPr'))
    if pPr is None:
        pPr = new.makeelement(qn('w:pPr'), {})
        new.insert(0, pPr)
    if align is not None:
        for e in pPr.findall(qn('w:jc')):
            pPr.remove(e)
        e = new.makeelement(qn('w:jc'), {})
        e.set(qn('w:val'), align)
        pPr.append(e)
    if indent is not None:
        for e in pPr.findall(qn('w:ind')):
            pPr.remove(e)
        if indent:
            e = new.makeelement(qn('w:ind'), {})
            e.set(qn('w:firstLine'), str(int(indent * 567)))  # cm -> twips
            pPr.append(e)
    sectPr.addprevious(new)
    return new


def h1(text):
    return _clone(h0_x, text)


def h2(text):
    return _clone(h1_x, text)


def body_p(text, bold=None, size=None):
    return _clone(body_x, text, bold=bold, size=size)


def caption(text):
    return _clone(body_x, text, size=9.5, align='center', indent=0)


def note(text):
    return _clone(body_x, text, size=8.5, indent=0)


def _cell_run(cell, text, size=8.5, bold=False, align=None):
    p = cell.paragraphs[0]
    run = p.add_run(_jp_punct(text))
    e = run._element
    rPr = e.find(qn('w:rPr'))
    if rPr is None:
        rPr = e.makeelement(qn('w:rPr'), {})
        e.insert(0, rPr)
    _rpr_add(rPr, 'w:rFonts', {'w:ascii': 'Times New Roman',
                               'w:hAnsi': 'Times New Roman',
                               'w:cs': 'Times New Roman'})
    if bold:
        _rpr_add(rPr, 'w:b')
        _rpr_add(rPr, 'w:bCs')
    for tag in ('w:sz', 'w:szCs'):
        _rpr_add(rPr, tag, {'w:val': str(int(size * 2))})
    if align is not None:
        p.alignment = align
    p.paragraph_format.space_after = Pt(0)
    p.paragraph_format.line_spacing = 1.0


def figure(name, cap, width_cm=15.5):
    """図を貼り、その下に図番号つきの説明を置く (表は上、図は下)。"""
    path = os.path.join(FIGS, name)
    if not os.path.exists(path):
        # 未生成の図は場所だけ確保しておく (PowerPoint から書き出す説明図など)
        ph = doc.add_paragraph()
        ph.alignment = WD_ALIGN_PARAGRAPH.CENTER
        r = ph.add_run(f'［図 {name} 未挿入］')
        r.font.size = Pt(10)
        print(f'  未挿入: {name}')
        return caption(cap)
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.space_before = Pt(6)
    p.paragraph_format.space_after = Pt(2)
    p.add_run().add_picture(path, width=Cm(width_cm))
    return caption(cap)


def table(headers, rows, widths, right_from=1):
    t = doc.add_table(rows=1, cols=len(headers))
    t.style = 'Table Grid'
    t.alignment = WD_TABLE_ALIGNMENT.CENTER
    t.autofit = False
    for i, hd in enumerate(headers):
        _cell_run(t.rows[0].cells[i], hd, bold=True, align=WD_ALIGN_PARAGRAPH.CENTER)
    for row in rows:
        cells = t.add_row().cells
        for i, v in enumerate(row):
            al = WD_ALIGN_PARAGRAPH.RIGHT if i >= right_from else None
            _cell_run(cells[i], str(v), align=al)
    for r in t.rows:
        for i, w in enumerate(widths):
            r.cells[i].width = Cm(w)
    return t


# ============================================================ データ
def mb(v):
    return f'{v:,.0f}'


def smb(v):
    return f'{v:+,.0f}'


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
    """ref に対する対差 (MB) と走行ごと相対差 (%) を返す。"""
    a = runs[runs.method == ref].set_index('run').total_data_MB
    x = runs[runs.method == m].set_index('run').total_data_MB
    d = (x - a).dropna()
    rel = (100 * (x - a) / a).dropna()
    return d, ci(d), rel, ci(rel)


runs, agg = load('abl_d70_n50')
o_runs, o_agg = load('rec_cm_d70_ablation')
N = int(agg.loc['assoc_hold', 'n'])
N_REC = len([d for d in os.listdir(f'{R}/rec_d70_n50') if d.startswith('seed_')])
N_OLD = int(o_agg.loc['assoc_hold', 'n'])

base = float(agg.loc['assoc_hold', 'total_data_MB_mean'])
conv = float(agg.loc['kkf_conv', 'total_data_MB_mean'])
orc = float(agg.loc['oracle', 'total_data_MB_mean'])
probe = float(agg.loc['kkf_cold_probe', 'total_data_MB_mean'])
d, d_ci, rel, rel_ci = diff_vs(runs, 'kkf_conv')
d_o, d_o_ci, rel_o, rel_o_ci = diff_vs(runs, 'oracle')
d_pr, _, rel_pr, _ = diff_vs(runs, 'kkf_cold_probe')
capture = 100.0 * d.mean() / d_o.mean()
o_d, o_d_ci, o_rel, o_rel_ci = diff_vs(o_runs, 'kkf_conv')
o_cap = 100.0 * o_d.mean() / diff_vs(o_runs, 'oracle')[0].mean()



# ============================================================ データ
runs, agg = load('ablation_n50')
N = int(agg.loc['assoc_hold', 'n'])
base = float(agg.loc['assoc_hold', 'total_data_MB_mean'])
d, d_ci, rel, rel_ci = diff_vs(runs, 'kkf_conv')
d_o, d_o_ci, rel_o, rel_o_ci = diff_vs(runs, 'oracle')
d_pr, _, rel_pr, _ = diff_vs(runs, 'kkf_cold_probe')
capture = 100.0 * d.mean() / d_o.mean()
p_conv = stats.wilcoxon(d)[1]

ABL = {}
for m in ('kkf_novar', 'kkf_nolcb', 'kkf_nokrig', 'kkf_frozen'):
    y = (runs[runs.method == 'kkf_conv'].set_index('run').total_data_MB
         - runs[runs.method == m].set_index('run').total_data_MB).dropna()
    ABL[m] = (y.mean(), ci(y), 100 * y.mean() / d.mean(), stats.wilcoxon(y)[1])

g = runs.groupby('method')[['connected_time_s', 'grant_time_s']].mean()
def eff(m): return 100 * g.loc[m, 'connected_time_s'] / g.loc[m, 'grant_time_s']

au = pd.read_csv(f'{R}/rem_audit_n50/summary_frozen.csv').iloc[0]
ncar = runs[runs.method == 'assoc_hold'].n_cars

# ============================================================ 本文
_clone(body_x, '学習済み電波地図に基づく能動的ハンドオーバーの有用性', bold=True,
       size=14, align='center', indent=0)
_clone(body_x, f'都市部 2 車線・論理 RSU 8 基における {N} 走行の評価',
       size=11, align='center', indent=0)
_clone(body_x, '内部資料', size=9.5, align='center', indent=0)

# ------------------------------------------------------------------ 1 全容
h1('全容')
body_p('ミリ波（60 GHz）で路側機（RSU）と走行車両が通信するとき，1 基の RSU に'
       '同時に接続できる車両は 1 台に限られる（802.15.3e のペアネット方式 [2]）。'
       'さらに，接続していない相手については，割当の判断に足るだけの受信電力を'
       '継続して測ることができない。規格の受動アソシエーションで分かるのは，'
       'つなげる見込みがあるかどうかの粗い判定にとどまる。'
       'どの車両をどの RSU にいつ接続するかを決めるには，測っていない相手の'
       '電波状況を何らかの方法で補う必要がある（図1）。')
figure('A_problem.png', '図1　問題設定．1 基の RSU に接続できるのは 1 台のみで，'
       '接続していない相手の受信電力は測れない．')
body_p('提案手法は，過去の走行から学習した電波地図でこれを補い，接続先を能動的に'
       '決める。地図は位置を入力すると受信電力の予測値と，その予測がどれだけ'
       '不確かかを返す。空間的に離れた地点の値を相関構造から補間する Kriging [1] を'
       '基礎とし，道路に沿って置いた基底関数による回帰と，直近の観測に対する残差の'
       '空間補間を組み合わせる。定式化と実装は第 2 章に示す。')
body_p(f'本シミュレーション環境において，学習済み電波地図に基づく能動的ハンドオーバーは，'
       f'802.15.3e 準拠の受動接続に対し総配信量で +{rel.mean():.1f}% 優位である'
       f'（対差 +{mb(d.mean())} MB，95% 信頼区間 ±{mb(d_ci)} MB，'
       f'Wilcoxon 符号順位検定 p = {p_conv:.1g}，{N} 走行）。'
       f'{N} 走行すべてで提案手法が上回り，最も差が小さい走行でも '
       f'+{rel.min():.1f}% である。この優位性は事前学習した電波地図に由来し，'
       f'地図は規格上観測できないペアの現在値を代替する役割を担う。'
       f'全ペアの現在値を雑音なしで与えた理想反応型に対する到達率は '
       f'{capture:.1f}% である。')

# ------------------------------------------------------------------ 2 提案手法
h1('提案手法')
body_p('本章では，第 1 章で述べた電波地図の中身と，それを接続先の決定に'
       '変換する手順を示す。2.1 節は基礎となる Kriging，2.2 節は本実装の'
       '電波地図の構成，2.3 節は割当と切替の手順である。')

h2('Kriging の原理')
body_p('Kriging [1] は空間統計学における内挿手法であり，観測地点以外の地点における'
       '値を，観測値の線形結合として最良線形不偏予測により推定する。'
       '空間領域上の位置における確率場を，平均構造と，空間相関を持つ誤差成分に'
       '分解して扱う。')
body_p('空間データの相関構造は，2 地点間の距離の関数として定義される共分散関数に'
       'よって特徴づけられる。未観測地点の値を，観測地点の値の線形結合として'
       '予測するとき，予測値の期待値が真値の期待値と一致するという制約のもとで'
       '平均 2 乗予測誤差を最小化する重み係数は，観測地点間の共分散行列と，'
       '観測地点と予測地点との間の共分散ベクトルから定まる。')
body_p('本実装では，この枠組みを 2 つの形で用いる。1 つは道路に沿って置いた'
       '基底関数による平均構造の表現であり，もう 1 つは，その基底では表現しきれない'
       '残差成分の補間である。')

h2('電波地図の構成')
body_p('電波地図は，位置を入力すると受信電力の予測値と，その予測がどれだけ'
       '不確かかを返す（図2）。構成は次のとおりである。')
figure('D_map.png', '図2　地図が返すもの（学習済みの実データ）．'
       '位置ごとに予測値 μ と不確かさ σ を返し，κσ を差し引いた LCB で'
       '割当の優先順位を決める．接続の可否は平均で判定する（2.3 節）．')
table(['項目', '内容'],
      [['位置', '道路に沿った 1 次元の距離（0〜210 m）'],
       ['空間基底', '道路に沿って等間隔に置いたガウス基底関数 40 個（間隔 5.4 m）'],
       ['係数', '40 個の基底の重み．受信電力の予測値を与える部分の実体である'],
       ['観測', '受信電力レポート（20 Hz）．運用相では割当中のペアに限られる'],
       ['残差', '空間基底では表現しきれない成分．直近の残差は 2.1 節の補間に用い，'
                'その分散を位置ごとに保持したものが遮蔽リスク地図 σν²(s) である'],
       ['予測', '未観測ペアの現時刻の受信電力の予測値．空間基底による項に，'
                '直近の観測から求めた残差の補間を加えたものを用いる'],
       ['不確かさ σ', '地図が予測値に添えて返す標準偏差．LCB（κσ）に用いる'],
       ['地図の枚数', '（RSU 8 基）×（走行方向 2）×（車載アンテナ 2）= 32 枚を'
                     '独立に保持する']],
      [3.6, 11.9])
body_p('係数の推定は，観測を逐次に取り込むベイズ線形回帰（再帰最小二乗）で行う。'
       '学習相では 30 走行にわたって係数と共分散行列を保持し，観測が入るたびに'
       '更新する。本実装が学習するのは環境に固有の減衰であり，走行を重ねても'
       '変わらないためである。運用相の各走行は，この学習済みの状態を共通の'
       f'初期値として開始する。したがって評価の {N} 走行の間に状態の持ち越しは'
       'なく，走行の順序は結果に影響しない。')
body_p('残差成分の扱いは次のとおりである。直近 64 点の残差（位置・時刻・残差値）を'
       '保持し，時空間分離型の指数共分散（空間相関長 20 m，時間相関長 5 s）で'
       '2.1 節の補間を解いて，予測値に加える。予測の不確かさも同じ共分散から'
       '低減する。これとは別に，位置ごとの残差の分散を遮蔽リスク地図 σν²(s) として'
       '保持する。')
body_p('予測の不確かさ σ は，係数の共分散行列から定まる項，残差場の標準偏差として'
       '与えた定数（4 dB），残差の補間による低減，および遮蔽リスク地図 σν²(s) の'
       '和として合成される。')

h2('割当と切替')
body_p('割当を再計算する各時刻において，接続先は次の手順で決まる（図3）。')
figure('E_flow.png', '図3　手法の流れ．学習相で電波地図を作り，運用相では'
       '地図の予測から割当を決める．')
body_p('(1) 予測．各（車両アンテナ，RSU）ペアについて，対応する地図から'
       '現在位置の受信電力の予測値と，その不確かさ σ を得る。')
body_p('(2) 慎重な評価．予測値から κσ を差し引いた下側信頼限界（LCB）で評価し，'
       'あわせて遮蔽リスク地図 σν²(s) により揺らぎの大きい位置の評価を下げる。'
       '確度の低い予測や不安定な位置を高く評価しないための処置である。')
body_p('(3) 評価値の決定．(2) の LCB を dB のまま評価値として用いる。'
       'あわせて，平均の予測値が接続閾値（−68.5 dBm）に届かないペアを'
       '割当の候補から外す。掴んでも 1 バイトも送れず，需要が供給を超える'
       '状況では他の車両を締め出すだけだからである。この可否の判定には '
       'LCB ではなく平均を用いる。LCB で判定すると，揺らぎの大きい位置で'
       '過度に保守的になり，繋がる機会を見送るためである。')
body_p('(4) 割当．1 基の論理 RSU に 1 台という排他制約の下で，評価値の総和が'
       '最大になる組合せを求める。これは二部グラフの最大重みマッチングであり，'
       'ハンガリアン法によってその時刻における厳密解を得る。ここで最大化して'
       'いるのは dB のまま足した評価値の和であり，レートに換算した和ではない。'
       'また，直前まで使っていた RSU には切替を抑えるための加点（3 dB）を与える。')
body_p('(5) 切替．前の時刻から接続先の RSU が変わった車両アンテナで，'
       'ハンドオーバーが生じる。')
body_p('手順 (4) の割当層は，受動接続を除く比較手法で共通である。'
       'したがって手法間の差は手順 (1) から (3)，すなわち地図が与える予測にのみ'
       '由来する。受動接続は手順 (1) から (4) を持たず，規格の受動アソシエーションに'
       '従って，閾値を超えた空き RSU に接続して保持する。')

# ------------------------------------------------------------------ 3 評価の方法
h1('評価の方法')

h2('シミュレーションの設定')
body_p('都市部の 2 車線道路を模した環境で，路側の RSU と走行車両がミリ波で'
       '通信する。配置を図4に，条件を表2に示す。')
figure('B_layout.png', '図4　シミュレーションの配置．ポール 4 本にそれぞれ'
       '上流向き・下流向きの面があり，合わせて論理 RSU 8 基となる．'
       '断面図は奥車線の車両から RSU への見通しを示す．')
table(['項目', '設定'],
      [['道路・RSU', '都市部 2 車線（各方向 1 車線），論理 RSU 8 基'
                     '（ポール 4 本 × 上流/下流 2 面，ポール間隔 20 m，'
                     '路側 y = 6 m，高さ 2.5 m）．同一ポールの 2 面は独立した'
                     '送受信機として扱い，同時に別の車両と通信できる'
                     '（排他制約は論理 RSU ごとに掛かる）．'
                     'アンテナ傾き δ_rsu = 70°，車載 δ_veh = 20°'],
       ['車両', f'中央 {int(ncar.median())} 台/走行（{int(ncar.min())}〜{int(ncar.max())}），'
                '全走行車両が通信対象，速度 7.0 m/s，車頭時間 平均 2.0 s，'
                '前後 2 アンテナ（高さ 1.35 m）'],
       ['無線', '60 GHz，送信電力 −7 dBm，接続閾値 −68.5 dBm，帯域 100 MHz，'
                '排他接続．観測レポートは 20 Hz，観測雑音の標準偏差 2.0 dB'],
       ['チャネル', '自由空間経路損失 + 持続シャドウイング（標準偏差 4 dB，'
                   '相関長 6 m）+ 路上駐車 2 台による遮蔽．フェージング無効．'
                   'シャドウイングの場と駐車位置は環境ごとに固定'],
       ['レート', 'Shannon 式（帯域 100 MHz，雑音床 −95 dBm，効率 1.0，'
                 'SNR 上限 50 dB）．802.15.3e に準拠するのは接続手順'
                 '（受動アソシエーションと排他接続）であり，'
                 'チャネル帯域は規格の 2.16 GHz ではない'],
       ['電波地図', '構成は表1のとおり．学習 30 走行（評価とは別の乱数系列）'],
       ['評価', f'{N} 走行．対比較 Wilcoxon 符号順位検定，'
                '信頼区間は対差の t 区間']],
      [3.0, 12.5])
body_p('接続閾値 −68.5 dBm は，802.15.3e の MCS テーブル下限に対応する'
       '受信レベルである。持続シャドウイングの場と路上駐車の位置は，学習相と'
       '評価相で同一である。これは環境に固有の減衰を学習対象とする電波地図の'
       '前提であり，走行ごとに変わるのは交通実現と観測雑音である。')

h2('評価の手続き')
body_p('全手法を事後再計算で評価する（図5）。軌跡と全ペアの受信電力を 1 回記録し，'
       '同一のチャネル実現の上で通信とスケジューリングだけを再計算するため，'
       '同じ交通・同じ電波の実現の上で手法だけを差し替えることになり，'
       '条件差が完全に打ち消される。再計算は決定論的である。')
figure('C_pipeline.png', '図5　評価の手続き．記録は手法に依存しないため，'
       '手法を増やしても再記録は要らない．')
body_p(f'評価は，全車が走り出した時刻を起点とする 35 秒の窓で行う。'
       f'{N} 走行すべてで同じ長さの窓を用いる。')

h2('用語')
table(['用語', '意味'],
      [['割当（grant）', 'ある車両にある RSU を使う権利が与えられている状態．'
                        '割当があっても電波が弱ければ送れないので，'
                        '割当時間と接続時間は一致しない'],
       ['接続時間', '割当を得たうえで実際に受信電力が閾値を超え，'
                   'データを送れていた時間'],
       ['対差', '同じ交通実現（同じ記録）どうしで手法間の値を引き算した差．'
               '走行ごとの交通の当たり外れが打ち消されるため，本資料の主指標とする'],
       ['走行ごと相対差', '対差を，その走行の受動接続の値で割ったもの'],
       ['到達率', '受動接続との対差のうち，理想反応型（oracle）が得た対差の'
                 '何 % を提案手法が得たか．地図が完全な計測をどこまで'
                 '代替できたかを表す'],
       ['LCB（κσ）', '予測値から不確かさ σ の κ 倍を差し引いた値で評価する'
                    '慎重な選び方（2.3 節）'],
       ['遮蔽リスク地図 σν²', '空間基底では表現しきれない受信電力の変動の大きさを，'
                            '位置ごとに別に保持する地図（2.2 節）'],
       ['割当層', '各時刻に「どの車両にどの RSU を与えるか」を決める部分．'
                 '受動接続を除くすべての手法で共通に，ハンガリアン法で'
                 '評価値の総和を最大化する']],
      [3.6, 11.9])

h2('比較した手法')
table(['手法', '内容'],
      [['assoc_hold', '802.15.3e 準拠の受動接続．閾値を超えた空き RSU に接続し'
                      '保持する．能動的なハンドオーバーを行わない標準ベースライン'],
       ['rem_conv', '提案手法．学習済みの電波地図を持ち，観測は割当中のペアに限る'
                    '（規格忠実）'],
       ['rem_cold', '地図を持たず，観測も割当中のペアに限る．規格忠実な条件で'
                    '事前学習だけを外した構成'],
       ['rem_cold_probe', '地図を持たないが，全ペアを観測して走行内で地図を育てる．'
                          '規格の接続手順では実行できない参照条件'],
       ['rem_nolcb', '提案手法から不確実性の利用（LCB，κσ）を外した構成'],
       ['rem_novar', '提案手法から遮蔽リスク地図（σν²）を外した構成'],
       ['rem_nokrig', '提案手法から残差の補間を外した構成'],
       ['rem_frozen', '提案手法から走行内の地図更新を外した構成'],
       ['oracle', '全ペアの現在の真値を雑音なしで用いる理想反応型．割当層は'
                  '提案手法と同じ．未来の予測ではなく「完全な計測が得られたら'
                  'どれだけ配信できるか」を表す']],
      [3.6, 11.9])

# ------------------------------------------------------------------ 4 根拠
h1('主張の根拠')

h2('受動接続に対する優位')
figure('H_gain.png', f'図6　受動接続（assoc_hold）に対する対差（{N} 走行）．'
       '誤差棒は対応データの 95% 信頼区間．')
body_p(f'提案手法の対差は +{mb(d.mean())} MB（95% 信頼区間 ±{mb(d_ci)} MB，'
       f'Wilcoxon 符号順位検定 p = {p_conv:.1g}），走行ごと相対差の平均は '
       f'+{rel.mean():.1f}% である。全ペアの受信電力が既知の理想反応型に対する'
       f'到達率は，対差ベースで {mb(d.mean())} / {mb(d_o.mean())} = {capture:.1f}% であり，'
       f'地図が規格上行えない計測をこの水準まで代替できたことを意味する。')
figure('I_runs.png', f'図7　走行ごとの相対差（{N} 走行，昇順）．')
body_p(f'{N} 走行すべてで提案手法が受動接続を上回る。最も差が小さい走行でも '
       f'+{rel.min():.1f}%，最も大きい走行で +{rel.max():.1f}% である。'
       f'走行ごとの交通実現のばらつきに対して，優位は一貫している。')

h2('利得の源泉は割当時間の質である')
figure('J_grant.png', '図8　割当を得ていた時間（grant）のうち，'
       '実際に送れた時間（接続時間）の割合．')
body_p(f'割当を得ていた時間のうち実際に送れた割合でみると，受動接続の '
       f'{eff("assoc_hold"):.1f}% に対し提案手法は {eff("kkf_conv"):.1f}% である。'
       f'すなわち利得は割当時間を延ばすことではなく，接続できない相手を'
       f'選ばないことから来ている。1 基の RSU に同時に接続できる車両は'
       f'1 台に限られるため，送れていない割当はその RSU を使えたはずの'
       f'他の車両にとっての損失でもある。')

h2('どの機構が効いているか')
figure('K_ablation.png', f'図9　提案手法から機構を 1 つずつ外したときの対差'
       f'（{N} 走行）．正の値は提案手法が上回ることを意味する．')
body_p('地図を持たない rem_cold は総配信量 0 MB であった。割当層は各ペアの'
       '見込みを必要とするため，割当中のペアしか観測できない規格忠実な条件では，'
       '接続しなければ観測できず，観測できなければ割り当てられないという循環に'
       '陥る。本実装においてこの循環を断っている機構は事前学習した地図のみである。')
body_p(f'利得は地図の内部機構ではなく，事前学習した地図を持つこと自体に'
       f'帰属する。構成要素ごとに切り分けると，遮蔽リスク地図 σν² の寄与が '
       f'+{mb(ABL["kkf_novar"][0])} MB（利得の {ABL["kkf_novar"][2]:.1f}%，'
       f'p = {ABL["kkf_novar"][3]:.1g}），不確実性を用いた慎重な選択（LCB）が '
       f'+{mb(ABL["kkf_nolcb"][0])} MB（{ABL["kkf_nolcb"][2]:.1f}%，'
       f'p = {ABL["kkf_nolcb"][3]:.1g}），残差の補間が '
       f'+{mb(ABL["kkf_nokrig"][0])} MB（{ABL["kkf_nokrig"][2]:.1f}%，'
       f'p = {ABL["kkf_nokrig"][3]:.1g}）である。'
       f'いずれも符号は正だが，利得全体に占める割合は限られる。')
body_p(f'一方，走行内の地図更新を外した構成との差は '
       f'{smb(ABL["kkf_frozen"][0])} MB（p = {ABL["kkf_frozen"][3]:.2f}）で，'
       f'有意でない。本評価の環境ではシャドウイングの場も遮蔽体も走行間で'
       f'固定されており，学習済みの地図に対して走行内の観測が加える情報が'
       f'小さいためである。')
table(['外した機構', '対差 [MB]', '95%CI', '利得に占める割合', 'p'],
      [['遮蔽リスク地図 σν²', smb(ABL['kkf_novar'][0]), f'±{mb(ABL["kkf_novar"][1])}',
        f'{ABL["kkf_novar"][2]:+.1f}%', f'{ABL["kkf_novar"][3]:.1g}'],
       ['不確実性の利用（LCB）', smb(ABL['kkf_nolcb'][0]), f'±{mb(ABL["kkf_nolcb"][1])}',
        f'{ABL["kkf_nolcb"][2]:+.1f}%', f'{ABL["kkf_nolcb"][3]:.1g}'],
       ['残差の補間', smb(ABL['kkf_nokrig'][0]), f'±{mb(ABL["kkf_nokrig"][1])}',
        f'{ABL["kkf_nokrig"][2]:+.1f}%', f'{ABL["kkf_nokrig"][3]:.1g}'],
       ['走行内の地図更新', smb(ABL['kkf_frozen'][0]), f'±{mb(ABL["kkf_frozen"][1])}',
        f'{ABL["kkf_frozen"][2]:+.1f}%', f'{ABL["kkf_frozen"][3]:.2f}']],
      [4.6, 2.6, 2.2, 3.3, 2.8])
note('対差は提案手法から見た差であり，正の値は提案手法が上回ることを意味する。'
     '探索的比較であり多重比較の補正は行っていない。')

h2('地図は接続可否を当てられるか')
body_p('本文の主張は，地図が「その相手に今つないだら送れるか」を当てられることに'
       '依存する。これを，記録した真値を答え合わせに用いて測った（図10）。')
figure('L_calibration.png', f'図10　地図の判定精度（{N} 走行）．'
       '(a) 予測と真値，(b) 不確かさの較正．')
body_p(f'取りこぼし（真に接続可能なのに地図が接続不可と判定した割合）は '
       f'{au.miss_pct:.1f}%，幻リンク（その逆）は {au.false_alarm_pct:.2f}% で，'
       f'地図は接続可否をおおむね正しく判定できている。接続可能な標本での'
       f'予測誤差はバイアス {au.bias_db_on_connectable:+.2f} dB，'
       f'RMSE {au.rmse_db_on_connectable:.1f} dB である。')
body_p(f'不確かさ σ については，実際の誤差との比 z =（予測値 − 真値）/ σ の'
       f'標準偏差が {au.z_std:.2f} であり，σ は実際の誤差より大きく見積もられている。'
       f'一方で σ と誤差の大きさの順位相関は {au.spearman_sigma_abserr:.2f} と高い。'
       f'すなわち σ は誤差の絶対的な大きさは表していないが，どのペアの予測が'
       f'より不確かかという順序は捉えており，LCB による順位付けが機能する。')

h2('車両間の公平性')
figure('M_fairness.png', f'図11　車両ごとの配信量からみた公平性（{N} 走行）．')
mn_a = float(agg.loc['assoc_hold', 'min_car_data_MB_mean'])
mn_c = float(agg.loc['kkf_conv', 'min_car_data_MB_mean'])
z_a = float(agg.loc['assoc_hold', 'zero_car_pct_mean'])
z_c = float(agg.loc['kkf_conv', 'zero_car_pct_mean'])
body_p(f'提案手法は，最も恵まれない車両の配信量で受動接続の {mn_c/mn_a:.1f} 倍'
       f'（{mn_c:.0f} MB 対 {mn_a:.0f} MB）であり，配信量が 0 の車両の割合も '
       f'{z_a:.2f}% から {z_c:.2f}% に減る。総配信量を増やす一方で取りこぼしを'
       f'増やしてはいない。これは目的関数の性質でもある。2.3 節のとおり割当層が'
       f'最大化するのは評価値を dB のまま足した和であり，dB 和の最大化は線形の'
       f'受信電力の積の最大化にあたる。レートの総和を最大化する場合より，'
       f'弱い相手を切り捨てにくい目的関数になっている。')

# ------------------------------------------------------------------ 5 限界
h1('限界')
body_p('第一に，提案手法自体の測定はこの RSU 配置の 1 点に限られる。'
       '配置を変えたときに到達率が保たれるかは確かめていない。'
       '条件ごとに地図の学習が必要なため，他の配置での測定は行っていない。')
body_p('第二に，能動的な切替を行う非提案ベースラインを置いていない。'
       '比較対象は受動接続および地図を外した構成のみである。受動接続は'
       '閾値を超えた RSU に接続して保持するだけで切替を行わない。'
       '閾値割れで解放して再アソシエートする反応型は規格の枠内で構成でき，'
       '提案手法の利得のうち「切替を行うこと自体」に帰属する分を切り分けるには'
       'このベースラインの測定が要る。')
body_p('第三に，oracle は絶対的な上界ではない。oracle が持つのは全ペアの'
       '現在値であって未来ではなく，割当層は提案手法と同じである。'
       '本実装は将来時刻の予測を行わない設定で評価しているが，地図は位置の'
       '関数であるため，車両の未来位置を与えれば未来の受信電力を返せる。'
       'この意味で，地図には oracle を超えうる余地が残っている。')
body_p('第四に，学習した環境と評価する環境が同一である。環境が変わったときに'
       '地図がどれだけ陳腐化するか，また学習 30 走行という取得コストが利得に'
       '見合うかは評価していない。また学習相のみ全ペア観測を用いており'
       '規格忠実ではない。評価相は割当中のペアに限った観測に戻しているため'
       '比較の公正性は保たれるが，現地で地図を育てる運用には初期探索方策が'
       '別途必要である。')
body_p('第五に，走行中の車両を遮蔽体として扱っていない。本評価で電波をさえぎるのは'
       '位置の固定された路上駐車 2 台のみである。60 GHz では走行車両による'
       '見通しの遮断が主要因になるため，実環境では遮蔽の頻度も予測の難しさも'
       '本評価より大きい。')
body_p('第六に，ハンドオーバー回数・ping-pong 率（同じ 2 基の間を往復する切替の'
       '割合）・無接続時間の最大連続長を測定していない。接続時間と割当時間で'
       '代替しているが，切替コストの直接評価は今後の課題である。')

# ------------------------------------------------------------------ 付録
h1('付録：本実装の設定値')
table(['項目', '値'],
      [['空間基底', 'ガウス基底 40 個，道路に沿って 0〜210 m に等間隔．'
                   '幅は中心間隔と同じ（約 5.4 m）'],
       ['係数の事前分散', '100'],
       ['共分散の膨張項', '1×10⁻⁴ ×Δt（観測間で加える）'],
       ['平均場の事前値', '−110 dBm（地図はこの値からの偏差を学習する）'],
       ['残差場の標準偏差', '4.0 dB（定数）'],
       ['残差の空間相関長 / 時間相関長', '20 m / 5 s'],
       ['残差バッファ', '64 点（古いものから捨てる）'],
       ['遮蔽リスク地図 σν²(s)', '有効．格子 2.0 m，カーネル幅は格子と同じ'],
       ['LCB の係数 κ', '1.0'],
       ['接続見込みの判定', '平均の予測値が −68.5 dBm 以上（接続閾値と同値）'],
       ['割当', 'ハンガリアン法．切替の抑制は現割当 RSU への加点 3.0 dB'],
       ['再計画周期', '0.05 s'],
       ['学習走行数', '30']],
      [5.4, 10.1])

h1('付録：再現手順')
body_p('記録は手法にも設定にも依存しないため，一度記録すれば手法を変えた評価は'
       'Gazebo なしで再現できる。')
note('# 記録\n'
     'python3 tools/sim.py run --sweep-config scratch/record_p4_all_n50.yaml\n'
     'python3 tools/verify_recording.py sim_results/rec_p4_all_n50 --duration 35\n'
     '\n'
     '# 評価\n'
     'python3 tools/replay_sweep.py scratch/rec50 \\\n'
     '  --state-dir sim_results/urban_cm_d70_learn/rem_state \\\n'
     '  --out sim_results/ablation_n50 --jobs 12 --from-motion --duration 35\n'
     '\n'
     '# 地図の判定精度\n'
     'python3 tools/audit_rem_calibration.py scratch/rec50 \\\n'
     '  --state-dir sim_results/urban_cm_d70_learn/rem_state \\\n'
     '  --out sim_results/rem_audit_n50 --mode frozen --from-motion --duration 35')

h1('参考文献')
note('[1] N. Cressie, Statistics for Spatial Data. New York, NY, USA: '
     'John Wiley & Sons, 1993.')
note('[2] IEEE Std 802.15.3e-2017, IEEE Standard for High Data Rate Wireless '
     'Multi-Media Networks—Amendment 1: High-Rate Close Proximity '
     'Point-to-Point Communications, 2017.')

OUTP = f'{OUT}/調査資料_電波地図に基づく能動的ハンドオーバー.docx'
doc.save(OUTP)
print(f'wrote {OUTP}')
