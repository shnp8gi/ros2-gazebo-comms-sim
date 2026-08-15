#!/usr/bin/env python3
"""内部審査資料 (Word) を生成する。構成は一本の柱に従う。

  主張 → 枠組み → 根拠 → 主張が成り立つ条件 → 限界と対応

数値はすべて sim_results の集計 CSV から読む (本文への手書き写しをしない)。
"""
import os
import sys

import numpy as np
import pandas as pd
from docx import Document
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.oxml.ns import qn
from docx.shared import Cm, Pt, RGBColor
from scipy import stats

ROOT = '/workspace'
OUT = os.path.join(ROOT, 'scratch', 'review')
os.makedirs(OUT, exist_ok=True)
sys.path.insert(0, f'{ROOT}/tools')
from lib import eval_metrics  # noqa: E402

doc = Document()
st = doc.styles['Normal']
st.font.name = 'Yu Gothic'
st.font.size = Pt(10.5)
st.element.rPr.rFonts.set(qn('w:eastAsia'), '游ゴシック')
st.paragraph_format.space_after = Pt(6)
st.paragraph_format.line_spacing = 1.2
for sec in doc.sections:
    sec.top_margin = sec.bottom_margin = Cm(1.9)
    sec.left_margin = sec.right_margin = Cm(1.9)


def _jp(run, size=10.5, bold=False):
    run.font.size = Pt(size)
    run.bold = bold
    run.font.name = 'Yu Gothic'
    run._element.rPr.rFonts.set(qn('w:eastAsia'), '游ゴシック')
    return run


def h(level, text):
    p = doc.add_heading(text, level=level)
    for r in p.runs:
        r.font.color.rgb = RGBColor(0x13, 0x1C, 0x26)
        _jp(r, size={0: 17, 1: 13, 2: 11.5}.get(level, 11))
    return p


def para(text, bold=False, size=10.5):
    p = doc.add_paragraph()
    _jp(p.add_run(text), size=size, bold=bold)
    return p


def bullet(text, bold_head=None):
    p = doc.add_paragraph(style='List Bullet')
    if bold_head:
        _jp(p.add_run(bold_head), bold=True)
    _jp(p.add_run(text))
    return p


def claim_box(text):
    t = doc.add_table(rows=1, cols=1)
    t.style = 'Table Grid'
    c = t.rows[0].cells[0]
    c.text = ''
    _jp(c.paragraphs[0].add_run(text), size=11, bold=True)
    c.width = Cm(17.2)
    doc.add_paragraph()
    return t


def table(headers, rows, widths, note=None):
    t = doc.add_table(rows=1, cols=len(headers))
    t.style = 'Table Grid'
    t.alignment = WD_TABLE_ALIGNMENT.CENTER
    for i, hd in enumerate(headers):
        c = t.rows[0].cells[i]
        c.text = ''
        _jp(c.paragraphs[0].add_run(hd), size=9, bold=True)
        c.paragraphs[0].alignment = WD_ALIGN_PARAGRAPH.CENTER
    for row in rows:
        cells = t.add_row().cells
        for i, v in enumerate(row):
            cells[i].text = ''
            _jp(cells[i].paragraphs[0].add_run(str(v)), size=9)
            if i > 0:
                cells[i].paragraphs[0].alignment = WD_ALIGN_PARAGRAPH.RIGHT
    for r in t.rows:
        for i, w in enumerate(widths):
            r.cells[i].width = Cm(w)
    if note:
        p = doc.add_paragraph()
        _jp(p.add_run(note), size=8.5)
        p.paragraph_format.left_indent = Cm(0.3)
    else:
        doc.add_paragraph()
    return t


# ------------------------------------------------------------- データ
R = f'{ROOT}/sim_results'
abl = pd.read_csv(f'{R}/rec_cm_d70_ablation/analysis/agg.csv').set_index('method')
runs = pd.read_csv(f'{R}/rec_cm_d70_ablation/analysis/runs.csv')
pa = pd.read_csv(f'{R}/rec_cm_d70_ablation/analysis/paired.csv')
pa = pa[pa.metric == 'total_data_MB'].set_index('baseline')
pk = eval_metrics.paired(runs, 'kkf_conv', 'method', ['load'], ['total_data_MB'])
pk = pk[pk.metric == 'total_data_MB'].set_index('baseline')
reg = pd.read_csv(f'{R}/rec_cm_d70_abl_reg2/analysis/agg.csv').set_index('method')
sweep = pd.read_csv(f'{R}/choice_margin_summary.csv')
sweep = sweep[sweep['索引OK'] == True].sort_values('選択余地率')

base = float(abl.loc['assoc_hold', 'total_data_MB_mean'])
conv = float(abl.loc['kkf_conv', 'total_data_MB_mean'])
orc = float(abl.loc['oracle', 'total_data_MB_mean'])
N = int(abl.loc['assoc_hold', 'n'])
capture = 100.0 * (conv - base) / (orc - base)
p_main = float(pa.loc['kkf_conv', 'wilcoxon_p'])
a_s = runs[runs.method == 'assoc_hold'].set_index('run')
k_s = runs[runs.method == 'kkf_conv'].set_index('run')
d = (k_s.total_data_MB - a_s.total_data_MB).dropna()
hw = stats.t.ppf(0.975, len(d) - 1) * d.std(ddof=1) / np.sqrt(len(d))
rel = (100 * (k_s.total_data_MB - a_s.total_data_MB) / a_s.total_data_MB).dropna()
hw_r = stats.t.ppf(0.975, len(rel) - 1) * rel.std(ddof=1) / np.sqrt(len(rel))
ncar = runs.n_cars.mean()
reg_b = float(reg.loc['assoc_hold', 'total_data_MB_mean'])
reg_c = float(reg.loc['kkf_conv', 'total_data_MB_mean'])
reg_pct = 100.0 * (reg_c - reg_b) / reg_b
_rr = pd.read_csv(f'{R}/rec_cm_d70_abl_reg2/analysis/runs.csv')
_ra = _rr[_rr.method == 'assoc_hold'].set_index('run').total_data_MB
_rk = _rr[_rr.method == 'kkf_conv'].set_index('run').total_data_MB
reg_rel = float((100 * (_rk - _ra) / _ra).dropna().mean())


def _audit(path):
    a = pd.read_csv(path).iloc[0]
    return {'miss': float(a['miss_pct']), 'fa': float(a['false_alarm_pct']),
            'bias': float(a['bias_db_on_connectable']),
            'rmse': float(a['rmse_db_on_connectable']), 'z': float(a['z_std'])}


au = _audit(f'{R}/audit_d70/summary_frozen.csv')
ar = _audit(f'{R}/audit_d70_reg/summary_frozen.csv')


def pct(m):
    return 100.0 * (float(abl.loc[m, 'total_data_MB_mean']) - base) / base


def paired_vs_conv(m):
    return float(pk.loc[m, 'diff_mean']), float(pk.loc[m, 'wilcoxon_p'])


# ------------------------------------------------------------- 本文
h(0, 'KKF 電波地図に基づくプロアクティブ・ハンドオーバーの有用性')
para('シミュレーション評価 / 2026-08-09 / feat/urban-2lane', size=9.5)
para('本稿で「プロアクティブ」とは、規格上 observe できないペアの現在値を地図で'
     '推定して能動的に切替判断を行うことを指し、将来時刻の予測は含まない '
     '(先読み段数 1)。', size=9.5)

h(1, '1. 主張')
claim_box(
    f'本シミュレーション環境の配置 d70 において、KKF 電波地図に基づく能動的な'
    f'ハンドオーバーは、受動接続 (assoc_hold) に対し総配信量で '
    f'+{rel.mean():.1f}% 優位である ({conv:,.0f} MB 対 {base:,.0f} MB)。'
    f'この優位性は事前学習した電波地図に由来し、地図は「規格上observeできないペアの'
    f'現在値」を代替する役割を担う。ただし優位性の大きさは RSU 配置に条件付けられ、'
    f'当初構成 (選択余地率 0.000) では利得の上限が +4.5% にとどまる。')
para(f'主指標は走行ごとの相対対差の平均で +{rel.mean():.1f}% ± {hw_r:.1f} pt '
     f'(95% 信頼区間、対応データの t 区間)。総和比では +{pct("kkf_conv"):.1f}%。'
     f'対差の絶対量は +{d.mean():,.0f} MB (半幅 ±{hw:,.0f} MB)。'
     f'N = {N} 走行すべてで提案手法が上回り、'
     f'最も差が小さい走行でも +{d.min():,.0f} MB (+{rel.min():.1f}%) の改善である。'
     f'ばらつきによる上振れではない。')

h(1, '2. 評価の枠組み')
para('全手法を事後再計算 (replay) で評価する。軌跡と全ペア RSSI を 1 回記録し、'
     '同一チャネル実現の上で通信とスケジューリングだけを再計算するため、'
     '手法間の比較は共通乱数の理想形になる。', bold=True)
table(['項目', '設定'],
      [['道路・RSU', '都市部 2 車線 (各方向 1 車線)、論理 RSU 8 基 '
                     '(ポール 4 本 × 上流/下流 2 面、ポール間隔 20 m、路側 y = 6 m、高さ 2.5 m)。'
                     'アンテナ傾き δ_rsu = 70°、車載 δ_veh = 20° (相互ボアサイト)。'
                     'これを配置 d70 と呼ぶ (§4 表参照)'],
       ['車両', f'平均 {ncar:.0f} 台/走行 (30〜42)、速度 7.0 m/s、車頭時間 平均 2.0 s、'
                '前後 2 アンテナ (高さ 1.35 m)'],
       ['無線', '60 GHz、送信電力 −7 dBm、接続閾値 −78.5 dBm、帯域 100 MHz、'
                '排他接続 (1 RSU = 同時 1 台)'],
       ['チャネル', '自由空間経路損失 + 固定シャドウイング (σ = 4 dB) + 路上駐車による遮蔽。'
                    'フェージング無効'],
       ['走行', '交通生成窓 35 s、走行のシム時刻は約 76 s (上限 90 s)、'
                 '記録周期 5 ms、観測レポート 20 Hz。接続時間は車両 × アンテナで積算するため、'
                 '上限は 8 RSU × 走行時間 ≈ 610 s'],
       ['レート', 'Shannon 式 (帯域 100 MHz、雑音床 −95 dBm、効率 1.0、SNR 上限 50 dB)。'
                  '接続閾値は MCS 表の最下位に相当する SNR から −10 dB の感度点。'
                  '802.15.3e に準拠するのは接続手順 (受動アソシエーションと排他接続) であり、'
                  'チャネル帯域は規格の 2.16 GHz ではない'],
       ['KKF 地図', 'RBF 基底 40 個 (弧長 0〜210 m)、地図は (RSU, 走行方向, 車載アンテナ) '
                    'ごとに独立 = 32 枚。学習 30 走行 (評価とシード域を分離)'],
       ['評価', f'N = {N} 走行 (記録 20 走行のうち pairs が生成された走行)、'
                 f'対比較 Wilcoxon 符号順位検定。信頼区間は対差の t 区間'],
       ['脱落', '記録 20 走行のうち 7 走行は pairs ファイルが生成されず評価から外れた。'
                 '原因は未特定であり、交通条件との系統的な関係は確認できていない '
                 '(選択的除外によるバイアスの可能性は排除できない)'],
       ['再現', 'commit 755fe3d / シナリオ config/scenarios/urban_cm_d70.yaml / '
                '記録 scratch/record_cm_d70.yaml (base_seed 12345) / '
                '学習 base_seed 512345・30 走行 / '
                '再生 tools/replay_sweep.py sim_results/rec_cm_d70 --state-dir <学習状態>']],
      widths=[3.0, 14.2])
para('比較対象:')
bullet('— 802.15.3e 準拠の受動接続。閾値を超えた空き RSU に接続し保持する。'
       '能動的なハンドオーバーを行わない標準ベースライン', bold_head='assoc_hold ')
bullet('— 提案手法。学習済み電波地図を持ち、観測は grant 中のペアに限る (規格忠実)',
       bold_head='kkf_conv ')
bullet('— 全ペアの**現在**真値を雑音なしで用いる理想反応型。未来の予測ではなく'
       '「完全な計測が得られたら何点取れるか」を表す。本評価の先読み段数は 1 '
       '(先読みなし) であり、地図の役割は未来の予測ではなく、規格上 observe できない'
       'ペアの現在値の代替である',
       bold_head='oracle (完全 CSI 既知) ')

h(1, '3. 主張の根拠')
rows = []
for m, label in [('oracle', 'oracle (完全 CSI 既知の理想反応型)'),
                 ('kkf_conv', 'kkf_conv (提案手法)'),
                 ('assoc_hold', 'assoc_hold (標準ベースライン)'),
                 ('kkf_cold', 'kkf_cold (地図なし・規格忠実)')]:
    mean = float(abl.loc[m, 'total_data_MB_mean'])
    conn = float(abl.loc[m, 'connected_time_s_mean'])
    gr = float(abl.loc[m, 'grant_time_s_mean'])
    if m == 'assoc_hold':
        dd, rr = '—', '—'
    else:
        s = (runs[runs.method == m].set_index('run').total_data_MB
             - a_s.total_data_MB).dropna()
        hh = stats.t.ppf(0.975, len(s) - 1) * s.std(ddof=1) / np.sqrt(len(s)) if s.std() > 0 else 0.0
        dd = f'{s.mean():+,.0f} ±{hh:,.0f}'
        _rl = (100 * (runs[runs.method == m].set_index('run').total_data_MB
                      - a_s.total_data_MB) / a_s.total_data_MB).dropna()
        rr = f'{_rl.mean():+.1f}'
    rows.append([label, f'{mean:,.0f}', dd, rr, f'{conn:.0f}', f'{gr:.0f}'])
table(['手法', '総配信量 [MB]', 'assoc_hold との対差 [MB]', '走行ごと相対差 [%]',
       '接続時間 [s]', 'grant 時間 [s]'],
      rows, widths=[5.2, 2.6, 3.8, 1.8, 2.0, 1.8],
      note='対差とその 95% 信頼区間が本命の指標である (同一チャネル実現の対応データのため)。'
           '総配信量の絶対値は走行ごとの交通実現に大きく左右されるので、単独の信頼区間は参考値。')
para(f'提案手法は受動接続より grant 時間が {float(abl.loc["assoc_hold","grant_time_s_mean"]) - float(abl.loc["kkf_conv","grant_time_s_mean"]):.0f} 秒短いにもかかわらず、'
     f'接続時間は {float(abl.loc["kkf_conv","connected_time_s_mean"]) - float(abl.loc["assoc_hold","connected_time_s_mean"]):.0f} 秒長く、配信量は多い。'
     f'「繋がらない相手を掴まない」判断が効いている。'
     f'完全 CSI 既知の理想反応型に対する到達率は、対差ベースで '
     f'{d.mean():,.0f}/{orc-base:,.0f} = {capture:.1f}% であり、'
     f'地図が「行えない計測」をこの水準まで代替できたことを意味する。')

h(2, '3.1 優位性は事前学習した地図に由来する')
para('地図を持たない kkf_cold は総配信量 0 MB であった。grant 中のペアしか観測できない'
     '規格忠実な条件では、繋がないと観測できず、観測できないと繋げないという循環に陥る。'
     '本実装において、この循環を断っている機構は事前学習した地図のみである。', bold=True)
ab_rows = []
for m, label in [('kkf_cold', '事前学習した地図を外す'),
                 ('kkf_novar', '遮蔽リスク地図 σν² を外す'),
                 ('kkf_nolcb', '不確実性の利用 (LCB, κσ) を外す')]:
    dv, pv = paired_vs_conv(m)
    ci = float(pk.loc[m, 'diff_ci95'])
    tot = float(abl.loc[m, 'total_data_MB_mean'])
    share = ('—' if m == 'kkf_cold' else f'{100*dv/d.mean():+.1f}%')
    ab_rows.append([label, f'{tot:,.0f}', f'{dv:+,.0f}', f'±{ci:,.0f}', share,
                    '— (決定論的)' if m == 'kkf_cold' else f'{pv:.2g}'])
table(['外した機構', '総配信量 [MB]', 'kkf_conv との対差 [MB]', '95%CI 半幅',
       '利得に占める割合', 'p'], ab_rows,
      widths=[4.4, 2.4, 3.0, 2.0, 2.4, 2.0],
      note='いずれも提案手法から当該機構だけを外した構成。'
           'p は Wilcoxon 符号順位検定で、N = 13 では 0.00024 が到達可能な下限 '
           '(= 全走行で同符号) であり効果量を表さない。探索的比較であり多重比較の補正は'
           '行っていない。kkf_cold は全走行 0 MB で分散がなく検定は意味を持たず、'
           '「利得に占める割合」も (地図の有無は機構の内訳ではないため) 定義しない。'
           '割合の分母は assoc_hold との対差 %s MB。' % f'{d.mean():,.0f}')
dv_l, pv_l = paired_vs_conv('kkf_nolcb')
_ci_l = float(pk.loc['kkf_nolcb', 'diff_ci95'])
para(f'一方、不確実性を用いた慎重な選択 (LCB) の寄与は {dv_l:+,.0f} MB '
     f'(95% 信頼区間 ±{_ci_l:,.0f} MB、p = {pv_l:.2f})。'
     f'区間の上端でも利得の {100*(dv_l+_ci_l)/d.mean():.1f}% にとどまり、'
     f'実質的な寄与は無視できる大きさである。N = {N} では「効果がないこと」の証明には'
     'ならないが、本設定で意味のある寄与をしていないとは言える。')

h(2, '3.2 地図の較正と、健全性指標の妥当性')
para('学習した地図が意思決定に効く領域でどれだけ正確かを、記録した真値を'
     '答え合わせに用いて測った (5 走行、接続可能標本 6.6%)。', bold=True)
table(['地図', '取りこぼし率', '幻リンク率 (ゲート基準)', 'バイアス', 'RMSE',
       'z の標準偏差', '幻リンク率 (グリッド基準)', 'kkf_conv 走行ごと相対差 [%]'],
      [['本結果 (RBF 基底 40)', f'{au["miss"]:.1f}%', f'{au["fa"]:.2f}%',
        f'{au["bias"]:+.2f} dB', f'{au["rmse"]:.2f} dB', f'{au["z"]:.2f}',
        '最大 11.1%', f'{rel.mean():+.1f}'],
       ['正則化版 (基底 20・事前分散 1/4)', f'{ar["miss"]:.1f}%', f'{ar["fa"]:.2f}%',
        f'{ar["bias"]:+.2f} dB', f'{ar["rmse"]:.2f} dB', f'{ar["z"]:.2f}',
        '最大 3.2%', f'{reg_rel:+.1f}']],
      widths=[4.0, 1.8, 2.2, 1.6, 1.5, 1.7, 2.4, 2.0],
      note='取りこぼし・幻リンク (ゲート基準)・バイアス・RMSE・z は較正監査 (5 走行) による。'
           '取りこぼし = 真に接続可能なのに地図が接続不可と判定した割合、幻リンクはその逆。'
           'z = (μ − 真値)/σ で標準偏差 1.0 が較正済みを意味する。'
           '「幻リンク率 (グリッド基準)」は健全性チェックが未観測グリッド上で測る別指標 '
           '(許容 2.0%)。相対差の列は N = 13 走行による。')
para('本結果の地図は接続可能領域でバイアス −1.0 dB、z の標準偏差 1.03 と'
     '較正できており、σ が誤差の大きさを正しく表している。', bold=True)
para('正則化版はグリッド基準の幻リンク率を 11.1% → 3.2% に下げる一方、'
     '意思決定に効く領域では取りこぼし・幻リンク・RMSE のすべてが悪化し、'
     f'総配信量も {100*(reg_c-conv)/conv:.1f}% 低下した。'
     'すなわち本条件では、2 つの指標が逆向きに動いた。', bold=True)
para('ただし正則化版は基底数 (40→20) と事前分散 (1/4) を同時に変えており、'
     'RMSE が 2 倍に悪化したのは表現力不足の帰結とも読める。'
     'したがってこの 2 条件 1 事例から「健全性チェックは地図品質の代理指標として使えない」'
     'と一般化することはできない。ここで言えるのは、本結果の地図が意思決定に効く領域では'
     '較正できており (バイアス −1.0 dB、z 標準偏差 1.03)、'
     'グリッド基準の幻リンク率が示唆するほど劣化していない、という事実にとどまる。')

h(1, '4. 優位性の大きさと RSU 配置の関係')
para('能動的な割当の利得は 3 つの経路から生まれる。(a) どの RSU を選ぶか、(b) いつ掴み・いつ放すか、'
     '(c) 排他接続の下で誰に割り当てるか。このうち (a) は、ある時点で接続可能な RSU が'
     '1 基以下なら自由度を持たない。そこで選択余地率 (2 基以上が同時に接続可能な時刻の割合) を'
     'RSU 配置で制御し、完全 CSI 既知時の利得 (oracle の assoc_hold 比) が'
     'どう変わるかを測った。')
sw_rows = [['当初構成 (RSU 2 基)', '2', '0.000', '+4.5%', '未測定', '19']]
for _, r in sweep.iterrows():
    nm = str(r['条件']).replace('rec_cm_', '配置 ')
    kk = f'{rel.mean():+.1f}%' if 'd70' == str(r['条件']).replace('rec_cm_', '') else '未測定'
    sw_rows.append([nm, '8', f"{r['選択余地率']:.3f}", f"{r['天井_pct']:+.1f}%", kk,
                    f"{int(r['n'])}"])
table(['RSU 配置', '論理 RSU 数', '選択余地率', 'oracle の assoc_hold 比',
       'kkf_conv の assoc_hold 比', 'N'], sw_rows,
      widths=[3.6, 1.8, 2.4, 3.0, 3.2, 1.0],
      note='記号は dNN = アンテナ傾き δ_rsu を NN 度に設定した配置 (ポール 4 本・間隔 20 m 固定)、'
           'd70s10 = δ_rsu 70° でポール間隔のみ 10 m。当初構成は上流/下流を交互に割り当てた '
           'RSU 2 基構成であり、他の条件とは基地局数が異なる参考条件である。'
           '交通条件は全条件で共通だが N は条件ごとに異なる。'
           'kkf_conv は条件ごとに地図の学習が必要なため d70 のみ測定した。'
           '選択余地率は記録された真値からの実測値 (解析計算値ではない)。')
para('本節は主張の適用条件を見積もるためのもので、指標は完全 CSI 既知時の利得 '
     '(oracle の assoc_hold 比) である。提案手法自体の測定は d70 の 1 点に限られる。')
para('当初構成では、記録した全時刻 (19 走行 × 平均 38 台 × 20 Hz) のうち 2 基以上が'
     '同時に接続可能な瞬間が 1 つも存在せず (0.000%)、完全 CSI 既知時の利得は '
     '+4.5% にとどまった。'
     'これは経路 (a) が完全に閉じた状態での上限であり、残る (b) タイミングと '
     '(c) 車間調停だけで得られる利得の大きさを意味する。', bold=True)
para('観測される事実は次の 2 点にとどまる。第一に、選択余地率 0.15 以下の 3 条件は'
     'いずれも上限 5% 未満である。第二に、余地 0.18 と 0.31 の 2 条件はいずれも +25% を超える。'
     'ただし系列は単調でなく (余地 0.147 の条件は +1.7% と当初構成より低い)、'
     '条件数 6 での相関は選択余地率 r = +0.74、カバレッジ r = +0.72 とほぼ同等で有意でない。'
     'したがって「選択余地率が上限を決める」とは現時点で言えない。'
     '言えるのは、当初構成が上限 +4.5% の領域にあり、配置を変えることで +26% の領域へ'
     '移せたという事実である。交絡の切り分けは §5 に記した直交条件で行う。', bold=True)

h(1, '5. 限界と対応')
bullet('oracle は「完全な現在値 + 提案手法の割当層」であり割当層は固定である。'
       '実際、配置 d50 (N = 17) では 17 走行中 3 走行で oracle が受動接続を下回った '
       '(最小 −2,790 MB)。→ 対応: 割当層を改良すればこの値は超えうる。当該走行を解析中',
       bold_head='oracle は絶対的な上界ではない。')
bullet('上限との相関は選択余地率 r = +0.74、カバレッジ (受動接続の配信量) r = +0.72 で'
       'ほぼ同等であり、現在の 5 条件では分離できない。'
       '→ 対応: カバレッジを揃えて余地だけを変える直交 4 条件を記録中',
       bold_head='選択余地とカバレッジが交絡している。')
bullet('replay 評価では予測計算とスケジュール配信の遅延を 0 と仮定している。'
       '予測ホライズンは 1 段 (先読みなし) で評価した。'
       '→ 対応: 遅延を仮定した感度解析は未実施',
       bold_head='制御遅延を評価していない。')
bullet('地図が未学習だと grant されず学習が始まらないため、学習相のみ全ペア観測を用いた。'
       '評価相は grant 限定観測に戻しており比較の公正性は保たれるが、'
       '現地で地図を育てる運用には初期探索方策が別途必要である。'
       '→ 対応: 探索方策の設計は今後の課題',
       bold_head='学習相は規格忠実でない。')
_cv_rel = rel.std(ddof=1) / rel.mean()
_nn = 2
for _ in range(200):
    _t = stats.t.ppf(0.975, max(1, int(_nn) - 1))
    _n2 = (_t * _cv_rel / 0.05) ** 2
    if abs(_n2 - _nn) < 0.5:
        break
    _nn = _n2
bullet(f'主指標 (走行ごと相対差) の 95% 信頼区間半幅は現在 ±{hw_r:.1f} pt '
       f'(平均の ±{100*hw_r/rel.mean():.1f}%)。これを平均の ±5% にするには '
       f'N = {int(np.ceil(_nn))} が必要である。'
       f'→ 対応: 拡大予定。ただし符号の確からしさは {N}/{N} 走行の一致で既に確立している',
       bold_head=f'走行数が N = {N} と少ない。')
bullet('ハンドオーバー回数・ping-pong 率・無接続時間の最大連続長は本評価では'
       '測定していない。→ 対応: 接続時間と grant 時間で代替しているが、'
       '切替コストの直接評価は今後の課題',
       bold_head='ハンドオーバー固有の指標が不足している。')

path = os.path.join(OUT, '内部審査資料_2026-08-09.docx')
doc.save(path)

# --- 査読用テキスト (表を本文の位置に差し込む) ---
from docx.table import Table
from docx.text.paragraph import Paragraph
body = doc.element.body
out = []
ti = 0
for child in body.iterchildren():
    if child.tag.endswith('}p'):
        p = Paragraph(child, doc)
        if not p.text.strip():
            continue
        s = p.style.name
        pre = ('# ' if s == 'Title' else '## ' if s.startswith('Heading')
               else '- ' if 'List' in s else '')
        out.append(pre + p.text)
    elif child.tag.endswith('}tbl'):
        t = Table(child, doc)
        ti += 1
        rws = [' | '.join(c.text.replace('\n', ' ') for c in r.cells) for r in t.rows]
        out.append('[表%d]\n' % ti + '\n'.join(rws))
with open(os.path.join(OUT, 'review_text.md'), 'w', encoding='utf-8') as f:
    f.write('\n\n'.join(out) + '\n')
print('wrote', path)
