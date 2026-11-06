"""最新版Wordの書式を使い、十字固定の訂正を独立した別紙にする。"""
from copy import deepcopy
from datetime import datetime
import hashlib
import json
from pathlib import Path
import posixpath
from zipfile import ZipFile, ZIP_DEFLATED

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Arc
import numpy as np
from lxml import etree as E

from raw_antenna_campaign_common import read_csv
from write_raw_antenna_report import Report, NS, q, node, xml, mr, sub, frac, sup, render

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'sim_results/model12_speed_campaign_20261029/raw_csv_speed_position/cross_fixed_forward'
SOURCE = Path('/home/rosgazebo/Downloads/初回リンク確立までのTimeout回数評価.docx')
TITLE = '十字固定シナリオに対する訂正と追加検討'
DEST = OUT / 'report' / (TITLE + '.docx')


def fonts(root):
    for f in root.findall('.//w:rFonts', NS):
        if any(E.QName(a).namespace == NS['m'] for a in f.iterancestors()):
            continue
        for key in ['asciiTheme', 'hAnsiTheme', 'eastAsiaTheme']:
            f.attrib.pop(q('w:' + key), None)
        f.set(q('w:ascii'), 'Times New Roman')
        f.set(q('w:hAnsi'), 'Times New Roman')
        f.set(q('w:eastAsia'), 'ＭＳ 明朝')


def sqrt(parts):
    rad = node('m:rad'); pr = node('m:radPr')
    hidden = E.Element(q('m:degHide')); hidden.set(q('m:val'), '1'); pr.append(hidden)
    rad.extend([pr, node('m:deg')]); value = node('m:e')
    value.extend([mr(x) if isinstance(x, str) else x for x in parts]); rad.append(value)
    return rad


class Supplement(Report):
    def __init__(self):
        with ZipFile(SOURCE) as z:
            self.files = {n: z.read(n) for n in z.namelist()}
        self.doc = E.fromstring(self.files['word/document.xml'])
        self.body = self.doc.find('w:body', NS)
        # 最新版は元テンプレートと要素配置が異なるため、該当する書式を明示的に取得。
        self.prototypes = {0: deepcopy(self.body[0]), 1: deepcopy(self.body[1]),
                           8: deepcopy(self.body[5]), 45: deepcopy(self.body[9]),
                           46: deepcopy(self.body[10])}
        self.eq_proto = deepcopy(self.body[20])
        section = deepcopy(self.body.find('w:sectPr', NS))
        self.body.clear(); self.body.append(section)
        self.rels = E.fromstring(self.files['word/_rels/document.xml.rels'])
        for rel in list(self.rels):
            if rel.get('Type').rsplit('/', 1)[-1] in ['image', 'oleObject', 'comments', 'commentsExtended', 'commentsIds']:
                self.rels.remove(rel)
        self.image_id = 2000; self.images = []; self.next_page = False
        for p in self.prototypes.values():
            fonts(p)
        header = E.fromstring(self.files['word/header1.xml'])
        for p in header.findall('.//w:p', NS):
            text = ''.join(p.xpath('.//w:t/text()', namespaces=NS))
            if text == '初回リンク確立までのTimeout回数評価':
                self.replace_text(p, TITLE)
            elif text.startswith('2026/'):
                self.replace_text(p, datetime.now().strftime('%Y/%m/%d'))
        fonts(header)
        self.files['word/header1.xml'] = xml(header)

    def paragraph(self, text, kind='body'):
        follows_table = len(self.body) > 1 and self.body[-2].tag == q('w:tbl')
        p = super().paragraph(text, kind)
        pr = p.find('w:pPr', NS)
        if follows_table:
            spacing = pr.find('w:spacing', NS)
            if spacing is None:
                spacing = node('w:spacing'); pr.append(spacing)
            spacing.set(q('w:before'), '100')
        for tag in ['numPr', 'pageBreakBefore']:
            # 改ページはReportが今回付けたものだけ保持。
            if tag == 'pageBreakBefore': continue
            for e in pr.findall('w:' + tag, NS): pr.remove(e)
        if kind == 'caption':
            for e in pr.findall('w:ind', NS): pr.remove(e)
            if text.startswith('図補'):
                for e in pr.findall('w:keepNext', NS): pr.remove(e)
        return p

    def table(self, rows, widths=None):
        t = super().table(rows, widths)
        for p in t.findall('.//w:p', NS):
            for e in p.findall('w:pPr/w:keepNext', NS): e.getparent().remove(e)
        fonts(t)
        return t

    def equation(self, parts, number):
        p = deepcopy(self.eq_proto); pr = deepcopy(p.find('w:pPr', NS)); p.clear(); p.append(pr)
        for e in pr.findall('w:spacing', NS): pr.remove(e)
        pr.append(node('w:spacing', before=120, after=120, line=400, lineRule='atLeast'))
        pr.append(node('w:keepLines'))
        for name in ['left', 'right']:
            r = node('w:r'); r.append(node('w:tab'))
            if name == 'left':
                p.append(r); math = node('m:oMath')
                math.extend([mr(x) if isinstance(x, str) else x for x in parts]); p.append(math)
            else:
                p.append(r); r = node('w:r'); t = node('w:t'); t.text = '(補' + str(number) + ')'
                r.append(t); p.append(r)
        self.append(p)

    def save(self, path):
        fonts(self.body)
        self.files['word/document.xml'] = xml(self.doc)
        self.files['word/_rels/document.xml.rels'] = xml(self.rels)
        # 本文から除いた図の画像を持ち込まず、ヘッダー画像等の参照先は保持する。
        referenced = set()
        for name, data in self.files.items():
            if not name.endswith('.rels'): continue
            rels = E.fromstring(data)
            base = posixpath.dirname(posixpath.dirname(name))
            for r in rels:
                if r.get('TargetMode') != 'External':
                    referenced.add(posixpath.normpath(posixpath.join(base, r.get('Target'))))
        for name in list(self.files):
            if name.startswith('word/media/') and name not in referenced:
                del self.files[name]
        ct = E.fromstring(self.files['[Content_Types].xml'])
        for e in list(ct):
            if e.tag.endswith('Override') and e.get('PartName').lstrip('/') not in self.files:
                ct.remove(e)
        self.files['[Content_Types].xml'] = xml(ct)
        core = E.fromstring(self.files['docProps/core.xml'])
        for e in core:
            if E.QName(e).localname == 'title': e.text = TITLE
            if E.QName(e).localname == 'modified': e.text = datetime.now().astimezone().isoformat(timespec='seconds')
        self.files['docProps/core.xml'] = xml(core)
        path.parent.mkdir(parents=True, exist_ok=True)
        with ZipFile(path, 'w', ZIP_DEFLATED) as z:
            for n, data in self.files.items(): z.writestr(n, data)


def make_figures():
    directory = OUT / 'report/assets'; directory.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({'font.family': 'Noto Serif CJK JP',
                         'font.size': 11, 'axes.unicode_minus': False, 'pdf.fonttype': 3})
    blue, orange = '#24649c', '#c36621'
    fig, axes = plt.subplots(1, 3, figsize=(7.5, 2.4), gridspec_kw={'width_ratios': [1.1, 1, 1]})
    a = axes[0]; a.set_xlim(-3.4, 1.8); a.set_ylim(-3.1, 1.9)
    a.plot([-3.1, 1.3], [0, 0], color=blue, lw=1)
    a.plot([0, 0], [-2.8, 1.3], color=orange, lw=1)
    for pos, dest, color, name in [((-2.4, 0), (-.7, 0), blue, 'A'), ((0, -2.1), (0, -.4), orange, 'B')]:
        a.scatter(*pos, c=color, s=25)
        a.annotate('', dest, pos, arrowprops={'arrowstyle': '->', 'color': color, 'lw': 2})
        a.text(pos[0]-.45, pos[1]-.5, name, color=color)
    a.text(.35, .35, '交差点', fontsize=10)
    a.text(-3.1, 1.55, '進行方向・主軸の水平投影', fontsize=8.5)
    a.set_aspect('equal'); a.axis('off')
    for a, color, name, sign in zip(axes[1:], [blue, orange], ['A（20 m）', 'B（30 m）'], [1, -1]):
        a.set_xlim(-.6, 2.8); a.set_ylim(-1.7, 1.7); a.set_aspect('equal'); a.axis('off')
        a.plot([-.3, 2.3], [0, 0], color='.5', lw=.8)
        a.annotate('', (2.3, 0), (0, 0), arrowprops={'arrowstyle': '->', 'color': '.4', 'lw': 1})
        a.annotate('', (1.35, sign*1.35), (0, 0), arrowprops={'arrowstyle': '->', 'color': color, 'lw': 2})
        a.scatter([0], [0], color=color, s=25)
        a.add_patch(Arc((0, 0), 1.5, 1.5, theta1=0 if sign>0 else -45,
                        theta2=45 if sign>0 else 0, color=color, lw=.8))
        a.text(1., sign*.42, '+45°' if sign>0 else '−45°', color=color, va='center')
        a.text(.1, -sign*.65, name, color=color)
        a.text(1.5, -sign*.27, '前方', fontsize=10)
    fig.subplots_adjust(wspace=.28, left=.01, right=.99, top=.98, bottom=.02)
    fig.savefig(directory/'orientation.png', dpi=240); plt.close(fig)

    fig, ax = plt.subplots(figsize=(7.5, 3.5))
    for rev, label, color, style in [('old', '訂正前', '#777777', '--'), ('corrected', '訂正後', blue, '-')]:
        rs = read_csv(OUT/f'radio/dual60_{rev}_curve.csv')
        ax.plot([float(r['remaining_path_m']) for r in rs], [float(r['q_dbm']) for r in rs],
                label=label, color=color, linestyle=style, lw=1.2)
    ax.axhline(-68.5, color='#a32e2e', lw=1, linestyle=':', label='受信しきい値 −68.5 dBm')
    ax.axvline(0, color='.7', lw=.6)
    ax.set(xlim=(30, -60), ylim=(-180, -60), xlabel='最接近地点までの各機の残り距離 [m]', ylabel='計算RSSI [dBm]')
    ax.grid(alpha=.2); ax.legend(frameon=False, loc='center right', fontsize=10)
    for label in [*ax.get_xticklabels(), *ax.get_yticklabels()]:
        label.set_fontfamily('Liberation Serif')
    fig.tight_layout(); fig.savefig(directory/'rssi_comparison.png', dpi=240)
    fig.savefig(directory/'rssi_comparison.pdf'); plt.close(fig)


def main():
    source_hash = hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    audit = json.loads((OUT/'verification.json').read_text())
    assert audit['status'] == 'complete' and audit['trials'] == 1722 and audit['conditions'] == 164
    rows = read_csv(OUT/'trials.csv')
    assert all(r['connected'] == '0' and r['timeouts'] == '0' and float(r['first_request_s']) < 0 for r in rows)
    maxima = {(p, v): read_csv(OUT/f'radio/{p}_{v}_maximum.csv')[0]
              for p in ['original', 'dual60'] for v in ['old', 'corrected']}
    make_figures(); r = Supplement()

    r.paragraph('1．本別紙の位置付け', 'heading')
    r.paragraph('本別紙は「初回リンク確立までのTimeout回数評価」（以下，元資料）の補足である．十字固定シナリオにおけるアンテナの取り付け方向を訂正し，訂正後の再計算と追加考察を示す．元資料の本文・図表は変更せず，本別紙と併せて参照する．図表・式には独立した「補」の番号を付す．')
    r.paragraph('訂正対象は元資料の表1の十字固定RPY，図2の固定アンテナ方向，図5・図9の十字固定の結果と関連する考察である．並行固定，並行追従，十字追従の条件・結果は今回の訂正対象に含めない．')
    r.paragraph('2．アンテナ取り付け方向の訂正', 'heading')
    r.paragraph('正しい条件は，低い機体A（PRDEV）のアンテナを進行方向から上方45°，高い機体B（PRC）のアンテナを進行方向から下方45°に固定するものである．両機とも水平面内の取り付け偏角は0°であり，機首の前方を向く．')
    r.paragraph('表補1　十字固定の訂正前後（RPYは各機の進行方向基準）', 'caption')
    r.table([
        ['項目', '訂正前（元資料）', '訂正後'],
        ['AのRPY', '(0°, −45°, +90°)', '(0°, −45°, 0°)'],
        ['BのRPY', '(0°, +45°, −90°)', '(0°, +45°, 0°)'],
        ['A／Bの仰角', '+45°／−45°', '+45°／−45°（変更なし）'],
        ['世界座標での方位 A／B', '90°／0°', '0°／90°'],
    ], [2442, 3100, 3100])
    r.paragraph('ここではAは世界座標の+x方向，Bは+y方向へ進む．RPYのpitchは上向きで負，仰角は上向きで正となるため，両者の符号を区別する．')
    r.picture(OUT/'report/assets/orientation.png')
    r.paragraph('図補1　訂正後の向き（左：俯瞰図，中央：Aの側面図，右：Bの側面図）', 'caption')

    r.page(); r.paragraph('3．追加計算の条件', 'heading')
    r.paragraph('元資料の開始30 mの速度比較に合わせ，十字固定のみ再計算した．元資料と同じ通信コアを用い，アンテナの水平取り付け偏角だけを変更した．生のE面・H面CSV，合成方法，無線条件，PER表，MCS，Timeout時間は維持した．')
    r.paragraph('表補2　訂正後の再計算条件', 'caption')
    r.table([
        ['項目', '設定'],
        ['無線条件', 'テラ波・ミリ波通信／双方ミリ波通信（60 GHz）'],
        ['航路・高度', '十字航路，A：20 m，B：30 m，最接近距離10 m'],
        ['開始位置・速度', '各機の航路上で30 m手前，1〜5 m/sを0.1 m/s刻み（41点）'],
        ['終了条件', '初回リンク確立時．未確立なら最接近地点通過後60 mまで'],
        ['(i) しきい値モデル', 'RSSI ≥ −68.5 dBmで受信．各条件1試行'],
        ['(ii) 確率モデル', '実測PER表とフレーム長補正．各条件20試行，独立seed'],
        ['接続手順', '接続待ちTimeout 500 ms．(ii)の制御フレームはMCS3'],
        ['試行数', '2無線条件 × 41速度 ×（1 + 20）＝1722試行'],
    ], [2442, 6200])
    r.paragraph('(ii)のMCS3曲線は−68.5 dBm以下でPER=1となり，フレームを短くしても受信成功確率は0である．(i)と(ii)ではMACの再送動作も異なるため，一般には両モデルの差を受信判定だけで説明できない点も元資料と同じである．')
    r.paragraph('4．初回リンク確立とTimeoutの結果', 'heading')
    r.paragraph('表補3　訂正後の速度1〜5 m/sにおける結果', 'caption')
    r.table([
        ['無線条件', 'モデル', '試行数', '確立数', '観測Timeout'],
        ['テラ波・ミリ波', '(i)', '41', '0', '全試行0回'],
        ['テラ波・ミリ波', '(ii)', '820', '0', '全試行0回'],
        ['双方ミリ波', '(i)', '41', '0', '全試行0回'],
        ['双方ミリ波', '(ii)', '820', '0', '全試行0回'],
    ], [2642, 1000, 1200, 1200, 2600])
    r.paragraph('両モデル・両無線条件とも，全確認速度で初回リンクは確立しなかった．全試行でRequest送信開始も記録されておらず，観測Timeoutが0回なのは接続に成功したためではない．「初回確立までのTimeoutが0回で接続した条件」としては扱えず，0回で確立する速度境界も得られていない．')
    r.paragraph('元資料の図5・図9が示す「全確認速度で未接続」という結論は，取り付け方向を訂正した後も変わらなかった．今回の受信電力は確率モデルの成功確率が0となる範囲にあり，接続成功と非成功が混在する境界もないため，600試行への追加反復は行っていない．')

    r.page(); r.paragraph('5．受信電力の変化', 'heading')
    r.paragraph('受信電力を共通の通信コアで別途走査した．各機5 m/s，開始30 mから通過後60 mまでを0.1 ms刻み（各機の移動量0.0005 m）で評価した．同じ航路位置での電力は速度によらず，速度はその位置に到達する時刻を変える．以下の最大値はこの離散走査で確認した値である．')
    r.picture(OUT/'report/assets/rssi_comparison.png')
    r.paragraph('図補2　双方ミリ波通信の訂正前後の計算RSSI（P・Qは同値）', 'caption')
    r.paragraph('横軸は時間の進行に合わせ30 mから−60 mへ配置した．0 mが最接近地点，負値は通過後を表す．縦軸は受信しきい値との関係が読める範囲を示し，−180 dBm未満の計算値は図の範囲外とした．この表示範囲は計算時の減衰打切りではない．')
    r.paragraph('表補4　走査で確認した最大RSSIとその位置', 'caption')
    rr = [['無線条件・方向', '訂正前 [dBm]', '訂正後 [dBm]']]
    for profile, direction, field in [('original', 'テラ波・ミリ波 P', 'p_dbm'), ('original', 'テラ波・ミリ波 Q', 'q_dbm'), ('dual60', '双方ミリ波 P・Q', 'q_dbm')]:
        rr.append([direction, f'{float(maxima[profile,"old"][field]):.2f}', f'{float(maxima[profile,"corrected"][field]):.2f}'])
    rr.append(['最大値の位置（残り距離）', '−0.6795 m（通過後）', '+0.6795 m（接近中）'])
    r.table(rr, [3442, 2600, 2600])
    r.paragraph('最大RSSIは両無線条件ともほぼ同じであり，主な変化は最大値の位置が最接近後から最接近前へ移ることであった．訂正後もテラ波・ミリ波通信のPはしきい値より約8.29 dB，Qは約33.26 dB低い．双方ミリ波通信では両方向とも約29.19 dB低い．')
    r.paragraph('双方ミリ波通信の最大値位置では，機体間距離は約10.046 m，A／Bの合成利得は約0.028／0.336 dBi，自由空間損失は約88.051 dBである．元資料の回路条件を代入すると式(補1)となる（単位はdBm，利得・損失はdB）．')
    r.equation([sub('P','r'), ' = −7 − 3 + 0.028 + 0.336 − 88.051 + 0 + 3 − 3 ≈ −97.69'], 1)

    r.page(); r.paragraph('6．角度訂正だけでは改善しない理由', 'heading')
    r.paragraph('十字航路で前方から上下45°に固定しても，両アンテナが相手に正対するとは限らない．高度差をh=10 m，最接近地点までの各機の残り距離をsとすると，AからBへの相対位置は(s, −s, h)である．Aの主軸は(1, 0, 1)/√2，Bの主軸は(0, 1, −1)/√2となる．両機の主軸から相手方向までの角度θは等しく，式(補2)で表せる．')
    r.equation(['cos θ = ', frac(['s + h'], [sqrt(['2']), ' ', sqrt(['2', sup(['s'], ['2']), ' + ', sup(['h'], ['2'])])])], 2)
    r.paragraph('cos θが最大になるのはs=h/2=5 mであり，このときcos θ=√3/2，すなわち最小角度は30°である．最接近地点s=0では45°となる．したがって，今回の固定条件では，正面方向の約22.17 dBiを両機にそのまま適用できない．')
    r.paragraph('また，主軸からの角度が最小になる位置とRSSIが最大になる位置は一致しない．利得は単一の軸外角だけで決まらず，E面・H面それぞれの角度とCSVの凹凸に依存し，自由空間損失も位置で変化するためである．今回のRSSI最大位置では軸外角は約41.26°であった．')
    r.paragraph('7．実機との照合と本別紙の適用範囲', 'heading')
    r.paragraph('今回の計算では取り付け方向の認識違いを訂正できたが，実機で確認された双方ミリ波通信の接続は再現できなかった．約29.19 dBの不足を，そのまま未計上の内部増幅量と断定することはできない．次の事項を照合する必要がある．')
    r.paragraph('表補5　実機との不一致について残る確認事項', 'caption')
    r.table([
        ['確認項目', '確認する内容'],
        ['配置と取り付け', '高度差，交差のタイミング，前方基準のyaw・pitch・roll，偏波とE面・H面の取り付け対応'],
        ['電力の基準位置', '−7 dBmがどの端子の値か，回路利得・損失と受信感度の基準点が一致するか'],
        ['角度別の利得', '2面CSVからの3次元合成は近似であり，今回の斜め方向の利得が実機と一致するか'],
        ['受信特性の校正', 'PER曲線のdBm軸は−68.5 dBmへ整合した仮定を含む．実機の受信電力と対応付ける'],
    ], [2442, 6200])
    r.paragraph('E面・H面CSVの角度範囲は±90°であり，範囲外は既存実装で極小利得を返す．この領域の極端に低いRSSIは実測電力として解釈しない．表補4の最大値は両面ともCSVの角度範囲内で求められている．')
    r.paragraph('本別紙の結果は，訂正した十字固定・開始30 m・速度1〜5 m/s・元資料と同じ無線条件に対するものである．実機での接続不能を示すものではなく，並行条件や追従条件の結果にも一般化しない．元資料の図5・図9は旧取り付け条件の記録として残し，訂正後の評価には表補3・表補4を用いる．')

    r.save(DEST)
    render(DEST)
    assert hashlib.sha256(SOURCE.read_bytes()).hexdigest() == source_hash
    verification = dict(source=str(SOURCE), source_sha256=source_hash, source_unchanged=True,
                        title=TITLE, document=str(DEST), trials=audit['trials'], conditions=audit['conditions'],
                        font_note='DOCX: MS明朝 / Times New Roman; PDF: Noto Serif CJK JP / Liberation Serif',
                        figures=r.images)
    (OUT/'report/verification.json').write_text(json.dumps(verification, ensure_ascii=False, indent=2))
    print(json.dumps(verification, ensure_ascii=False, indent=2))


if __name__ == '__main__': main()
