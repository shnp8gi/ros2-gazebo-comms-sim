#!/usr/bin/env python3
"""system_flowchart.pptx のテキストを修正し、数式を PowerPoint の数式
オブジェクト (挿入>数式 = OMML) に置き換える。

・KF の記載を削除 / 接続閾値を規格値へ / 残差クリギングを明示
・英数字 Times New Roman、日本語 MS 明朝
・数式は a14:m でくるんだ m:oMath として書き、PowerPoint が数式として編集できる形にする
"""
import os
import re
import zipfile

SRC = 'scratch/review/system_flowchart.pptx'
DST = 'scratch/review/system_flowchart_v2.pptx'
LATIN, EA = 'Times New Roman', 'MS Mincho'

# 実行文(run)の差し替え。値は (種別, 文字列) の並び。't'=通常文字 'm'=数式
#   数式の要素: ('sym', 文字) / ('sub', 基底, 下付き) / ('subsup', 基底, 下付き, 上付き)
SEGS = {
    'KF 更新':        [('t', '係数 '), ('m', [('sym', 'α')]), ('t', ' の逐次更新')],
    '式 (19)–(23)':   [('t', '（ベイズ線形回帰）')],
    'ν = Z − Φα':     [('m', [('sym', 'ν = Z − Φα')])],
    'LCB = μ − κσ':   [('m', [('sym', 'LCB = μ − κσ')])],
    'μ ≥ −78.5 dBm':  [('m', [('sym', 'μ ≥ −68.5 dBm')])],
    '予測（式 25）':   [('t', '予測（空間基底 + 残差クリギング）')],
    '予測値 μ と不確かさ σ':
        [('t', '予測値 '), ('m', [('sym', 'μ')]), ('t', ' と不確かさ '),
         ('m', [('sym', 'σ')])],
    'σν²(s) の更新':
        [('m', [('subsup', 'σ', 'ν', '2'), ('sym', '(s)')]), ('t', ' の更新')],
    '電波地図 32 枚（α, P, σν²）':
        [('t', '電波地図 32 枚（'), ('m', [('sym', 'α, P, ')]),
         ('m', [('subsup', 'σ', 'ν', '2')]), ('t', '）')],
    '破線：学習済み地図の受け渡しと，割当中ペアの観測による走行内の逐次更新':
        [('t', '破線：学習済み地図の受け渡しと，割当中ペアの観測の帰還'
               '（残差バッファ → クリギング補正／係数の更新）')],
}

MNS = 'http://schemas.openxmlformats.org/officeDocument/2006/math'
ANS = 'http://schemas.openxmlformats.org/drawingml/2006/main'
A14 = 'http://schemas.microsoft.com/office/drawing/2010/main'


def esc(s):
    return (s.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;'))


def m_run(text, italic=True):
    """数式内の 1 実行文。変数は斜体、それ以外は立体。"""
    nor = '' if italic else '<m:nor/>'
    return (f'<m:r><m:rPr>{nor}</m:rPr>'
            f'<a:rPr xmlns:a="{ANS}"><a:latin typeface="{LATIN}"/>'
            f'<a:ea typeface="{EA}"/></a:rPr>'
            f'<m:t>{esc(text)}</m:t></m:r>')


def m_body(elems):
    out = []
    for e in elems:
        if e[0] == 'sym':
            # 1 文字の英字・ギリシャ文字だけ斜体にする
            for tok in re.findall(r'[A-Za-zα-ωΑ-Ω]|[^A-Za-zα-ωΑ-Ω]+', e[1]):
                out.append(m_run(tok, italic=bool(re.fullmatch(
                    r'[A-Za-zα-ωΑ-Ω]', tok))))
        elif e[0] == 'subsup':
            out.append(
                '<m:sSubSup><m:sSubSupPr><m:ctrlPr/></m:sSubSupPr>'
                f'<m:e>{m_run(e[1])}</m:e>'
                f'<m:sub>{m_run(e[2])}</m:sub>'
                f'<m:sup>{m_run(e[3], italic=False)}</m:sup></m:sSubSup>')
    return ''.join(out)


def build(segs, rpr):
    """rPr (書式) を引き継いで、通常文字と数式が交互に並ぶ XML を組む。"""
    out = []
    for kind, val in segs:
        if kind == 't':
            out.append(f'<a:r>{rpr}<a:t>{esc(val)}</a:t></a:r>')
        else:
            out.append(f'<a14:m xmlns:a14="{A14}">'
                       f'<m:oMath xmlns:m="{MNS}">{m_body(val)}</m:oMath>'
                       '</a14:m>')
    return ''.join(out)


zin = zipfile.ZipFile(SRC)
zout = zipfile.ZipFile(DST, 'w', zipfile.ZIP_DEFLATED)
hits = {k: 0 for k in SEGS}
nfont = 0
for item in zin.infolist():
    data = zin.read(item.filename)
    if re.match(r'ppt/slides/slide\d+\.xml$', item.filename):
        s = data.decode('utf-8')
        s, nfont = re.subn(r'<a:latin typeface="MS Mincho"',
                           f'<a:latin typeface="{LATIN}"', s)
        for old, segs in SEGS.items():
            # <a:r> ... <a:t>old</a:t></a:r> を丸ごと差し替える
            # rPr の探索が </a:r> を越えて隣の run を飲み込まないように閉じる
            pat = re.compile(r'<a:r>(?P<rpr>(?:(?!</a:r>).)*?)<a:t>'
                             + re.escape(old) + r'</a:t></a:r>', re.S)

            def rep(m, segs=segs, key=old):
                hits[key] += 1
                return build(segs, m.group('rpr') or '')
            s = pat.sub(rep, s)
        data = s.encode('utf-8')
    zout.writestr(item, data)
zout.close()

print(f'latin フォント差し替え: {nfont} 箇所')
for k, v in hits.items():
    print(f'  {"OK " if v == 1 else "!! "}{v} 件  {k[:40]}')
print(f'\nwrote {DST}')
