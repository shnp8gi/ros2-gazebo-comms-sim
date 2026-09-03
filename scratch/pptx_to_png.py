#!/usr/bin/env python3
"""説明図の pptx を PNG に書き出す (LibreOffice で PDF 化 → PyMuPDF でラスタ化)。

pptx が編集可能な原本で、PNG は文書に貼るための書き出し。
図の中身を直すときは pptx を直してからこれを流す。
"""
import os
import subprocess
import sys

import pymupdf

REV = '/workspace/scratch/review'
FIGS = f'{REV}/figs'
TMP = '/tmp/loconv'
DPI = 220

# (pptx, スライド番号 1 起点, 出力名)
JOBS = [('figures_explain.pptx', 1, 'A_problem.png'),
        ('figures_explain.pptx', 2, 'B_layout.png'),
        ('figures_explain.pptx', 3, 'C_pipeline.png'),
        ('system_flowchart_v2.pptx', 1, 'E_flow.png')]

os.makedirs(FIGS, exist_ok=True)
os.makedirs(TMP, exist_ok=True)
for src in sorted({j[0] for j in JOBS}):
    subprocess.run(['soffice', '--headless', '--convert-to', 'pdf',
                    '--outdir', TMP, os.path.join(REV, src)],
                   check=True, capture_output=True)

for src, page, out in JOBS:
    pdf = os.path.join(TMP, os.path.splitext(src)[0] + '.pdf')
    doc = pymupdf.open(pdf)
    if page > len(doc):
        sys.exit(f'{src} に {page} ページ目がない')
    pix = doc[page - 1].get_pixmap(dpi=DPI)
    dst = os.path.join(FIGS, out)
    pix.save(dst)
    print(f'  {out}  {pix.width}x{pix.height}')
    doc.close()
print('完了')
