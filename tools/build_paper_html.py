#!/usr/bin/env python3
"""
論文Markdownをスタンドアロン HTML に変換する (ホストで実行可、依存なし)。
  python3 tools/build_paper_html.py
- 図 (figures/*.png) は base64 データURIとして埋め込む
- Markdown/数式はブラウザ側で marked + KaTeX (CDN) によりレンダリングする
  (数式区間はプレースホルダで退避してから marked に通し、下線の強調誤解釈を防ぐ)
"""
import base64
import json
import os
import re

MD_PATH = 'docs/paper/kkf_handover_paper.md'
OUT_PATH = 'docs/paper/kkf_handover_paper.html'
FIG_DIR = 'docs/paper'

md = open(MD_PATH, encoding='utf-8').read()

def embed_image(match):
    alt, rel = match.group(1), match.group(2)
    path = os.path.join(FIG_DIR, rel)
    if not os.path.exists(path):
        return match.group(0)
    b64 = base64.b64encode(open(path, 'rb').read()).decode()
    return f'![{alt}](data:image/png;base64,{b64})'

md = re.sub(r'!\[([^\]]*)\]\((figures[\w]*/[^)]+)\)', embed_image, md)

html = """<!DOCTYPE html>
<html lang="ja">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>高速鉄道ミリ波V2I通信における予測ハンドオーバー (ドラフト)</title>
<link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/katex@0.16.9/dist/katex.min.css">
<script src="https://cdn.jsdelivr.net/npm/katex@0.16.9/dist/katex.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/katex@0.16.9/dist/contrib/auto-render.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/marked@11.1.1/marked.min.js"></script>
<style>
body { font-family: "Hiragino Mincho ProN", "Yu Mincho", "Noto Serif JP", serif;
       max-width: 860px; margin: 2rem auto; padding: 0 1.2rem; line-height: 1.9;
       color: #1a1a1a; background: #fff; }
h1 { font-size: 1.5rem; line-height: 1.5; border-bottom: 2px solid #333; padding-bottom: .5rem; }
h2 { font-size: 1.25rem; border-bottom: 1px solid #bbb; padding-bottom: .3rem; margin-top: 2.2rem; }
h3 { font-size: 1.05rem; margin-top: 1.8rem; }
img { max-width: 100%; display: block; margin: 1.2rem auto; }
table { border-collapse: collapse; margin: 1rem auto; font-size: .92rem; }
th, td { border: 1px solid #999; padding: .35rem .7rem; }
th { background: #f2f2f2; }
em { color: #444; }
.katex-display { overflow-x: auto; padding: .2rem 0; }
@media print { body { max-width: none; } }
</style>
</head>
<body>
<div id="content">読み込み中… (数式表示にはネットワーク接続が必要です)</div>
<script>
const SRC = __MD_JSON__;
document.addEventListener('DOMContentLoaded', () => {
  const stash = [];
  let s = SRC.replace(/\\$\\$([\\s\\S]+?)\\$\\$/g,
      m => { stash.push(m); return '\\u0001' + (stash.length - 1) + '\\u0001'; });
  s = s.replace(/\\$([^\\$\\n]+?)\\$/g,
      m => { stash.push(m); return '\\u0001' + (stash.length - 1) + '\\u0001'; });
  let html = marked.parse(s);
  html = html.replace(/\\u0001(\\d+)\\u0001/g, (_, i) => stash[Number(i)]);
  const el = document.getElementById('content');
  el.innerHTML = html;
  renderMathInElement(el, { delimiters: [
    { left: '$$', right: '$$', display: true },
    { left: '$', right: '$', display: false } ] });
});
</script>
</body>
</html>
"""
html = html.replace('__MD_JSON__', json.dumps(md))
open(OUT_PATH, 'w', encoding='utf-8').write(html)
print(f'wrote {OUT_PATH} ({os.path.getsize(OUT_PATH) / 1e6:.1f} MB)')
