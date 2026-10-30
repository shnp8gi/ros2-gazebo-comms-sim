"""指定Wordの表・式・ヘッダー書式を引き継ぎ、本計算の結果を報告する。"""
import argparse
from copy import deepcopy
from datetime import datetime
import json
import math
import os
from pathlib import Path
import re
import subprocess
import tempfile
from zipfile import ZipFile, ZIP_DEFLATED
from lxml import etree as E
from PIL import Image
import yaml
from raw_antenna_campaign_common import read_csv

NS={'w':'http://schemas.openxmlformats.org/wordprocessingml/2006/main',
    'm':'http://schemas.openxmlformats.org/officeDocument/2006/math',
    'r':'http://schemas.openxmlformats.org/officeDocument/2006/relationships',
    'rel':'http://schemas.openxmlformats.org/package/2006/relationships'}


def q(name):prefix,tag=name.split(':');return '{'+NS[prefix]+'}'+tag
def node(name,**attrs):return E.Element(q(name),{q('w:'+k):str(v) for k,v in attrs.items()})
def xml(e):return E.tostring(e,xml_declaration=True,encoding='UTF-8',standalone=True)
def mr(text):
    r=node('m:r');pr=node('w:rPr');pr.append(node('w:rFonts',ascii='Cambria Math',hAnsi='Cambria Math'))
    pr.append(node('w:sz',val=21));r.append(pr);t=node('m:t');t.text=str(text);r.append(t);return r
def sub(base,index):
    s=node('m:sSub');a=node('m:e');a.append(mr(base));b=node('m:sub');b.append(mr(index));s.extend([a,b]);return s
def frac(numerator,denominator):
    f=node('m:f')
    for name,values in [('m:num',numerator),('m:den',denominator)]:
        n=node(name);n.extend([mr(v) if isinstance(v,str) else v for v in values]);f.append(n)
    return f
def sup(base,exponent):
    s=node('m:sSup')
    for tag,values in [('m:e',base),('m:sup',exponent)]:
        n=node(tag);n.extend([mr(v) if isinstance(v,str) else v for v in values]);s.append(n)
    return s


class Report:
    def __init__(self,template):
        with ZipFile(template) as z:self.files={n:z.read(n) for n in z.namelist()}
        self.doc=E.fromstring(self.files['word/document.xml']);self.body=self.doc.find('w:body',NS)
        self.prototypes={i:deepcopy(self.body[i]) for i in [0,1,6,8,11,45,46]}
        section=deepcopy(self.body.find('w:sectPr',NS));self.body.clear();self.body.append(section)
        self.rels=E.fromstring(self.files['word/_rels/document.xml.rels'])
        allowed={'styles','settings','webSettings','fontTable','theme','numbering','header','footnotes','endnotes'}
        for r in list(self.rels):
            if r.get('Type').rsplit('/',1)[-1] not in allowed:self.rels.remove(r)
        self.image_id=1000;self.images=[];self.next_page=False
        # 原稿のコメント・他研究の画像・埋込メタデータは新レポートに含めない。
        keep={'[Content_Types].xml','_rels/.rels','word/document.xml','word/_rels/document.xml.rels',
              'word/header1.xml','word/_rels/header1.xml.rels','word/styles.xml','word/numbering.xml',
              'word/settings.xml','word/fontTable.xml','word/webSettings.xml','word/theme/theme1.xml',
              'word/footnotes.xml','word/endnotes.xml','docProps/core.xml','docProps/app.xml',
              'word/media/image20.wmf','word/embeddings/oleObject1.bin'}
        self.files={n:b for n,b in self.files.items() if n in keep}
        for name in ['footnotes','endnotes']:
            e=E.fromstring(self.files[f'word/{name}.xml'])
            for child in list(e):
                if int(child.get(q('w:id'),'0'))>0:e.remove(child)
            self.files[f'word/{name}.xml']=xml(e)
        rel=E.fromstring(self.files['_rels/.rels'])
        for r in list(rel):
            if r.get('Target').lstrip('/') not in self.files:rel.remove(r)
        self.files['_rels/.rels']=xml(rel)
        styles=E.fromstring(self.files['word/styles.xml'])
        for f in styles.xpath('//w:docDefaults//w:rFonts',namespaces=NS):
            f.set(q('w:ascii'),'Times New Roman');f.set(q('w:hAnsi'),'Times New Roman');f.set(q('w:eastAsia'),'ＭＳ 明朝')
        self.files['word/styles.xml']=xml(styles)
        header=E.fromstring(self.files['word/header1.xml'])
        for p in header.findall('.//w:p',NS):
            text=''.join(p.xpath('.//w:t/text()',namespaces=NS))
            if text=='学習済み電磁環境マップに基づく能動的ハンドオーバー':
                self.replace_text(p,'生CSVアンテナ利得を用いた初回リンク確立の評価')
            elif text.startswith('2026/'):
                self.replace_text(p,datetime.now().strftime('%Y/%m/%d'))
        self.files['word/header1.xml']=xml(header)
        settings=E.fromstring(self.files['word/settings.xml'])
        settings.append(node('w:updateFields',val='true'));self.files['word/settings.xml']=xml(settings)

    @staticmethod
    def replace_text(p,text):
        rpr=p.find('w:r/w:rPr',NS);ppr=p.find('w:pPr',NS)
        rpr=deepcopy(rpr) if rpr is not None else None
        for child in list(p):
            if child is not ppr:p.remove(child)
        r=node('w:r')
        if rpr is not None:r.append(rpr)
        for i,line in enumerate(text.split('\n')):
            if i:r.append(node('w:br'))
            t=node('w:t');t.text=line;t.set('{http://www.w3.org/XML/1998/namespace}space','preserve');r.append(t)
        p.append(r)

    def append(self,e):self.body.insert(len(self.body)-1,e);return e

    def paragraph(self,text,kind='body'):
        p=deepcopy(self.prototypes[{'body':1,'heading':0,'subheading':8,'caption':45}[kind]])
        self.replace_text(p,text)
        # 参照文書の強制改ページは書式ではなく元の内容に属するため引き継がない。
        for br in p.findall('.//w:br',NS):br.getparent().remove(br)
        pr=p.find('w:pPr',NS)
        if self.next_page:
            pr.append(node('w:pageBreakBefore'));self.next_page=False
        if kind in ['heading','subheading','caption']:pr.append(node('w:keepNext'))
        if kind=='caption':pr.append(node('w:spacing',before=90,after=70))
        for fonts in p.findall('.//w:rFonts',NS):
            for name in ['asciiTheme','hAnsiTheme','eastAsiaTheme']:fonts.attrib.pop(q('w:'+name),None)
            fonts.set(q('w:ascii'),'Times New Roman');fonts.set(q('w:hAnsi'),'Times New Roman');fonts.set(q('w:eastAsia'),'ＭＳ 明朝')
        self.append(p);return p

    def page(self):
        # 独立した改ページ段落が次ページへ押し出される空白ページを防ぐ。
        self.next_page=True

    def table(self,rows,widths=None):
        source=self.prototypes[46];table=node('w:tbl');table.append(deepcopy(source.find('w:tblPr',NS)))
        widths=widths or [2122,6520];grid=node('w:tblGrid')
        for width in widths:grid.append(node('w:gridCol',w=width))
        table.append(grid);rowproto=source.find('w:tr',NS)
        for row in rows:
            assert len(row)==len(widths)
            tr=node('w:tr');tr.append(deepcopy(rowproto.find('w:trPr',NS)))
            for i,(text,width) in enumerate(zip(row,widths)):
                tc=deepcopy(rowproto.findall('w:tc',NS)[min(i,1)]);tc.find('w:tcPr/w:tcW',NS).set(q('w:w'),str(width))
                p=tc.find('w:p',NS);self.replace_text(p,str(text))
                # 表の9 ptと罫線・セル余白を維持し、多行セルの欠けだけ防ぐ。
                spacing=p.find('w:pPr/w:spacing',NS);spacing.set(q('w:lineRule'),'atLeast')
                table_tr=tr;table_tr.append(tc)
            table.append(tr)
        self.append(table);return table

    def equation(self,parts,number):
        table=deepcopy(self.prototypes[11]);cells=table.find('w:tr',NS).findall('w:tc',NS)
        for cell in cells:
            p=cell.find('w:p',NS);pr=deepcopy(p.find('w:pPr',NS));p.clear();p.append(pr)
        middle=cells[1].find('w:p',NS);para=node('m:oMathPara');math=node('m:oMath')
        math.extend([mr(p) if isinstance(p,str) else p for p in parts]);para.append(math);middle.append(para)
        self.replace_text(cells[2].find('w:p',NS),f'({number})')
        number_run=cells[2].find('w:p/w:r',NS);pr=node('w:rPr');pr.append(node('w:sz',val=21))
        pr.append(node('w:rFonts',ascii='Times New Roman',hAnsi='Times New Roman'));number_run.insert(0,pr)
        self.append(table)

    def picture(self,path):
        self.image_id+=1;i=self.image_id;rid=f'rIdRaw{i}';name=f'raw_result_{i}.png'
        width=6.0*914400
        with Image.open(path) as im:height=width*im.height/im.width
        self.files['word/media/'+name]=Path(path).read_bytes()
        E.SubElement(self.rels,q('rel:Relationship'),Id=rid,Type=NS['r']+'/image',Target='media/'+name)
        p=E.fromstring(f'''<w:p xmlns:w="{NS['w']}" xmlns:r="{NS['r']}" xmlns:wp="http://schemas.openxmlformats.org/drawingml/2006/wordprocessingDrawing" xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main" xmlns:pic="http://schemas.openxmlformats.org/drawingml/2006/picture"><w:pPr><w:jc w:val="center"/><w:keepNext/></w:pPr><w:r><w:drawing><wp:inline distT="0" distB="0" distL="0" distR="0"><wp:extent cx="{int(width)}" cy="{int(height)}"/><wp:docPr id="{i}" name="Result {i}"/><wp:cNvGraphicFramePr><a:graphicFrameLocks noChangeAspect="1"/></wp:cNvGraphicFramePr><a:graphic><a:graphicData uri="http://schemas.openxmlformats.org/drawingml/2006/picture"><pic:pic><pic:nvPicPr><pic:cNvPr id="{i}" name="{name}"/><pic:cNvPicPr/></pic:nvPicPr><pic:blipFill><a:blip r:embed="{rid}"/><a:stretch><a:fillRect/></a:stretch></pic:blipFill><pic:spPr><a:xfrm><a:off x="0" y="0"/><a:ext cx="{int(width)}" cy="{int(height)}"/></a:xfrm><a:prstGeom prst="rect"><a:avLst/></a:prstGeom></pic:spPr></pic:pic></a:graphicData></a:graphic></wp:inline></w:drawing></w:r></w:p>''')
        self.append(p);self.images.append(str(path))

    def save(self,path):
        for e in self.body.iter():
            for attr in list(e.attrib):
                local=E.QName(attr).localname
                if local in ['paraId','textId'] or local.startswith('rsid'):del e.attrib[attr]
        self.files['word/document.xml']=xml(self.doc);self.files['word/_rels/document.xml.rels']=xml(self.rels)
        ct=E.fromstring(self.files['[Content_Types].xml'])
        for entry in list(ct):
            if entry.tag.endswith('Override') and entry.get('PartName').lstrip('/') not in self.files:ct.remove(entry)
        self.files['[Content_Types].xml']=xml(ct)
        core=E.fromstring(self.files['docProps/core.xml'])
        for e in core:
            if E.QName(e).localname=='title':e.text='生CSVアンテナ利得を用いた初回リンク確立の評価'
            elif E.QName(e).localname in ['created','modified']:e.text=datetime.now().astimezone().isoformat(timespec='seconds')
        self.files['docProps/core.xml']=xml(core)
        path.parent.mkdir(parents=True,exist_ok=True)
        with ZipFile(path,'w',ZIP_DEFLATED) as z:
            for name,data in self.files.items():z.writestr(name,data)


def render(path):
    with tempfile.TemporaryDirectory(prefix='raw_report_lo_') as temp:
        config=Path(temp)/'fonts.conf'
        config.write_text('''<?xml version="1.0"?><!DOCTYPE fontconfig SYSTEM "fonts.dtd"><fontconfig><include>/etc/fonts/fonts.conf</include><alias><family>ＭＳ 明朝</family><prefer><family>Noto Serif CJK JP</family></prefer></alias><alias><family>MS Mincho</family><prefer><family>Noto Serif CJK JP</family></prefer></alias><alias><family>Times New Roman</family><prefer><family>Liberation Serif</family></prefer></alias></fontconfig>''')
        env=dict(os.environ,FONTCONFIG_FILE=str(config))
        subprocess.run(['libreoffice','-env:UserInstallation='+Path(temp).as_uri()+'/profile','--headless',
            '--convert-to','pdf','--outdir',str(path.parent),str(path)],env=env,check=True,timeout=180)
    assert path.with_suffix('.pdf').exists(),'PDFが作成されませんでした'
    info=subprocess.check_output(['pdfinfo',str(path.with_suffix('.pdf'))],text=True)
    pages=int(re.search(r'^Pages:\s+(\d+)',info,re.M).group(1))
    # Wordがフィールドを更新する前のプレビューにも正しい総ページ数を保存する。
    with ZipFile(path) as z:files={n:z.read(n) for n in z.namelist()}
    header=E.fromstring(files['word/header1.xml'])
    for p in header.findall('.//w:p',NS):
        field='';show=False
        for run in p.findall('w:r',NS):
            char=run.find('w:fldChar',NS)
            if char is not None:
                kind=char.get(q('w:fldCharType'))
                if kind=='begin':field='';show=False
                elif kind=='separate':show=True
                elif kind=='end':field='';show=False
            instruction=run.find('w:instrText',NS)
            if instruction is not None:field+=instruction.text or ''
            if show and field.strip()=='NUMPAGES':
                for text in run.findall('w:t',NS):text.text=str(pages)
    files['word/header1.xml']=xml(header)
    app=E.fromstring(files['docProps/app.xml'])
    for e in app:
        if E.QName(e).localname=='Pages':e.text=str(pages)
    files['docProps/app.xml']=xml(app)
    with ZipFile(path,'w',ZIP_DEFLATED) as z:
        for name,data in files.items():z.writestr(name,data)


def interval_text(rows):
    if not rows:return '境界を観測せず'
    names={'near_zero':'0回付近','unconnected':'未接続','nonzero_or_unreliable':'非0／接続不安定'}
    return '\n'.join(f"{float(r['lower']):g}–{float(r['upper']):g}：{names[r['lower_class']]}→{names[r['upper_class']]}" for r in rows)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('directory',type=Path);parser.add_argument('template',type=Path)
    parser.add_argument('--format-check',action='store_true');a=parser.parse_args();root=a.directory.resolve();r=Report(a.template)
    if a.format_check:
        r.paragraph('書式確認（計算結果ではありません）','heading')
        r.paragraph('指定文書からヘッダー，表の罫線・余白，式の中央配置と右端番号を引き継いだ書式確認用文書である．')
        r.paragraph('表1　書式確認','caption');r.table([['項目','内容'],['本文','10.5 pt，明朝体・Times New Roman'],['式','中央の数式と右端の式番号']])
        r.equation([sub('P','r'),' = ',sub('P','t'),' − ',sub('L','FS')],1)
        path=root/'format_check.docx';r.save(path);render(path);print(path);return
    verification=json.loads((root/'analysis/verification.json').read_text())
    replays=json.loads((root/'analysis/replay_verification.json').read_text());assert replays['all_normal_gazebo_matches']
    summary=read_csv(root/'analysis/position_summary.csv');limits=read_csv(root/'analysis/position_boundaries.csv')
    overview=read_csv(root/'analysis/condition_overview.csv');speed_limits=read_csv(root/'analysis/speed_boundaries.csv')
    uniform30=(root/'speed_30m/completed').exists()
    baseline=30 if uniform30 else 25
    speed_summary=read_csv(root/'analysis/speed_30m_summary.csv' if uniform30 else root/'analysis/fixed_speed_summary.csv')
    if uniform30:speed_limits=read_csv(root/'analysis/speed_30m_boundaries.csv')
    configs={p:yaml.safe_load((root/'configs'/p/'statistical.yaml').read_text()) for p in ['original','dual60']}
    r.paragraph('はじめに','heading')
    r.paragraph('移動するPRDEV（A）とPRC（B）の初回リンク確立について，各機の飛行速度と通信開始位置がTimeout回数に与える影響を調べた．並行・十字の航路と固定・追従アンテナを組み合わせた4パターンに，両機を60 GHz・−7 dBmとする十字固定の追加条件を加え，2種類の受信モデルを比較した．')
    r.paragraph('第1段階では従来4パターンすべてを各機の開始距離30 mにそろえ，速度1〜5 m/sを0.1 m/s刻みで比較する．第2段階では追従2パターンに絞って開始距離の影響を追加検討する．最後に双方向60 GHz・十字固定を同じ開始30 mの条件で比較し，実機との不一致を考察する．' if uniform30 else '追従は開始距離，固定は飛行速度を横軸として比較する．')
    r.paragraph(f"計算は{verification['groups']}組，開始位置{verification['conditions']:,}条件，速度比較{verification.get('speed_30m_conditions',verification['speed_conditions']):,}条件を含め計{verification['trials']:,}試行を実施した．Timeoutは初回リンク確立より前の発生回数とし，一度も接続しなかった試行を0回接続と混同しない．本報告は生CSV利得による再計算結果であり，従来の正面利得補正・減衰打切りを用いた結果を混在させていない．")
    r.paragraph('評価条件','heading')
    r.paragraph('軌道とアンテナ','subheading')
    r.paragraph('Q方向はA（PRDEV）→B（PRC），P方向はB→Aである．両機は同じ速度で移動し，並行では反対向きに飛行する．開始距離sは各機の航路に沿った最接近地点までの距離を表す．未接続時の終了位置は最接近地点を通過してから60 mに固定し，観測時間の上限を式(1)とする．反復計算は初回リンク確立時点で終了し，確立後の切断は評価対象としない．')
    r.equation([sub('T','obs'),' = ',frac(['s + 60'],['v'])],1)
    r.paragraph('表1　軌道と探索条件','caption')
    r.table([['項目','設定'],['高度 A／B','20 m／30 m'],['並行の横方向間隔','10 m'],['各機の速度','基礎比較：1〜5 m/s，0.1 m/s刻み' if uniform30 else '1，2，3，4，5 m/s'],['開始距離','0.1 m，1〜60 m（粗探索1 m，境界0.1 m）'],['固定アンテナ仰角 A／B','+45°／−45°'],['固定方位（+X基準）','A：+90°，B：並行−90°／十字0°'],['追従アンテナ','相手機体を3次元的に指向'],['速度比較の開始距離',f'各機{baseline} m（航路上の最接近地点まで）']])
    r.page();r.paragraph('受信電力の計算','subheading')
    r.paragraph('受信電力は自由空間損失，送受信アンテナ利得，回路の損失・利得から式(2)で求める．ここで示すRSSIは回路損失・利得を含む受信機入力の計算値であり，アンテナ端子の電力とは基準点が異なる．')
    r.equation([sub('P','r'),' = ',sub('P','t'),' − ',sub('L','tx'),' + ',sub('G','t'),' + ',sub('G','r'),' − ',sub('L','FS'),' + ',sub('G','LNA'),' + ',sub('G','conv'),' − ',sub('L','rx')],2)
    r.equation([sub('L','FS'),' = 20 ',sub('log','10'),'(',frac(['4πdf'],['c']),')'],3)
    r.paragraph('dは機体間の3次元距離，fは周波数，cは光速である．E面・H面のCSVを角度補間して用い，既存の合成方法を維持する．合成利得は式(4)で表され，Gmaxは両面CSVの最大値である．正面をP=24 dBi・Q=23 dBiへ合わせる補正と，最大減衰30 dBの打切りをともに外した．')
    r.equation(['G(θ,φ) = ',sub('G','E'),'(θ) + ',sub('G','H'),'(φ) − ',sub('G','max')],4)
    r.paragraph('表2　方向別の無線条件','caption')
    radios=[configs['original']['radio_ba'],configs['original']['radio_ab'],configs['dual60']['radio_ab']]
    def vals(key,scale=1):return [f'{float(c[key])*scale:g}' for c in radios]
    r.table([['項目','従来 P','従来 Q','追加 P・Q'],['周波数 [GHz]',*vals('frequency_hz',1e-9)],
        ['送信電力 [dBm]',*[f'{10*math.log10(c["power_mw"]):g}' for c in radios]],
        ['送信損失 [dB]',*vals('tx_insertion_loss_db')],['受信LNA利得 [dB]',*vals('rx_lna_gain_db')],
        ['受信変換利得 [dB]',*vals('rx_conversion_gain_db')],['受信損失 [dB]',*vals('rx_insertion_loss_db')]],
        [3200,1814,1814,1814])
    def circuit_offset(c):
        return 10*math.log10(c['power_mw'])-c['tx_insertion_loss_db']+c['rx_lna_gain_db']+c['rx_conversion_gain_db']-c['rx_insertion_loss_db']
    power_difference=circuit_offset(radios[0])-circuit_offset(radios[1])-20*math.log10(float(radios[0]['frequency_hz'])/float(radios[1]['frequency_hz']))
    r.paragraph(f'従来条件では，送信電力・回路利得と周波数の差により，P方向の計算受信電力がQ方向より約{power_difference:.2f} dB高くなる．このため，P方向のBeaconを受信してもQ方向のRequestが届かない区間が生じ得る．')
    r.paragraph('追加条件はPRDEV相当の回路からLNAを取り除き，両方向を同じ構成とした．アンプ非線形歪みは再現せず，既存の理想線形化用受信曲線を使用した．NFによる追加のSNR受信ゲート，Expected RSSIによる送信抑制，強制減衰は導入していない．')
    r.page();r.paragraph('受信判定とプロトコル','heading')
    r.paragraph('①しきい値モデル','subheading')
    r.paragraph('計算RSSIが−68.5 dBm以上のときに受信可能と判定する．各条件は1試行であり，同じ入力に対して決定的な結果となる．既存の簡易MACを使用する．')
    r.paragraph('②実測曲線を用いる確率モデル','subheading')
    r.paragraph('今回の実装はBERからPERを新規生成するものではなく，既存の実測PER–RSSI表を参照する．RSSI間はPERを線形補間し，基準長Lref=100000 bytesのPERを対象フレーム長Lに式(5)で換算する．受信成否はフレームごとに独立な一様乱数で判定する．時間相関は加えない．')
    r.equation([sub('p','L'),' = 1 − ',sup(['(1 − ',sub('p','ref'),')'],[frac(['L'],[sub('L','ref')])])],5)
    r.paragraph('PER=1の点は長さ補正後も1であり，未観測の成功を仮定しない．表の下限RSSIより弱い領域はPER=1，上限より強い領域は上端値を使用する．Beacon，Request，Response，ACK，DATA_ACK，DisassociateはMCS3とし，DATAは既存MCStable.csvと速度・MCS対応表を参照する．DATA長は2048 bytesである．')
    r.paragraph('②は波形詳細モデルと共通のHRCP MACを用いる．①とはMAC実装も異なるため，両者の差は受信判定だけに起因すると解釈しない．接続待ちTimeoutは500 msを維持する．制御フレーム時間を実機TJXへ完全校正したものではない．')
    r.paragraph('具体的には，①はRequestが未達になると500 msの応答待ちへ進む．②では最初のRequestからのTimeout時刻を延長せず，条件を満たすBeaconを受信するたびにRequestを再送し得る．このため，受信可能になる時刻が近くても，初回リンク確立までのTimeout回数が異なり得る．')
    r.paragraph('曲線の校正上の仮定','subheading')
    r.paragraph('受信曲線の電力軸は，MCS3の全行PER=1となる最初の減衰56.3 dBを−68.5 dBmへ置く既存仮定に基づく．絶対受信感度を今回新たに測定したものではない．下りQの測定曲線を未測定の上りPにも共用する．100 kBから短い制御フレームへの換算は誤りの独立性を仮定し，長さ別実測による校正ではない．')
    r.paragraph('既存の理想線形化曲線では，歪みが疑われるMCS11の減衰20〜27.5 dBの実測部分を除外し，27.5 dBに仮定の誤りゼロ点を置いている．これらの仮定を含むため，実機の接続境界との完全一致を主張するものではない．')
    r.page();r.paragraph('集計方法と検証','heading')
    r.paragraph('初回リンク確立までのTimeoutを各試行で計数し，②では試行平均を求めた．一度もリンク確立しなかった試行では，観測終了までのTimeoutを平均へ含める．未接続の0回は，Timeout待ちにすら到達しない状態を含むため，0回での接続成功と区別する．')
    r.paragraph('②は粗探索20試行，境界と確認点600試行を基本とする．平均Timeoutが0.01回以下，かつTimeoutなし接続率が99%以上の計算点を「0回付近」と定義した．これは有限試行による分類であり，真の失敗確率が厳密に0であるという意味ではない．')
    probabilistic=[d for d in summary if d['model']=='statistical']
    varied=sum(int(d['min_timeouts'])!=int(d['max_timeouts']) for d in probabilistic)
    r.paragraph(f'②の開始位置比較{len(probabilistic)}条件のうち，Timeout回数に試行間の差が出た条件は{varied}件であった．各フレームの成否は独立抽選であるが，フレーム単位のばらつきがTimeout回数の差として必ず現れるわけではない．')
    repeated=[d for d in probabilistic if int(d['trials'])>=600 and int(d['connected_trials'])>1]
    varied_time=[d for d in repeated if float(d['first_connected_spread_ms'])>1e-7]
    if repeated:
        largest=max(float(d['first_connected_spread_ms']) for d in repeated)
        r.paragraph(f'600試行以上を実施した接続条件{len(repeated)}件のうち，{len(varied_time)}件では初回接続時刻に試行間差があり，最大差は{largest:.3f} msであった．時刻の変動と500 msのTimeout回数を区別して確認する必要がある．')
        r.paragraph('本条件のBeacon間隔は約0.14 msであり，②では500 msの接続待ちの間に多数の再試行機会がある．受信成功の可能性がある領域へ入ると，あるフレームが失敗しても次の機会で成功しやすく，接続時刻の小さな差にとどまると考えられる．ただし，PER=1の領域では反復によって受信可能になるわけではない．')
    r.paragraph('開始距離比較は1 m刻みの隣接点で区分が変わる領域を0.1 m刻みで調べた．速度比較は開始30 mで全範囲を0.1 m/s刻みとし，②の通常条件は20試行，0回付近および区分境界の両側は600試行で確認した．未計算点の狭い非単調区間や，確認範囲外の境界までは保証しない．' if uniform30 else '開始距離は1 m刻み，境界は0.1 m刻みで調べた．固定の速度境界は0.1 m/s刻みで確認した．')
    unstable=verification['convergence_unstable']
    r.paragraph(f"600試行以上の{verification['convergence_checked']}条件について前半・後半の平均を比較した．平均差0.05回以下かつ両半分の区分一致を安定性の目安とし，満たさない条件は600試行ずつ最大3000試行まで追加した．最終的に基準を満たさなかった条件は{unstable}件である．"+('該当条件はconvergence_audit.csvに明記し，境界の解釈を保留する．' if unstable else 'この確認は信頼区間による評価ではない．'))
    r.paragraph(f"{replays['verified_cases']}代表条件を通常Gazebo経路で記録し，初回接続時刻（許容差10⁻⁹ s）と初回接続までのTimeout回数が高速反復計算と一致することを確認した．新規の接続成功例は初回確立直後まで，未接続例は観測終了までを記録した．既存の全区間記録が同じ条件・seedで存在する場合はそれを再利用した．②の再生記録はフレーム単位の確率判定も既存検証ツールで照合した．入力ファイルのSHA-256一致と全試行seedの一意性を確認した．")
    r.paragraph('再生用の30条件は，各速度6件，各パターン6件，①②各15件となるように選んだ．開始25 mの比較に加え，0回付近の境界両側を含めた．全条件で未接続のパターンでは開始距離10，25，50 mを選んだ．選定理由は再生一覧に表示する．')
    r.paragraph('図の読み方','subheading')
    r.paragraph('左は①しきい値，右は②確率モデルとする．速度比較は開始距離をそろえた単一系列，追従の距離比較は速度5系列である．細線は計算点間の補助線，×は全試行未接続，灰色の輪は一部未接続を表す．図中タイトルと拡大図は設けない．表の境界は区分の異なる隣接計算点の区間である．')
    figs=json.loads((root/'figures/index.json').read_text())
    for number,fig in enumerate(figs,1):
        fixed=fig.get('x_axis')=='speed_mps'
        stage={'baseline_speed':'基礎比較：','tracking_distance':'追加の距離比較：','dual60_comparison':'双方向60 GHzの比較：'}.get(fig.get('study_section'),'')
        r.page();r.paragraph(stage+fig['title'],'heading');r.picture(root/fig.get('report_file',fig['file']))
        start=fig.get('fixed_start_path_m',baseline)
        axis_title=f'飛行速度（開始距離{start:g} m）' if fixed else '開始距離'
        caption=r.paragraph(f"図{number}　{fig['title']}：{axis_title}と初回リンク確立までのTimeout（左：①，右：②）",'caption')
        keep=caption.find('w:pPr/w:keepNext',NS)
        if keep is not None:keep.getparent().remove(keep)
        if fixed:
            r.paragraph(f'表{number+2}　開始{start:g} mでの速度の区分境界 [m/s]','caption')
            table=[['開始距離 [m]','① しきい値モデル','② 確率モデル']];cells=[f'{start:g}'];findings=[]
            for model,label in [('threshold','①'),('statistical','②')]:
                same=lambda d:(d['profile'],d['scenario'],d['antenna_mode'],d['model'])==(
                    fig['profile'],fig['scenario'],fig['antenna_mode'],model)
                rs=[d for d in speed_summary if same(d)]
                changes=[d for d in speed_limits if same(d)]
                all_failed=all(float(d['connected_fraction'])==0 for d in rs)
                zero=[float(d['speed_mps']) for d in rs if float(d['mean_timeouts'])<=.01 and float(d['zero_timeout_connection_fraction'])>=.99]
                cells.append('全確認速度で未接続' if all_failed else
                             '全確認速度で0回付近' if len(zero)==len(rs) else interval_text(changes))
                if all_failed:result='全確認速度で未接続であった'
                elif zero:result=f'0回付近の確認速度は{min(zero):g}〜{max(zero):g} m/sであった'
                else:result='確認した速度には0回付近の計算点がなかった'
                findings.append(label+'は'+result+'．')
            table.append(cells);r.table(table,[1300,3671,3671])
            r.paragraph(f'各機の開始距離を{start:g} mに固定し，速度1〜5 m/sを比較した．'+''.join(findings)+
                        ('全範囲を0.1 m/s刻みで計算した．' if uniform30 else '区分が変わる領域は0.1 m/s刻みで計算した．')+'細線は未計算速度での挙動を保証するものではなく，境界は表の区間として扱う．')
            continue
        r.paragraph(f'表{number+2}　開始距離の区分境界 [m]','caption')
        table=[['速度 [m/s]','① しきい値モデル','② 確率モデル']]
        for speed in range(1,6):
            cells=[str(speed)]
            for model in ['threshold','statistical']:
                same=lambda d:(d['profile'],d['scenario'],d['antenna_mode'],d['model'],float(d['speed_mps']))==(
                    fig['profile'],fig['scenario'],fig['antenna_mode'],model,speed)
                rs=[d for d in limits if same(d)];item=next(d for d in overview if same(d))
                cells.append('全確認位置で未接続' if item['all_unconnected']=='True' else
                             '全確認位置で0回付近' if item['all_near_zero']=='True' else interval_text(rs))
            table.append(cells)
        r.table(table,[1000,3821,3821])
        findings=[]
        for model,label in [('threshold','①'),('statistical','②')]:
            observations=[]
            for speed in [1,5]:
                rs=[d for d in summary if (d['profile'],d['scenario'],d['antenna_mode'],d['model'],float(d['speed_mps']))==
                    (fig['profile'],fig['scenario'],fig['antenna_mode'],model,speed)]
                zero=[float(d['position_m']) for d in rs if float(d['mean_timeouts'])<=.01 and float(d['zero_timeout_connection_fraction'])>=.99]
                if all(float(d['connected_fraction'])==0 for d in rs):value='全確認位置で未接続'
                elif not zero:value='0回付近の計算点なし'
                else:value=f'0回付近の最大確認開始距離{max(zero):g} m'
                observations.append(f'{speed} m/sでは{value}')
            findings.append(label+'は'+'，'.join(observations)+'であった．')
        r.paragraph(''.join(findings)+'最大確認開始距離まで連続して0回付近になるとは限らないため，途中の未接続区間は表と図を併せて確認する．')
    r.page()
    r.paragraph('考察','heading')
    r.paragraph('開始位置と速度の影響は，受信可能領域へ入る時刻だけでなく，BeaconからRequest・Response・ACKへ進む時点と500 msのTimeout待ちとの関係によって変化する．同じ経路でも開始位置が変わればプロトコルの位相が変わるため，Timeout回数が距離や速度に対して単調になるとは限らない．')
    r.paragraph('固定アンテナでは相手方向が指向軸から外れると利得が低下する．追従では指向軸を相手に向け続けるが，自由空間損失は残る．追加の双方向60 GHz条件では周波数・送信電力・回路を対称とした一方で，接続手順の役割はAとBで異なるため，無線条件の対称性だけからTimeoutなしの接続は保証されない．')
    dual=next((v for v in replays['results'] if v.get('profile')=='dual60' and v['model']=='statistical'),None)
    if dual and 'power_summary' in dual:
        power=dual['power_summary']
        r.paragraph(f"追加の双方向60 GHz・十字固定の代表記録（各機{dual['speed_mps']:g} m/s，開始{dual['start_path_m']:g} m）では，0.01 s間隔の記録点における最大受信電力はP方向{power['sampled_max_p_dbm']:.2f} dBm，Q方向{power['sampled_max_q_dbm']:.2f} dBmであり，全記録点でP/Q電力が一致した．"+
                    ('この条件では利得が低い角度を通るため受信可能域へ到達せず，Timeoutが0回でもリンク確立に成功していない．' if dual['first_connected_s']<0 and max(power['sampled_max_p_dbm'],power['sampled_max_q_dbm'])<-68.5 else '受信電力の対称性とプロトコルの接続成否を分けて評価した．'))
    r.paragraph('双方向60 GHz・十字固定が実機で接続するとの観察に対し，本条件では接続を再現できていない．内部増幅の欠落だけを原因と断定せず，実機の距離・アンテナ姿勢，E/H断面からの立体利得の近似，−7 dBmの出力基準位置，−68.5 dBmの感度基準位置を照合する必要がある．アンテナ端子で規定された送信出力や受信機入力感度には内部回路の効果が含まれ得るため，内部利得の単純加算は二重計上を招く．')
    r.paragraph('②の0回付近は測定曲線とパケット長補正・独立抽選の仮定に依存する．特に短い制御フレームは100 kB測定と誤り率が大きく異なり得る．①②の比較から方向性を確認し，実機の絶対RSSI基準と方向別・長さ別の誤り率が得られた段階で受信曲線を再調整するのが適切である．')
    r.paragraph('成果物と再現方法','heading')
    r.paragraph(f'figures/に{len(figs)}図と一括PDF，analysis/に条件別平均・接続率・全試行・境界表を保存した．speed_refinement/には細分化速度と反復安定性の記録を残した．monitor.shで代表試行を選択して再生でき，俯瞰図では機体色に対応する矢印と方位角・仰角を表示する．Timeout発生点の強調と時刻選択による移動も利用できる．')
    r.paragraph('設定と入力CSVはconfigs/およびinputs/に保存し，metadata/input_sha256.jsonで入力を固定する．このWord文書は指定されたver2.docxのヘッダー，本文10.5 pt，表の罫線・セル余白，中央数式と右端式番号を基に作成した．Wordの日本語・英数字はMS明朝・Times New Romanを指定し，この環境のPDFと図は未導入フォントをNoto Serif CJK JP・Liberation Serifで代替した．')
    path=root/'report/初回リンク確立_生CSVアンテナ利得_評価レポート.docx';r.save(path);render(path)
    (path.parent/'format_verification.json').write_text(json.dumps(dict(template=str(a.template),
        body_font_pt=10.5,table_font_pt=9,native_omml_equations=5,header_preserved=True,
        original_comments_removed=True,figure_count=len(figs),source_verification=verification,
        replay_cases=replays['verified_cases']),ensure_ascii=False,indent=2))
    print('Word/PDFレポート作成完了:',path,flush=True)


if __name__=='__main__':main()
