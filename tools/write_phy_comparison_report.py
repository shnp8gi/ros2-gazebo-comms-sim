#!/usr/bin/env python3
"""同条件のモデル比較結果から日本語PDFと自己完結HTMLを作成する。"""
import argparse
import base64
import csv
from html import escape
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from fontTools.ttLib import TTFont as FontFile
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, Image, PageBreak
import yaml

MODELS=('empirical','theoretical')
LABELS={'empirical':'既存：実測PERモデル','theoretical':'新規：TJX理論モデル'}
COLORS={'empirical':'#3772A4','theoretical':'#DC7D35'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    args=parser.parse_args();root=args.directory.resolve()
    summary=json.loads((root/'aggregate.json').read_text());stats=summary['statistics'];n=summary['trials_per_model']
    verify=json.loads((root/'internal/verification.json').read_text())
    assert verify['status']=='complete' and verify['identical_RF_and_geometry_all_trials']
    config=yaml.safe_load((root/'internal/theoretical.yaml').read_text())
    assert config['scenario']=='cross' and config['antenna_mode']=='fixed'
    assert config['duration_s']==90 and config['encounter_time_s']==30
    assert config['drone_a']['speed_kmh']==config['drone_b']['speed_kmh']==3.6
    assert config['drone_a']['altitude_m']==20 and config['drone_b']['altitude_m']==30
    assert config['hrcp']['data_bytes']==2048 and config['protocol']['data_from_prc']
    report=root/'report';report.mkdir(exist_ok=False);figures=report/'figures';figures.mkdir()
    font='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
    font_manager.fontManager.addfont(font)
    plt.rcParams.update({'font.family':font_manager.FontProperties(fname=font).get_name(),'font.size':10,
        'axes.spines.top':False,'axes.spines.right':False,'axes.unicode_minus':False,
        'axes.labelcolor':'#24364A','text.color':'#24364A','xtick.color':'#536578','ytick.color':'#536578',
        'axes.grid':True,'grid.alpha':.18,'figure.facecolor':'white','savefig.facecolor':'white'})
    series={model:[] for model in MODELS};trials={model:[] for model in MODELS}
    for model in MODELS:
        for index in range(n):
            path=root/model/f'trial_{index:02d}'
            series[model].append(np.genfromtxt(path/'timeseries.csv',delimiter=',',names=True))
            trials[model].append(yaml.safe_load((path/'summary.yaml').read_text()))
    reference=series['theoretical'][0];t=reference['time_s']
    cumulative={m:np.array([s['total_data_MiB'] for s in series[m]]) for m in MODELS}
    mean=lambda model,key:stats[model][key]['mean']
    volume_delta=mean('theoretical','total_data_MiB')-mean('empirical','total_data_MiB')
    volume_percent=volume_delta/mean('empirical','total_data_MiB')*100
    active_delta=mean('theoretical','active_transfer_bins_s')-mean('empirical','active_transfer_bins_s')
    peak_idx=np.argmax(reference['rssi_ba_dbm']);peak=float(reference['rssi_ba_dbm'][peak_idx]);peak_time=float(t[peak_idx])
    face=config['encounter_time_s']-10/np.sqrt(2)

    def save(fig,name):
        fig.savefig(figures/(name+'.png'),dpi=180,bbox_inches='tight')
        fig.savefig(figures/(name+'.svg'),bbox_inches='tight')
        plt.close(fig)

    fig,axes=plt.subplots(1,3,figsize=(10,2.65),layout='constrained')
    for ax,key,title,unit in zip(axes,['total_data_MiB','active_transfer_bins_s','connected_time_s'],
                               ['総通信量','転送時間','接続時間'],['MiB','s','s']):
        values=[mean(m,key) for m in MODELS];err=[stats[m][key]['stddev'] for m in MODELS]
        ax.bar([0,1],values,yerr=err,capsize=4,color=[COLORS[m] for m in MODELS],width=.52)
        ax.set_xticks([0,1],['実測PER','TJX理論']);ax.set_title(title);ax.set_ylabel(unit);ax.set_ylim(0,max(values)*1.25)
        for i,v in enumerate(values):ax.text(i,v+max(values)*.04,f'{v:.2f}',ha='center',fontweight='bold')
    save(fig,'means')

    fig=plt.figure(figsize=(10,3.4),layout='constrained');ax=fig.add_subplot(121);az=fig.add_subplot(122)
    times=np.linspace(0,90,300);delta=times-30
    ax.plot(delta,np.zeros_like(delta),color='#3772A4',label='A：高度20 m')
    ax.plot(np.zeros_like(delta),delta,color='#DC7D35',label='B：高度30 m')
    a=np.array([-10/np.sqrt(2),0]);b=np.array([0,-10/np.sqrt(2)])
    ax.plot(*a,'o',color='#3772A4');ax.plot(*b,'o',color='#DC7D35');ax.plot([a[0],b[0]],[a[1],b[1]],'k--',lw=1)
    ax.annotate('A 正対位置',a,xytext=(-27,8),arrowprops={'arrowstyle':'-'})
    ax.annotate('B 正対位置',b,xytext=(7,-23),arrowprops={'arrowstyle':'-'})
    ax.plot(0,0,'+',color='black');ax.set(xlabel='x [m]',ylabel='y [m]',title='上面図：水平経路は直交',xlim=(-32,32),ylim=(-32,32));ax.set_aspect('equal');ax.legend(loc='upper left',fontsize=8)
    az.plot([0,10],[20,30],'k--',label='対向方向');az.scatter([0,10],[20,30],c=['#3772A4','#DC7D35'],s=55)
    az.annotate('A', (0,20),xytext=(-.7,18.8));az.annotate('B',(10,30),xytext=(10.5,30.1))
    az.text(4.8,24.2,'14.14 m',rotation=45,ha='center');az.set(xlabel='水平距離 [m]',ylabel='高度 [m]',title='正対時：水平差10 m・高度差10 m',xlim=(-3,13),ylim=(17,33));az.set_aspect('equal')
    save(fig,'geometry')

    fig,axes=plt.subplots(2,1,figsize=(10,6.2),sharex=True,layout='constrained')
    rate_rows=[]
    for m in MODELS:
        a=cumulative[m];mu=a.mean(axis=0)
        axes[0].plot(t,mu,color=COLORS[m],label=LABELS[m],lw=2)
        axes[0].fill_between(t,a.min(axis=0),a.max(axis=0),color=COLORS[m],alpha=.18)
        step=10;sampled=a[:,::step];rt=t[::step];rate=np.diff(sampled,axis=1)*1048576*8/1e6/np.diff(rt)
        axes[1].plot((rt[1:]+rt[:-1])/2,rate.mean(axis=0),color=COLORS[m],lw=1.7,label=LABELS[m])
        for i in range(len(rt)-1):rate_rows.append({'model':m,'start_s':rt[i],'end_s':rt[i+1],'mean_goodput_Mbps':rate[:,i].mean(),'min_goodput_Mbps':rate[:,i].min(),'max_goodput_Mbps':rate[:,i].max()})
    for ax in axes:ax.axvline(face,color='#536578',ls='--',lw=1);ax.set_xlim(16,32);ax.set_ylim(bottom=0);ax.legend(loc='upper left',fontsize=9)
    axes[0].set(ylabel='累積通信量 [MiB]',title=f'累積通信量：線は{n}試行平均、帯は最小～最大')
    axes[1].set(xlabel='開始からの時刻 [s]',ylabel='実効スループット [Mbps]',title='受領データ量から求めた0.1秒区間平均（破線：正対時刻）')
    save(fig,'transfer')
    with (report/'throughput_0p1s.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(rate_rows[0]));w.writeheader();w.writerows(rate_rows)

    ep=np.loadtxt(root/'internal/inputs/e_plane.csv',delimiter=',');hp=np.loadtxt(root/'internal/inputs/h_plane.csv',delimiter=',')
    gpeak=max(ep[:,1].max(),hp[:,1].max());e0=np.interp(0,ep[:,0],ep[:,1]);h0=np.interp(0,hp[:,0],hp[:,1]);g0=e0+h0-gpeak
    fig,axes=plt.subplots(1,2,figsize=(10,3.5),layout='constrained')
    axes[0].plot(ep[:,0],ep[:,1],label='E面',color='#3772A4');axes[0].plot(hp[:,0],hp[:,1],label='H面',color='#DC7D35')
    axes[0].set(xlabel='各断面の角度 [deg]',ylabel='利得 [dBi]',title='入力アンテナパターン',xlim=(-90,90));axes[0].legend()
    axes[1].plot(t,reference['gain_a_db'],label='A',color='#3772A4');axes[1].plot(t,reference['gain_b_db'],label='B',color='#DC7D35',ls='--')
    axes[1].axvline(face,color='#536578',ls=':',label='正対時刻');axes[1].set(xlabel='開始からの時刻 [s]',ylabel='合成アンテナ利得 [dBi]',title='交錯前後の合成利得',xlim=(16,32),ylim=(-60,25));axes[1].legend()
    save(fig,'antenna')
    fig,ax=plt.subplots(figsize=(10,2.6),layout='constrained')
    ax.plot(t,reference['rssi_ba_dbm'],color='#2B6777',label='共通の受信電力（両方向同値）')
    ax.axvline(face,color='#536578',ls='--',label='正対');ax.plot(peak_time,peak,'o',color='#DC7D35')
    ax.annotate(f'最大 {peak:.2f} dBm\n{peak_time:.2f} s',(peak_time,peak),xytext=(25,-65),arrowprops={'arrowstyle':'-'},fontsize=9)
    ax.set(xlabel='開始からの時刻 [s]',ylabel='受信電力 [dBm]',xlim=(16,32),ylim=(-100,-50));ax.legend(loc='lower left',fontsize=9)
    save(fig,'power')

    fig,axes=plt.subplots(1,3,figsize=(10,3),layout='constrained')
    for ax,key,title in zip(axes,['total_data_MiB','active_transfer_bins_s','connected_time_s'],['総通信量 [MiB]','転送時間 [s]','接続時間 [s]']):
        # 異なる平均値を比較できるよう、各平均からの偏差を表示する。
        for pos,m in enumerate(MODELS):
            values=np.array([r[key] for r in trials[m]])-mean(m,key)
            ax.scatter(pos+np.linspace(-.13,.13,n),values,color=COLORS[m],s=17,alpha=.75)
        ax.axhline(0,color='#536578',lw=.8);ax.set_xticks([0,1],['実測PER','TJX理論']);ax.set_title(title);ax.set_ylabel('各モデル平均からの差')
    save(fig,'dispersion')

    # PDFには日本語TrueTypeとLatinフォントを明示して埋め込む。
    jp='/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf';latin='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    pdfmetrics.registerFont(TTFont('JP',jp));pdfmetrics.registerFont(TTFont('Latin',latin))
    jpchars=set(FontFile(jp).getBestCmap())
    def markup(text):
        chunks=[];current='';previous=None
        for char in str(text):
            font='JP' if ord(char)>127 and ord(char) in jpchars else 'Latin'
            if previous!=font and current:chunks.append(f'<font name="{previous}">{escape(current)}</font>');current=''
            previous=font;current+=char
        if current:chunks.append(f'<font name="{previous}">{escape(current)}</font>')
        return ''.join(chunks)
    styles={name:ParagraphStyle(name,fontName='JP',fontSize=size,leading=leading,textColor=colors.HexColor('#24364A'),
                               spaceAfter=after,wordWrap='CJK',alignment=TA_LEFT)
            for name,size,leading,after in [('title',23,31,14),('heading',16,23,10),('body',9.5,15,8),('small',8,12,5),('cell',8.5,13,0)]}
    flow=[];html=[]
    def para(text,style='body'):
        flow.append(Paragraph(markup(text),styles[style]));html.append(f'<p class="{style}">{escape(str(text))}</p>')
    def table(rows,widths):
        cells=[[Paragraph(markup(cell),styles['cell']) for cell in row] for row in rows]
        tab=Table(cells,colWidths=widths,repeatRows=1,hAlign='LEFT')
        tab.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor('#E6EEF5')),('VALIGN',(0,0),(-1,-1),'TOP'),
            ('BOTTOMPADDING',(0,0),(-1,-1),6),('TOPPADDING',(0,0),(-1,-1),6),('LINEBELOW',(0,0),(-1,0),.8,colors.HexColor('#91A7BC')),
            ('LINEBELOW',(0,1),(-1,-1),.3,colors.HexColor('#DCE3EB'))]))
        flow.extend([tab,Spacer(1,10)])
        html.append('<table>'+''.join('<tr>'+''.join(f'<{"th" if i==0 else "td"}>{escape(str(c))}</{"th" if i==0 else "td"}>' for c in row)+'</tr>' for i,row in enumerate(rows))+'</table>')
    def picture(name,width=510):
        from PIL import Image as PILImage
        path=figures/(name+'.png');w,h=PILImage.open(path).size
        flow.extend([Image(str(path),width=width,height=width*h/w),Spacer(1,6)])
        data=base64.b64encode(path.read_bytes()).decode()
        html.append(f'<img alt="{name}" src="data:image/png;base64,{data}">')
    def newpage(title):
        if flow:flow.append(PageBreak());html.append('</section>')
        html.append('<section>');para(title,'heading')
    def fmt(m,key,places=3):return f"{mean(m,key):.{places}f}"
    exported=[]
    for metric in stats['empirical']:
        for m in MODELS:exported.append({'model':m,'metric':metric,**stats[m][metric]})
    with (report/'comparison_statistics.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(exported[0]));w.writeheader();w.writerows(exported)

    html.append('<section>')
    para('TJX理論モデルと実測PERモデルの比較','title')
    para(f'双方向60 GHz・十字交錯・各機1 m/s・固定アンテナ／各モデル{n}試行','small')
    para(f'同じ飛行・無線・アンテナ条件で比較した結果、TJX理論モデルの総通信量は平均{mean("theoretical","total_data_MiB"):.2f} MiB、実測PERモデルは平均{mean("empirical","total_data_MiB"):.2f} MiBとなった。予測値の差は{volume_delta:.2f} MiB（{volume_percent:+.1f}%）。')
    picture('means')
    para('棒は平均、エラーバーは標準偏差。エラーバーは平均値に比べ小さいため、ばらつきの詳細は第4節に示す。','small')
    rows=[[f'評価項目（{n}試行平均）','実測PERモデル','TJX理論モデル']]
    for key,label,unit in [('total_data_MiB','総通信量','MiB'),('active_transfer_bins_s','転送時間','s'),('connected_time_s','接続時間','s'),('mean_goodput_Mbps','90秒間の平均スループット','Mbps'),('timeouts','接続確立待ちタイムアウト','回')]:
        rows.append([label,fmt('empirical',key)+' '+unit,fmt('theoretical',key)+' '+unit])
    table(rows,[220,145,145])
    para(f'転送時間の平均差は{active_delta:+.2f}秒。ここで示す差は、同じ飛行・無線条件に対する両モデルの予測差である。')
    para('受信成否と伝送速度の扱いを各モデルに設定した、モデル全体の比較である。以下では共通条件、転送の推移、アンテナ利得、試行間のばらつきを示す。','small')

    newpage('1　共通のシミュレーション条件')
    table([['項目','両モデル共通の条件'],['飛行経路・速度','直交する2経路をA・Bとも1 m/sで飛行'],['高度','A：20 m、B：30 m（高度差10 m）'],
        ['開始位置・評価期間','各機は交錯点の30 m手前から開始。30秒後に交錯点通過、90秒間を評価。'],
        ['アンテナ姿勢','進行方向座標系でA：roll 0° / pitch −45° / yaw −45°、B：roll 0° / pitch +45° / yaw +45°。固定。'],
        ['正対する位置',f'両機とも交錯点の7.071 m手前。開始{face:.3f}秒、機体間距離14.142 m。'],
        ['周波数・送信電力','両方向60 GHz、送信機出力−7 dBm'],['RF回路','送信損失3 dB、受信LNA利得0 dB、変換利得+3 dB、受信損失3 dB'],
        ['伝搬・雑音','自由空間損失を基準とする指数2の伝搬。300 K、雑音帯域2.1648 GHz、NF 10 dB。'],
        ['送信データ','B→Aに2048 byteの有効データを送信。逆方向にはACK等を返信。'],
        ['接続・制御','制御MCS 3。接続確立待ち500 ms、接続後ATP 10秒。'],['試行・観測',f'各モデル{n}試行。時間指標の観測間隔10 ms。']], [105,405])
    picture('geometry',490)
    para('アンテナは追従させず、両機の相対位置と固定姿勢から時々刻々の指向方向と利得を求める。共通の位置・受信電力時系列が一致する条件で比較した。','small')

    newpage('2　通信量とスループットの時間変化')
    picture('transfer')
    rows=[['時間指標（平均）','実測PERモデル','TJX理論モデル']]
    for key,label in [('first_transfer_bin_start_s','最初の転送区間の開始'),('last_transfer_bin_end_s','最後の転送区間の終了'),('active_transfer_bins_s','転送時間（増加区間の合計）'),('transfer_bin_span_s','転送開始から終了までの幅')]:
        rows.append([label,fmt('empirical',key)+' s',fmt('theoretical',key)+' s'])
    table(rows,[230,140,140])
    para('通信量はAが受領した重複のないデータ本文をMiBで表す。双方向の合算ではない。スループットは0.1秒区間の受領データ量をbitに換算し、その区間長で割った値。転送時間は受領量が増加した10 ms区間の合計で、接続状態の継続時間とは区別する。','small')

    newpage('3　アンテナ利得と受信電力の計算')
    para('相手機への単位方向ベクトルをアンテナ座標系へ変換し、仰角をE面、方位角をH面のパターンに対応させる。回転行列はR = Rz(yaw) Ry(pitch) Rx(roll)、方向変換はu_ant = Rᵀ u_world。')
    para('各断面の利得を角度に対して線形補間し、共通のピーク基準G_peakを用いて合成する。今回の設定では減衰の打切りを行わないため、次式となる。')
    para('G_total = G_E(仰角) + G_H(方位角) − G_peak','body')
    table([['項目','値・処理'],['入力範囲・補間','E面・H面とも−90°～+90°、各1801点。隣接点間を線形補間。'],
        ['ピーク基準',f'G_peak = max(E面ピーク, H面ピーク) = {gpeak:.6f} dBi'],
        ['正面方向の合成利得',f'{e0:.6f} + {h0:.6f} − {gpeak:.6f} = {g0:.6f} dBi'],
        ['範囲外・基準調整','範囲外の断面値は−1000 dB。減衰打切りなし、正面利得の上書き・再正規化なし。']], [120,390])
    picture('antenna')
    para('送受信それぞれの合成利得をRF収支へ加算する。両方向のRF設定が対称なので、同時刻の受信電力は一致する。正対時刻と最大受信電力の時刻には、機体間距離の変化によって差が生じる。','small')
    picture('power')

    newpage(f'4　{n}試行のばらつきとタイムアウト')
    rows=[['指標','実測PER：平均 ± 標準偏差','TJX理論：平均 ± 標準偏差','実測PER：分散','TJX理論：分散']]
    for key,label in [('total_data_MiB','総通信量 [MiB]'),('active_transfer_bins_s','転送時間 [s]'),('connected_time_s','接続時間 [s]')]:
        vals=[stats[m][key] for m in MODELS]
        rows.append([label]+[f"{v['mean']:.3f} ± {v['stddev']:.4g}" for v in vals]+[f"{v['variance']:.4g}" for v in vals])
    table(rows,[90,125,125,85,85])
    para(f'表の分散は{n}試行の平均からの二乗偏差を{n}で割った値（母分散）。標準偏差はその平方根。分散の単位は通信量がMiB²、時間がs²。下図は各モデルの平均からの偏差で、各点が1試行を表す。','small')
    picture('dispersion')
    rows=[['回数指標','実測PER：平均 / 分散','TJX理論：平均 / 分散']]
    for key,label in [('timeouts','接続確立待ちタイムアウト'),('connections','接続成立'),('disconnections','接続後のATP切断')]:
        rows.append([label]+[f"{mean(m,key):.3f} / {stats[m][key]['variance']:.6f}" for m in MODELS])
    table(rows,[230,140,140])
    para('接続確立待ちタイムアウトは、接続手続きを開始した端点が待機期限を超えた回数。Beacon探索を続ける時間や単体のパケット失敗・再送は含めない。接続後のATP満了による切断は別に数える。','small')
    para(f'総通信量の範囲は、実測PERモデルが{stats["empirical"]["total_data_MiB"]["min"]:.3f}～{stats["empirical"]["total_data_MiB"]["max"]:.3f} MiB、TJX理論モデルが{stats["theoretical"]["total_data_MiB"]["min"]:.3f}～{stats["theoretical"]["total_data_MiB"]["max"]:.3f} MiB。今回の試行では、平均値のモデル間差は試行間のばらつきより大きい。')
    html.append('</section>')

    def footer(canvas,doc):
        canvas.saveState();canvas.setStrokeColor(colors.HexColor('#DCE3EB'));canvas.line(42,35,553,35)
        p=Paragraph(markup(f'TJX理論モデル／実測PERモデル　比較レポート　|　{doc.page}'),styles['small'])
        p.wrap(510,20);p.drawOn(canvas,42,18);canvas.restoreState()
    doc=SimpleDocTemplate(str(report/'model_comparison_ja.pdf'),pagesize=A4,leftMargin=42,rightMargin=42,topMargin=40,bottomMargin=48,
                         title='TJX理論モデルと実測PERモデルの同条件比較',author='通信シミュレーション比較')
    doc.build(flow,onFirstPage=footer,onLaterPages=footer)
    css='''body{font-family:"Noto Sans CJK JP","Yu Gothic",sans-serif;background:#eef2f6;color:#24364a;margin:0;line-height:1.8}main{max-width:1000px;margin:auto}section{background:white;margin:28px 0;padding:42px;border-radius:12px;box-shadow:0 4px 24px #20344a12}p.title{font-size:30px;font-weight:700;line-height:1.4}p.heading{font-size:24px;font-weight:700}p.small{font-size:13px;color:#536578}table{width:100%;border-collapse:collapse;font-size:14px;margin:20px 0}th{background:#e6eef5;text-align:left}td,th{padding:10px;border-bottom:1px solid #dce3eb;vertical-align:top}img{width:100%;height:auto}nav{padding:20px}a{color:#3772a4}@media(max-width:700px){section{padding:18px}table{font-size:12px}}@media print{body{background:white}section{page-break-after:always;box-shadow:none;padding:0;margin:0}}'''
    (report/'model_comparison_ja.html').write_text('<!doctype html><html lang="ja"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>TJX理論モデルと実測PERモデルの比較</title><style>'+css+'</style><body><main>'+''.join(html)+'</main></body></html>')
    (root/'internal/report_facts.json').write_text(json.dumps({'volume_delta_MiB':volume_delta,'volume_delta_percent':volume_percent,
        'active_time_delta_s':active_delta,'peak_received_dbm':peak,'peak_time_s':peak_time,'face_time_s':face,'boresight_gain_dbi':g0,
        'report_scope':'essential_conditions_model_identity_statistics_and_antenna_details; implementation_and_calibration_logged_separately'},ensure_ascii=False,indent=2))
    print(f'レポート作成完了: {report}')


if __name__=='__main__':main()
