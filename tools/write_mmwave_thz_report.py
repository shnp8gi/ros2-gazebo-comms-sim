#!/usr/bin/env python3
"""4パターンの速度・開始距離・境界比較をPDF/Word/HTMLにまとめる。"""
import argparse
import csv
import json
from pathlib import Path
import shutil
import statistics
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from local_comparison_report import LocalReport

PATTERNS=('parallel_fixed','parallel_tracking','cross_fixed','cross_tracking')
NAMES={'parallel_fixed':'並行・固定','parallel_tracking':'並行・追従','cross_fixed':'十字・固定','cross_tracking':'十字・追従'}
MODELS=('empirical','theoretical')
LABELS={'empirical':'実測PERモデル','theoretical':'TJX理論モデル'}
COLORS={'empirical':'#3772A4','theoretical':'#DC7D35'}


def load(path):
    with path.open() as f:rows=list(csv.DictReader(f))
    for row in rows:
        for key in ('speed_mps','start_path_m','connected_fraction','zero_timeout_connection_fraction','mean_timeouts','variance_timeouts','stddev_timeouts'):
            row[key]=float(row[key])
        for key in ('trials','connected_trials','zero_timeout_connected_trials','min_timeouts','max_timeouts'):row[key]=int(row[key])
    return rows


def segments(values,step=.1):
    if not values:return '該当なし'
    values=sorted(values);groups=[];start=previous=values[0]
    for value in values[1:]:
        if value-previous>step+1e-7:groups.append((start,previous));start=value
        previous=value
    groups.append((start,previous))
    return '、'.join(f'{a:g}' if a==b else f'{a:g}～{b:g}' for a,b in groups)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path);p.add_argument('--output-name',default='report');a=p.parse_args()
    root=a.directory.resolve();verified=json.loads((root/'internal/verification.json').read_text());assert verified['status']=='complete'
    n=verified['trials_per_condition'];speed=load(root/'speed_summary.csv');distance=load(root/'distance_summary.csv')
    with (root/'speed_trials.csv').open() as f:speed_trials=list(csv.DictReader(f))
    refinement=load(root/'refinement_summary.csv') if (root/'refinement_summary.csv').exists() else []
    spatial=distance+refinement
    output=root/a.output_name;output.mkdir(exist_ok=False);figures=output/'figures';figures.mkdir()
    font='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc';font_manager.fontManager.addfont(font)
    plt.rcParams.update({'font.family':font_manager.FontProperties(fname=font).get_name(),'font.size':10,'axes.unicode_minus':False,
        'axes.spines.top':False,'axes.spines.right':False,'axes.grid':True,'grid.alpha':.18,'figure.facecolor':'white',
        'axes.labelcolor':'#24364A','text.color':'#24364A','savefig.facecolor':'white'})
    def save(fig,name):
        fig.savefig(figures/(name+'.png'),dpi=180,bbox_inches='tight');fig.savefig(figures/(name+'.svg'),bbox_inches='tight');plt.close(fig)
        return figures/(name+'.png')
    def select(rows,pattern,model,velocity=None):
        return sorted((r for r in rows if r['pattern']==pattern and r['model']==model and (velocity is None or r['speed_mps']==velocity)),
                      key=lambda r:(r['speed_mps'],r['start_path_m']))
    def value(pattern,model,velocity=1):return select(speed,pattern,model,velocity)[0]
    speed_images={}
    for pattern in PATTERNS:
        fig,axes=plt.subplots(2,2,figsize=(7.1,4.7),sharex='col',sharey='row',layout='constrained')
        ymax=max(r['mean_timeouts']+r['stddev_timeouts'] for r in speed if r['pattern']==pattern)
        for j,model in enumerate(MODELS):
            rows=select(speed,pattern,model);x=np.array([r['speed_mps'] for r in rows]);y=np.array([r['mean_timeouts'] for r in rows]);sd=np.array([r['stddev_timeouts'] for r in rows])
            connected=np.array([r['connected_fraction'] for r in rows]);zero=np.array([r['zero_timeout_connection_fraction'] for r in rows])
            axes[0,j].plot(x,np.where(connected>0,y,np.nan),color=COLORS[model],marker='o',ms=2.5,lw=1.4,label='平均Timeout')
            axes[0,j].fill_between(x,np.maximum(0,y-sd),y+sd,where=connected>0,color=COLORS[model],alpha=.18,label='±標準偏差')
            failed=connected==0
            if failed.any():axes[0,j].scatter(x[failed],y[failed],marker='x',color='#A43B3B',label='全試行未接続')
            axes[0,j].set(title=LABELS[model],ylim=(-.1,max(1,ymax*1.15)));axes[0,j].legend(fontsize=8)
            axes[1,j].plot(x,connected*100,color='#36836A',lw=1.6,label='接続率')
            axes[1,j].plot(x,zero*100,color=COLORS[model],ls='--',lw=1.6,label='Timeout 0回での接続率')
            axes[1,j].set(xlabel='各機の速度 [m/s]',ylim=(-5,105),xlim=(.95,5.05));axes[1,j].legend(fontsize=8,loc='lower right')
        axes[0,0].set_ylabel('初回接続までのTimeout [回]');axes[1,0].set_ylabel('試行の割合 [%]')
        speed_images[pattern]=save(fig,'speed_'+pattern)
    distance_images={};palette=['#3772A4','#DC7D35','#36836A','#9467BD','#B14A60']
    for pattern in ('parallel_tracking','cross_tracking'):
        fig,axes=plt.subplots(2,2,figsize=(7.1,4.7),sharex='col',sharey='row',layout='constrained')
        for j,model in enumerate(MODELS):
            for v,color in zip(range(1,6),palette):
                rows=select(spatial,pattern,model,v);rows.sort(key=lambda r:r['start_path_m'])
                x=np.array([r['start_path_m'] for r in rows]);y=np.array([r['mean_timeouts'] for r in rows]);connected=np.array([r['connected_fraction'] for r in rows])
                sd=np.array([r['stddev_timeouts'] for r in rows])
                axes[0,j].plot(x,np.where(connected>0,y,np.nan),color=color,lw=1.3,label=f'{v} m/s')
                axes[0,j].fill_between(x,np.maximum(0,y-sd),y+sd,where=connected>0,color=color,alpha=.12)
                if (connected==0).any():axes[0,j].scatter(x[connected==0],y[connected==0],color=color,marker='x',s=12)
                axes[1,j].plot(x,[r['zero_timeout_connection_fraction']*100 for r in rows],color=color,lw=1.2,label=f'{v} m/s')
            axes[0,j].set_title(LABELS[model]);axes[0,j].set_ylim(bottom=-.1);axes[0,j].legend(fontsize=8)
            axes[1,j].set(xlabel='開始距離（各機の航路上）[m]',ylim=(-5,105),xlim=(0,61));axes[1,j].legend(fontsize=8,loc='upper right')
        axes[0,0].set_ylabel('初回接続までのTimeout [回]');axes[1,0].set_ylabel('Timeout 0回での接続率 [%]')
        distance_images[pattern]=save(fig,'distance_'+pattern)
    # 1mから続く全試行0回接続の最初の境界を、未計算領域へ外挿せず記載。
    boundaries=[]
    for pattern in ('parallel_tracking','cross_tracking'):
        for v in range(1,6):
            for model in MODELS:
                rows=select(spatial,pattern,model,v);rows.sort(key=lambda r:r['start_path_m']);last=None;first_failure=None
                for r in rows:
                    if r['category']!='all_zero_connected':first_failure=r['start_path_m'];break
                    last=r['start_path_m']
                label='開始1 mから条件未達' if last is None else '60 mまで全確認点で成立' if first_failure is None else f'{last:.1f}～{first_failure:.1f} m'
                boundaries.append({'pattern':pattern,'model':model,'speed_mps':v,'last_all_zero_connected_m':last,
                    'first_not_all_zero_m':first_failure,'label':label})
    ep=np.loadtxt(root/'inputs/e_plane.csv',delimiter=',');hp=np.loadtxt(root/'inputs/h_plane.csv',delimiter=',')
    peak=max(ep[:,1].max(),hp[:,1].max());boresight=np.interp(0,ep[:,0],ep[:,1])+np.interp(0,hp[:,0],hp[:,1])-peak
    fig,axes=plt.subplots(1,2,figsize=(7.1,3.0),layout='constrained')
    axes[0].plot(ep[:,0],ep[:,1],label='E面');axes[0].plot(hp[:,0],hp[:,1],label='H面');axes[0].set(xlabel='断面角度 [deg]',ylabel='利得 [dBi]',xlim=(-90,90),title='入力パターン');axes[0].legend()
    for pattern,color in zip(PATTERNS,palette):
        curve=np.genfromtxt(root/'internal'/f'{pattern}_radio.csv',delimiter=',',names=True)
        axes[1].plot(curve['time_s'],curve['gain_a_dbi'],color=color,label=NAMES[pattern],ls='--' if pattern.endswith('tracking') else '-')
    axes[1].set(xlabel='時刻 [s]（1 m/s）',ylabel='A側合成利得 [dBi]',title='4パターンの合成利得',xlim=(0,45),ylim=(-80,25));axes[1].legend(fontsize=8)
    antenna_image=save(fig,'antenna')
    power_peaks=[]
    fig,axes=plt.subplots(2,2,figsize=(7.1,4.5),layout='constrained')
    for ax,pattern in zip(axes.flat,PATTERNS):
        curve=np.genfromtxt(root/'internal'/f'{pattern}_radio.csv',delimiter=',',names=True)
        peak_index=int(np.argmax(curve['p_dbm']))
        power_peaks.append([NAMES[pattern],f"{curve['p_dbm'][peak_index]:.3f}",f"{curve['q_dbm'][peak_index]:.3f}",f"{curve['time_s'][peak_index]:.2f}"])
        ax.plot(curve['time_s'],curve['p_dbm'],label='P：テラヘルツ',color='#DC7D35');ax.plot(curve['time_s'],curve['q_dbm'],label='Q：ミリ波',color='#3772A4')
        ax.set(title=NAMES[pattern],xlabel='時刻 [s]',ylabel='受信電力 [dBm]',xlim=(0,45),ylim=(-110,-15));ax.legend(fontsize=8)
    power_image=save(fig,'power')

    report=LocalReport('ミリ波・テラヘルツ通信：4パターンの初回リンク確立比較')
    report.page('ミリ波・テラヘルツ通信\n初回リンク確立までのTimeout比較'.replace('\n','　'))
    report.text(f'並行・十字 × 固定・追従／実測PERモデルとTJX理論モデル／各条件{n}試行')
    report.text('参考資料「初回リンク確立までのTimeout回数評価」の無線条件を用い、速度と開始距離が初回リンク確立に与える影響を比較した。十字固定のアンテナは、直近の検討に合わせて正対する±45°設定を採用した。')
    rows=[['1 m/s・開始30 m','実測PER：Timeout平均','TJX理論：Timeout平均','接続率（実測 / 理論）']]
    for pattern in PATTERNS:
        e,t=[value(pattern,m) for m in MODELS]
        rows.append([NAMES[pattern],f"{e['mean_timeouts']:.3f}回",f"{t['mean_timeouts']:.3f}回",f"{e['connected_fraction']*100:.1f}% / {t['connected_fraction']*100:.1f}%"])
    report.table(rows,[110,140,140,120])
    report.text('速度比較では4パターンすべての開始距離を30 mに統一し、1～5 m/sを0.1 m/s刻みで評価した。開始距離の比較は追従2パターンを対象とし、速度1・2・3・4・5 m/sで1～60 mを調べた。')
    report.text('このレポートで示す差は、同じ飛行・無線条件に対するモデルの予測差である。比較するのは、受信成否とPHY伝送の扱いをそれぞれ設定したモデル全体の結果である。')
    zero_rows=[['全試行Timeout 0回で接続した確認速度','実測PERモデル [m/s]','TJX理論モデル [m/s]']]
    for pattern in PATTERNS:zero_rows.append([NAMES[pattern]]+[segments([r['speed_mps'] for r in select(speed,pattern,m) if r['category']=='all_zero_connected']) for m in MODELS])
    report.table(zero_rows,[150,180,180])
    report.text('表の速度範囲は0.1 m/s刻みの計算点に基づく。Timeout回数だけでなく接続率と「Timeoutなしで接続した割合」を併記する。',small=True)
    for pattern in ('parallel_tracking','cross_tracking'):
        empirical_bounds=[next(r['label'] for r in boundaries if r['pattern']==pattern and r['model']=='empirical' and r['speed_mps']==v) for v in (1,5)]
        report.text(f'{NAMES[pattern]}の開始距離比較：実測PERモデルで全試行Timeout 0回接続を初めて満たさなくなる境界は、1 m/sで{empirical_bounds[0]}、5 m/sで{empirical_bounds[1]}。',small=True)
    if all(r['category']=='all_zero_connected' for r in spatial if r['model']=='theoretical'):
        report.text(f'TJX理論モデルは、追従2パターン・速度1～5 m/sの開始距離比較で、1～60 mの全確認点において全{n}試行がTimeout 0回で接続した。',small=True)

    report.page('1　共通条件と評価指標')
    report.table([['項目','条件'],['4パターン','並行・固定、並行・追従、十字・固定、十字・追従'],['飛行','高度A=20 m、B=30 m。並行は横間隔10 mの逆方向飛行。十字は直交経路。'],
        ['固定角（進行方向基準）','並行：A=(0°,−45°,+90°)、B=(0°,+45°,+90°)。十字：A=(0°,−45°,−45°)、B=(0°,+45°,+45°)。順序はroll/pitch/yaw。'],
        ['追従','相手機を3次元的に指向する。'],['速度比較','各機の開始距離30 m、速度1～5 m/s、0.1 m/s刻み。'],
        ['開始距離比較','追従2パターン、各機1～60 m、速度1・2・3・4・5 m/s。1 m刻みで探索し、分類が変わる区間を0.1 m刻みで追加評価。'],
        ['観測の終了','初回リンク確立時点。未接続の場合は最接近地点を60 m通過した位置まで。観測上限T=(開始距離+60)/速度。'],
        ['接続待ち・制御',f'Timeout 500 ms、制御MCS 3。各条件{n}試行。']], [105,405])
    report.table([['無線条件','P：B→A（テラヘルツ）','Q：A→B（ミリ波）'],['周波数','271.08 GHz','60.48 GHz'],['送信電力','−1 dBm','−8 dBm'],['送信損失','0 dB','3 dB'],['受信LNA利得','25 dB','0 dB'],['受信変換利得','+3 dB','0 dB'],['受信損失','3 dB','3 dB'],['雑音指数','10 dB','0 dB'],['温度・雑音帯域','300 K・2.1648 GHz','300 K・2.1648 GHz']], [150,180,180])
    report.text('Timeoutは初回接続までに端点の待機期限が満了した回数。A/B両端が満了した場合はそれぞれ数える。初回接続後のATP切断は対象外。未接続の試行は観測上限までのTimeout回数を記録し、接続率で区別する。',small=True)
    report.text(f'平均と分散は各条件{n}試行から算出する。分散は二乗偏差の和を{n}で割る母分散、単位は回²。接続率とTimeout 0回接続率の分母はいずれも全{n}試行。',small=True)
    report.text(f'分散0は、その条件の{n}試行でTimeout回数が一致したことを表す。確率的な受信判定でも、回数として集計すると同じ値になる場合がある。',small=True)

    for index,pattern in enumerate(PATTERNS,2):
        report.page(f'{index}　速度比較：{NAMES[pattern]}')
        report.text('開始距離は両機とも30 m。上段はTimeout平均と±標準偏差、下段は接続率とTimeout 0回での接続率。×は全試行未接続を表す。',small=True)
        report.image(speed_images[pattern])
        rows=[['速度 [m/s]','実測PER：平均 / 分散','TJX理論：平均 / 分散','接続率（実測 / 理論）']]
        for v in range(1,6):
            e,t=[value(pattern,m,v) for m in MODELS]
            rows.append([str(v)]+[f"{r['mean_timeouts']:.3f} / {r['variance_timeouts']:.4g}" for r in (e,t)]+[f"{e['connected_fraction']*100:.1f}% / {t['connected_fraction']*100:.1f}%"])
        report.table(rows,[75,155,155,125])
        for m in MODELS:
            rows=select(speed,pattern,m);zero=[r['speed_mps'] for r in rows if r['category']=='all_zero_connected'];none=[r['speed_mps'] for r in rows if r['category']=='none_connected']
            report.text(f"{LABELS[m]}：全{n}試行がTimeout 0回で接続した確認速度は「{segments(zero)}」。全試行未接続の確認速度は「{segments(none)}」。",small=True)

    for index,pattern in enumerate(('parallel_tracking','cross_tracking'),6):
        report.page(f'{index}　開始距離比較：{NAMES[pattern]}')
        report.text('開始距離は各機の航路に沿った最接近地点までの距離。上段はTimeout平均と±標準偏差、下段はTimeout 0回で接続した試行の割合。速度別に色分けする。',small=True)
        report.image(distance_images[pattern])
        rows=[['速度 [m/s]','実測PERモデルの最初の境界','TJX理論モデルの最初の境界']]
        for v in range(1,6):rows.append([str(v)]+[next(r['label'] for r in boundaries if r['pattern']==pattern and r['model']==m and r['speed_mps']==v) for m in MODELS])
        report.table(rows,[75,217,218])
        report.text(f'境界は、開始1 mから順に調べて「全{n}試行がTimeout 0回で接続」を初めて満たさなくなる隣接計算点の区間。これは確認した条件・試行の結果であり、60 mを超える距離へは外挿しない。',small=True)
        report.text('全開始距離の平均・分散は別添distance_summary.csv、境界付近の追加点はrefinement_summary.csvに記録した。',small=True)

    report.page('8　アンテナ利得と正対条件')
    report.text('相手機への方向をアンテナ座標系に変換し、仰角をE面、方位角をH面に対応させる。E/H各面のCSVを線形補間し、共通ピーク基準を差し引いて合成する。')
    report.text('G_total = G_E(仰角) + G_H(方位角) − G_peak')
    report.table([['項目','値・処理'],['角度範囲','E/H各面−90°～+90°、各1801点。隣接点間を線形補間。'],['ピークと正面利得',f'G_peak={peak:.6f} dBi、正面合成利得={boresight:.6f} dBi。'],['打切り・範囲外','減衰打切りなし、正面利得の上書きなし。範囲外の断面値は−1000 dB。']], [115,395])
    report.image(antenna_image)
    report.text('十字・固定ではroll=0°、pitch・yawは各機の進行方向座標系でそれぞれ±45°とする。両機が交錯点まで各7.071 mの位置に来ると、アンテナ軸が完全に向かい合う。この時の機体間距離は14.142 m。開始30 m・速度1 m/sでは開始から22.929 sに相当する。')
    report.text('並行・固定では最接近時に正対し、機体間距離は14.142 mとなる。追従では相手機への方向を追い続けるため、正面合成利得が維持される。')
    report.text('受信電力の最大時刻は、アンテナ利得と機体間距離の両方で決まる。したがって、固定アンテナの正対時刻と受信電力の最大時刻は必ずしも一致しない。',small=True)

    report.page('9　方向別の受信電力')
    report.text('開始30 m・速度1 m/sにおける受信電力の時間変化。初回接続の計算とは別に、同じ飛行軌道とアンテナ条件のRF値を示す。両モデルに共通の値である。')
    report.image(power_image)
    report.table([['パターン','P最大 [dBm]','Q最大 [dBm]','最大時刻 [s]']]+power_peaks,[150,125,125,110])
    report.text('表の最大値は0.01 s刻みで計算した0～90 sの観測点から求めた。グラフは最接近前後を読み取りやすいように0～45 sを表示している。',small=True)
    report.text('受信電力は送信電力−送信損失＋送受信利得−自由空間損失＋LNA利得＋変換利得−受信損失で算出する。左右のモデルには同じRF・アンテナ条件を与えている。グラフは1 m/s、開始30 mの場合。',small=True)

    report.page('10　結果の整理')
    for model in MODELS:
        group=[r for r in speed if r['model']==model]
        zero_count=sum(r['category']=='all_zero_connected' for r in group)
        report.text(f'{LABELS[model]}：速度比較の{len(group)}条件のうち、全{n}試行がTimeout 0回で接続したのは{zero_count}条件。')
    if all(r['variance_timeouts']==0 for r in speed):
        report.text(f'今回の速度比較は全{len(speed)}条件でTimeout回数の分散が0だった。各条件内の{n}試行の回数が一致した結果であり、接続時刻まで同一であることは意味しない。',small=True)
    report.text('固定アンテナでは、飛行に伴って相手機が指向方向を通過する時間が変わる。追従では指向方向を相手機に合わせ続けるため、通信を開始する位置と、両方向の接続手順が成立するまでの時間の関係を評価する。')
    report.text('P方向のBeacon、Q方向のRequestなど、接続には両方向の通信が必要である。受信電力の大小だけで初回接続の成否を決めず、各モデルが返す受信結果と接続手順を通してTimeoutを集計した。')
    timing=[['1 m/s・開始30 m','実測PER：初回Request / 接続 [s]','TJX理論：初回Request / 接続 [s]']]
    for pattern in PATTERNS:
        scenario,mode=pattern.split('_');row=[NAMES[pattern]]
        for model in MODELS:
            group=[r for r in speed_trials if r['scenario']==scenario and r['antenna_mode']==mode and r['model']==model and float(r['speed_mps'])==1]
            requested=[float(r['first_request_s']) for r in group if float(r['first_request_s'])>=0]
            connected=[float(r['first_connected_s']) for r in group if int(r['connected'])]
            row.append(('未送信' if not requested else f'{statistics.mean(requested):.6f}')+' / '+('未接続' if not connected else f'{statistics.mean(connected):.6f}'))
        timing.append(row)
    report.table(timing,[110,200,200])
    report.text('時刻は飛行開始を0 sとした平均値。Timeout 0回でも、開始直後に接続するとは限らない。接続の早さを比較する際は、Timeout回数と初回接続時刻の両方を見る。',small=True)
    report.text('十字・固定は、参照資料の横向き設定から正対する設定へ変更している。今回の結果と資料中の十字固定の結果を比較する際は、このアンテナ条件の違いを含む比較となる。')
    report.text('参考資料：「初回リンク確立までのTimeout回数評価.docx」。方向別周波数・送信電力・回路利得、開始距離と観測上限、速度比較と追従条件の距離比較を参照した。')
    report.text(f'速度・開始距離・境界追加を合わせて{verified["conditions"]//2}評価点 × 2モデル × 各{n}試行、合計{verified["trials"]:,}試行。各工程の全計算点について、平均・分散・接続率を別添CSVに保存した。')
    prefix=output/'mmwave_thz_initial_connection_comparison_ja';report.write(prefix)
    for source in ('speed_summary.csv','distance_summary.csv','refinement_summary.csv'):
        if (root/source).exists():shutil.copyfile(root/source,output/source)
    with (output/'boundary_summary.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(boundaries[0]));w.writeheader();w.writerows(boundaries)
    (root/'internal/report_facts.json').write_text(json.dumps({'boundaries':boundaries,'power_peaks':power_peaks,'total_trials':verified['trials'],'sections':11,
        'source_reference':str(root/'internal/reference.docx'),'cross_angles':'user_approved_facing_45','policy':'implementation_and_calibration_details_internal_only'},ensure_ascii=False,indent=2))
    print(f'PDF・Word・HTMLの作成完了: {output}',flush=True)


if __name__=='__main__':main()
