#!/usr/bin/env python3
"""非対称SNR掃引を、日本語PDF・Word・HTMLと編集用図表にまとめる。"""
import argparse
import csv
import json
import math
from pathlib import Path
from zipfile import ZipFile, ZIP_DEFLATED

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from local_comparison_report import LocalReport

PATTERNS = ('parallel_fixed','parallel_tracking','cross_fixed','cross_tracking')
NAMES = dict(zip(PATTERNS, ('並行・固定','並行・追従','十字・固定','十字・追従')))
COLORS = dict(zip(PATTERNS, ('#24669c','#38a18b','#cf713c','#9364ae')))
P_COLOR, Q_COLOR = '#c45d39', '#286ba3'


def load(path):
    with path.open() as stream:
        rows = list(csv.DictReader(stream))
    for row in rows:
        for key, value in list(row.items()):
            if key not in ('phase','pattern','case_id'):
                row[key] = float(value) if value else math.nan
    return rows


def groups(data, mask):
    indices = np.flatnonzero(mask)
    if not len(indices):
        return []
    chunks = np.split(indices, np.flatnonzero(np.diff(indices)>1)+1)
    return [(float(data['remaining_path_m'][g[0]]),float(data['remaining_path_m'][g[-1]])) for g in chunks]


def interval_text(intervals):
    return 'なし' if not intervals else ' / '.join(f'{a:+.2f} → {b:+.2f}' for a,b in intervals)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    root = args.directory.resolve()
    verified = json.loads((root/'internal/verification.json').read_text())
    assert verified['status']=='complete'
    speed, distance = load(root/'speed_summary.csv'), load(root/'distance_summary.csv')
    profile = {p:np.genfromtxt(root/'profiles'/f'{p}.csv', delimiter=',', names=True) for p in PATTERNS}
    out = root/'report'
    out.mkdir(exist_ok=False)
    figures = out/'figures'
    figures.mkdir()
    font = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
    font_manager.fontManager.addfont(font)
    plt.rcParams.update({'font.family':font_manager.FontProperties(fname=font).get_name(),
        'font.size':10,'axes.unicode_minus':False,'axes.spines.top':False,'axes.spines.right':False,
        'axes.grid':True,'grid.alpha':.2,'figure.facecolor':'white','savefig.facecolor':'white'})

    def save(fig, name):
        path = figures/f'{name}.png'
        fig.savefig(path,dpi=180,bbox_inches='tight')
        fig.savefig(figures/f'{name}.svg',bbox_inches='tight')
        plt.close(fig)
        return path

    def select(rows, pattern):
        key = 'speed_mps' if rows is speed else 'start_path_m'
        return sorted((r for r in rows if r['pattern']==pattern),key=lambda r:r[key])

    # 速度掃引：接続待ち時間とTimeoutを分ける。
    fig, axes = plt.subplots(2,2,figsize=(8.3,5.5),layout='constrained')
    for p,ax in zip(PATTERNS,axes.flat):
        rows = select(speed,p)
        x = [r['speed_mps'] for r in rows]
        scale = 1e6 if p.endswith('tracking') else 1
        ax.plot(x,[r['mean_first_connected_s']*scale for r in rows],color=COLORS[p],lw=1.5)
        ax.set(title=NAMES[p],xlabel='各機の速度 [m/s]',ylabel='初回接続までの平均時間 ['+('µs' if scale>1 else 's')+']',xlim=(1,2))
        ax.ticklabel_format(axis='y',style='plain',useOffset=False)
    speed_time = save(fig,'speed_connection_time')
    fig,axes=plt.subplots(1,2,figsize=(8.3,3.1),layout='constrained')
    for i,p in enumerate(PATTERNS):
        rows=select(speed,p); x=[r['speed_mps'] for r in rows]
        axes[0].plot(x,[r['mean_timeouts'] for r in rows],label=NAMES[p],color=COLORS[p],ls=['-','--',':','-.'][i])
        axes[1].plot(x,[r['connected_fraction']*100 for r in rows],label=NAMES[p],color=COLORS[p],ls=['-','--',':','-.'][i])
    axes[0].set(xlabel='各機の速度 [m/s]',ylabel='初回接続までのTimeout [回]',ylim=(-.1,max(1,max(r['mean_timeouts'] for r in speed)*1.1)))
    axes[1].set(xlabel='各機の速度 [m/s]',ylabel='接続率 [%]',ylim=(-5,105))
    for ax in axes:ax.legend(fontsize=8)
    speed_outcomes=save(fig,'speed_outcomes')

    fig,axes=plt.subplots(2,2,figsize=(8.3,5.5),layout='constrained')
    for j,p in enumerate(('parallel_tracking','cross_tracking')):
        rows=select(distance,p); x=[r['start_path_m'] for r in rows]
        axes[0,j].plot(x,[r['initial_p_snr_db'] for r in rows],color=P_COLOR,label='P：272 GHz')
        axes[0,j].plot(x,[r['initial_q_snr_db'] for r in rows],color=Q_COLOR,label='Q：61 GHz')
        axes[0,j].set(title=NAMES[p],ylabel='開始時SNR [dB]');axes[0,j].legend(fontsize=8)
        axes[1,j].plot(x,[r['mean_first_connected_s']*1e6 for r in rows],color=COLORS[p])
        axes[1,j].set(xlabel='開始位置：交錯点の手前 [m]',ylabel='初回接続までの平均時間 [µs]')
        axes[1,j].ticklabel_format(axis='y',style='plain',useOffset=False)
    distance_plot=save(fig,'tracking_distance')

    # 共通制御フレームを仮想送信した場合のSNR/PERを、MACの送信回数と独立に表示。
    rf_images={}
    for metric in ('snr_db','control_per'):
        fig,axes=plt.subplots(2,2,figsize=(8.3,5.5),layout='constrained')
        for p,ax in zip(PATTERNS,axes.flat):
            data=profile[p]; x=data['remaining_path_m']
            ax.plot(x,data['p_'+metric],color=P_COLOR,label='P：272 GHz',lw=1.4)
            ax.plot(x,data['q_'+metric],color=Q_COLOR,label='Q：61 GHz',lw=1.4)
            ax.axvline(0,color='#555',ls=':',lw=.8);ax.axvline(20,color='#777',ls='--',lw=.8)
            ax.set(title=NAMES[p],xlabel='交錯点までの航路上距離 [m]（負値は通過後）',
                   ylabel='SNR [dB]' if metric=='snr_db' else 'MCS3・共通制御フレームのPER',xlim=(30,-60))
            if metric=='control_per':ax.set_ylim(-.04,1.04)
            ax.legend(fontsize=8)
        rf_images[metric]=save(fig,'profile_'+metric)
    fig,axes=plt.subplots(1,2,figsize=(8.3,3.0),layout='constrained')
    for p,ax,limits in zip(('parallel_fixed','cross_fixed'),axes,((3,-3),(15,1))):
        d=profile[p];x=d['remaining_path_m']
        mask=(d['q_control_per']<=.1)&(d['p_control_per']>=.9)
        ax.fill_between(x,0,1,where=mask,color='#e7b45a',alpha=.35,label='Q良好・P困難')
        ax.plot(x,d['p_control_per'],color=P_COLOR,label='P')
        ax.plot(x,d['q_control_per'],color=Q_COLOR,label='Q')
        for threshold in (.1,.9):ax.axhline(threshold,color='#888',ls=':',lw=.7)
        ax.set(title=NAMES[p],xlabel='交錯点までの航路上距離 [m]',ylabel='共通制御フレームPER',xlim=limits,ylim=(-.04,1.04))
        ax.legend(fontsize=8)
    asymmetry_zoom=save(fig,'control_asymmetry_zoom')
    fig,axes=plt.subplots(2,2,figsize=(8.3,5.5),layout='constrained')
    for i,mcs in enumerate((3,8)):
        for j,p in enumerate(('parallel_tracking','cross_tracking')):
            d=profile[p];ax=axes[i,j];x=d['remaining_path_m']
            ax.plot(x,d[f'p_data_mcs{mcs}_per'],color=P_COLOR,label='P：272 GHz')
            ax.plot(x,d[f'q_data_mcs{mcs}_per'],color=Q_COLOR,label='Q：61 GHz')
            ax.axvline(0,color='#555',ls=':',lw=.8);ax.axvline(20,color='#777',ls='--',lw=.8)
            ax.set(title=f'{NAMES[p]}・MCS{mcs}',xlabel='交錯点までの航路上距離 [m]',ylabel='DATAフレームのPER',xlim=(30,-60),ylim=(-.04,1.04))
            ax.legend(fontsize=8)
    data_plot=save(fig,'tracking_data_per')

    # 領域分類は指標であり、同時送信・実測成否を表すものではない。
    asymmetry=[]
    for p in PATTERNS:
        d=profile[p]
        for kind in ('control','data_mcs3','data_mcs8'):
            mask=(d[f'q_{kind}_per']<=.1)&(d[f'p_{kind}_per']>=.9)
            active=d['remaining_path_m']<=20+1e-9
            intervals=groups(d,mask)
            length=float(np.trapz(mask[active].astype(float),d['time_s'][active])) # 1m/sなので数値はmと等しい
            asymmetry.append(dict(pattern=p,probe=kind,intervals_m=intervals,
                                  length_m_within_speed_observation=length,
                                  duration_s_at_1mps=length,duration_s_at_2mps=length/2))
    (root/'asymmetry_regions.json').write_text(json.dumps(asymmetry,ensure_ascii=False,indent=2))
    all_rows=speed+distance
    total_trials=int(sum(r['trials'] for r in all_rows))
    connected=int(sum(r['connected_trials'] for r in all_rows))
    zero=int(sum(r['zero_timeout_connected_trials'] for r in all_rows))
    max_timeout=int(max(r['max_timeouts'] for r in all_rows))
    n=verified['trials_per_condition']
    control_bytes=int(profile['parallel_fixed']['control_bytes'][0])
    report=LocalReport('非対称通信：290 K・SNR→PERによる4シナリオ評価')
    report.page('非対称通信の速度・開始位置評価')
    report.text(f'4シナリオの速度掃引404条件と、追従の開始位置掃引602条件、計1,006条件を各{n}試行。合計{total_trials:,}試行を実行した。Pは272 GHz、Qは61 GHz。P側LNA15 dB・受信系全体NF8 dB、Q側全体NF0 dB、温度290 K。')
    report.table([['評価結果','実行結果'],['接続成立',f'{connected:,} / {total_trials:,}試行'],
                  ['Timeout 0回で接続',f'{zero:,} / {total_trials:,}試行'],['観測した最大Timeout',f'{max_timeout}回'],
                  ['SNR差','QがPより8.9848 dB高い（全RF診断点で照合）'],
                  ['固定アンテナ','Qが受信しやすくPが受信しにくい区間を確認'],
                  ['追従アンテナ','開始0〜30 mの制御フレームは低PER。DATAの高MCSでは片方向劣化が生じる']], [155,355])
    report.text('約9 dBのSNR差は片方向の受信性能差を生むが、それだけで接続Timeoutが増えるとは限らない。現在のMACではPのBeaconを復号するまで探索し、探索中は接続待ちタイマーを開始しない。固定アンテナの待ち時間は主にこの探索時間となった。')
    report.text('本評価は初回リンク確立まで。接続成立時点で各試行を終了する。以後の連続転送量・切断回数は測定していない。DATAのPER図は同じ位置で仮想フレームを送信した場合の診断である。',small=True)
    report.text('結果は既存の理論PHY・固定LUTに基づく予測。QのNF0 dBは理想仮定であり、実機の非対称性を保証するものではない。',small=True)

    report.page('1　実行条件と基準点')
    report.table([['RF条件','P：B→A','Q：A→B'],['周波数','272 GHz','61 GHz'],['送信機出力','−1 dBm','−8 dBm'],
        ['送信挿入損失','0 dB','3 dB'],['送信／受信正面利得','各24 dBi','各23 dBi'],['LNA利得','15 dB','0 dB'],
        ['変換利得／受信挿入損失','＋3 dB／3 dB','0 dB／3 dB'],['受信系全体NF','8 dB','0 dB（理想仮定）'],
        ['温度／雑音帯域','290 K／2.1648 GHz','290 K／2.1648 GHz'],['アンテナ端換算雑音','−72.621 dBm','−80.621 dBm']], [160,175,175])
    report.text('熱雑音＝10 log10(k×290×B)＋30＝−80.621 dBm。入力換算雑音＝熱雑音＋全体NF。SNR＝アンテナ端信号電力−入力換算雑音。LNA利得は信号と雑音の両方へ適用されるためSNRでは相殺される。')
    report.text('アンテナ角度特性は既存E/H面CSVを正面利得23/24 dBiに正規化。drone_params.yamlに合わせ減衰上限30 dBを使用する。十字固定の角度は対称通信設定と同じだが、RF条件は本表の非対称設定である。')
    report.text('自由空間相当の距離指数2。高度はA＝20 m、B＝30 m。並行経路の横間隔は10 m。大気吸収・反射・遮蔽・偏波ずれ・ドップラーは含めない。',small=True)

    report.page('2　掃引範囲とアンテナの向き')
    report.table([['項目','条件'],['速度掃引','並行固定／並行追従／十字固定／十字追従。各機1.00〜2.00 m/s、0.01 m/s刻み、各101点。'],
        ['速度掃引の開始位置','全4シナリオとも各機が交錯点の20 m手前から開始。'],
        ['追従の追加実験','並行追従／十字追従。各機1.0 m/s固定、開始位置0.0〜30.0 m手前、0.1 m刻み、各301点。'],
        ['終了条件','初回接続成立、または各機が交錯点の60 m先へ到達。未接続は打切りとして別集計。'],
        ['0 m開始','交錯点の真上・真下／航路中央から離脱を開始。機体間距離0 mではない。'],
        ['試行数','各条件30試行。PHY抽選とMACのseedを変更。'],
        ['十字固定の角度','進行方向基準RPY：A＝(0, −π/4, −π/4)、B＝(0, ＋π/4, ＋π/4)。'],
        ['並行固定の角度','仰角A＝＋45°、B＝−45°。各機の左90°方向を向き、航路中央で正対。'],
        ['追従の角度','相手方向へ瞬時追従。機械的な追従速度・遅延はモデル化しない。']], [120,390])
    report.text('十字固定は交錯点の手前7.0711 mで正対する。機体間距離は14.1421 m。20 m手前から開始するので、正対時刻は12.9289／速度 [秒]。YAMLの仰角はRPYのpitchと逆符号で指定される。')
    report.text('「開始位置」は各機の航路上距離。開始20 mでは並行の機体間距離42.4264 m、十字は30.0000 m。開始0 mでも並行14.1421 m、十字10.0000 mの間隔がある。',small=True)

    report.page('3　速度による初回接続時間')
    report.image(speed_time)
    rows=[['シナリオ','1.00 m/s','1.50 m/s','2.00 m/s']]
    for p in PATTERNS:
        cells=[]
        for v in (1.,1.5,2.):
            r=next(r for r in speed if r['pattern']==p and r['speed_mps']==v)
            t=r['mean_first_connected_s']
            cells.append(f'{t*1e6:.3f} µs' if p.endswith('tracking') else f'{t:.4f} s')
        rows.append([NAMES[p],*cells])
    report.table(rows,[120,130,130,130])
    report.text('固定アンテナは指向性の良い位置へ到達するまで待つため、速度を上げると待ち時間が短くなる。追従は開始時から接続可能で、図の縦軸単位はµs。両者の縦軸尺度は異なる。',small=True)

    report.page('4　Timeoutと接続率')
    report.image(speed_outcomes)
    report.text(f'速度掃引全404条件の各{n}試行について、接続率と初回接続までのTimeoutを集計した。重なった曲線は同じ値を表す。')
    report.text('Timeoutは、接続手順に入った端点の待機期限500 msが満了した回数。Beacon探索中、フレーム単体の受信失敗、再送回数とは別である。P側BeaconのPERが高くても、まだRequest送信に至らなければTimeoutは増えない。')
    report.text('Beaconは140 µs周期で反復するため、1フレームの成功率が低い区間でも繰返しの中で受信できる。PERが高いことと、観測期間中に一度も接続できないことは同義ではない。',small=True)
    report.table([['指標','意味'],['接続までの時間','開始から両端接続成立まで。探索時間を含む。'],['Timeout回数','接続手順開始後の待機期限満了。探索時間は含まない。'],
        ['方向別フレーム失敗率','実際に送信され、初回接続前に結果通知されたフレームの失敗割合。Beacon数が多いため方向間の単純比較には不向き。'],
        ['共通フレームPER','同じMCS・同じ長さの仮想フレームをP/Qに適用した品質比較。MACの送信機会の差を除く。']], [135,375])

    report.page('5　追従：開始位置0〜30 mの追加実験')
    report.image(distance_plot)
    report.text('1.0 m/s固定、0.1 m刻みの全301点を、並行追従・十字追従それぞれ30試行した。開始位置が遠いほどSNRは下がるが、今回の範囲ではMCS3の制御フレームを受信できた。')
    report.table([['シナリオ','条件数','接続成立','Timeout 0回で接続'],*[[NAMES[p],str(len(select(distance,p))),
        f"{int(sum(r['connected_trials'] for r in select(distance,p))):,} / {int(sum(r['trials'] for r in select(distance,p))):,}",
        f"{int(sum(r['zero_timeout_connected_trials'] for r in select(distance,p))):,}"] for p in ('parallel_tracking','cross_tracking')]], [125,65,160,160])
    report.text('図の微小な接続時間差には伝搬遅延・内部時刻量子化が含まれる。PERの0はLUT上での値であり、真の誤り確率が厳密に0である証明ではない。',small=True)

    report.page('6　軌道に沿ったP/QのSNR')
    report.image(rf_images['snr_db'])
    report.text('横軸は各機の航路上の残り距離。左から右へ時間が進み、正値が接近、0が航路中央、負値が通過後。破線＋20 mが速度掃引の開始位置。診断用には＋30 mから−60 mまで0.01 m刻みで計算した。')
    report.text('全シナリオでQ−P＝8.9848 dB。共通のアンテナ角度特性を両帯域へ適用し、正面利得だけ23/24 dBiに補正しているため差は一定となる。実機の帯域別指向性や大気吸収を入れた場合の一定性は保証しない。')
    report.text('固定では距離よりも指向ずれの影響が大きい。十字固定は交錯前に正対し、交錯時には相手が上下方向となって固定仰角から外れる。追従では主に距離によってSNRが変わる。',small=True)

    report.page('7　共通制御フレームのPERと非対称区間')
    report.image(rf_images['control_per'])
    report.text(f'比較用にMCS3・本文{control_bytes} byte（Beacon相当）を両方向に仮想送信し、ヘッダーを含むPERを計算した。実際のRequest等とは本文長が異なる。PERはSNRをEs/N0へ変換し、短縮LDPC・ヘッダーのLUTから合成する。')
    report.text('「Q受信良好・P受信困難」の指標を、QのPER≦0.1、かつPのPER≧0.9と定義した。これは領域を比較するための基準であり、MACの受信判定へ新たな閾値を追加したものではない。',small=True)

    report.page('8　片方向だけ受信しやすい区間の長さ')
    report.image(asymmetry_zoom)
    region_rows=[['シナリオ','制御フレームの該当区間 [m]','1 m/sでの時間','2 m/sでの時間']]
    for p in PATTERNS:
        r=next(r for r in asymmetry if r['pattern']==p and r['probe']=='control')
        region_rows.append([NAMES[p],interval_text(r['intervals_m']),f"{r['duration_s_at_1mps']:.3f} s",f"{r['duration_s_at_2mps']:.3f} s"])
    report.table(region_rows,[90,230,95,95])
    report.text('区間は0.01 m刻みの診断点で条件を満たす最初と最後の位置。時間は速度掃引の観測範囲＋20〜−60 m内の指標を台形積分した近似値。複数区間は合算する。境界の精度は0.01 m程度であり、連続位置の厳密な境界ではない。')
    report.text('この時間は、仮に同じ制御フレームを各位置で送信したときの受信性能差が大きい時間である。実際にQを送信できた時間や、接続後の片方向通信継続時間ではない。初回接続後に現れる区間もRF診断には含まれる。')
    report.text('SNR差が約9 dBあっても、低速・強い符号化の制御フレームでは両方向とも受信できる場合がある。非対称性の影響は、SNR差だけでなくMCS・フレーム長・送信する位置に依存する。')

    report.page('9　制御が通ってもDATAが通るとは限らない')
    report.image(data_plot)
    report.text('追従の同じ位置で、2048 byte payload（サブフレーム付加後2056 byte）のDATAをMCS3またはMCS8で仮想送信したPER。ヘッダーを含む。MCS3はBPSK・符号化率11/15、MCS8はQPSK・符号化率14/15の既存モデルを使用。')
    report.text('MCS8ではP側だけPERが高い領域が現れる。接続成立は短いMCS3制御フレームで評価しているため、接続成功と高レートDATAの成功は区別する必要がある。',small=True)
    report.text('本図はMCSを固定した診断であり、接続後の適応MCS・ACK・再送を走らせたスループット測定ではない。',small=True)

    report.page('10　検証内容と結果の適用範囲')
    report.table([['検証項目','結果'],['ケース数・試行数',f"{verified['conditions']:,}条件・{verified['trials']:,}試行。欠落・重複なし。"],
        ['指定範囲','速度101点×4、開始位置301点×2。各条件30試行。'],['幾何','十字固定の正対位置と両追従の指向ずれを数値照合。'],
        ['SNR','全RF診断点でQ−P＝8.9848 dB、温度290 Kを確認。'],['再現情報','設定・入力CSV・LUT・ソーススナップショット・実行バイナリSHA256・seedを保存。'],
        ['実行方式','Gazeboと共通のC++通信コアを直接実行。Gazeboプロセスは起動していない。']], [130,380])
    report.text('理想同期・既知チャネル・AWGNの理論PHYを使用。MCS0/3の符号化やフレーム構成には既存モデルの仮定がある。実機測定による性能保証ではない。')
    report.text('LUTは各点256試行。曲線範囲未満ではPER＝1、範囲超過では末端値を保持する。観測誤り0の点をPER＝0として使っているため、30試行すべて成功でも真の成功率100%とは言えない。今回の試行間ばらつきは固定LUTに条件付けられ、LUT推定誤差を含まない。')
    report.text('方向別フレーム集計は初回CONNECTED通知より前にFRAME_RESULTが出たもの。接続成立と同時に終了するため、最後のフレーム結果通知が集計に入らない場合がある。接続率・接続時刻の集計には影響しない。',small=True)
    report.table([['ファイル','内容'],['all_summary.csv / all_trials.csv','全条件の集計／30試行の個票。'],['profiles/*.csv','位置ごとのP/Q信号電力・SNR・制御／DATAのPER。'],
        ['asymmetry_regions.json','非対称区間と速度別の滞在時間。'],['configs/ / inputs/ / internal/','実行設定、アンテナ・LUT、検証結果と再現情報。'],
        ['report/figures/','編集用PNG・SVG図。PDF・Word・HTMLは同一内容。']], [185,325])
    report.write(out/'asymmetric_snr_report')
    # 共有用ZIPはレポートと集計・設定・入力・再実行ツールを含め、個別の中間ファイルは省く。
    with ZipFile(root/'asymmetric_snr_report_bundle.zip','w',ZIP_DEFLATED) as archive:
        paths=list(out.rglob('*'))
        paths += [root/name for name in ('all_summary.csv','all_trials.csv','speed_summary.csv','distance_summary.csv','conditions.csv','asymmetry_regions.json')]
        for folder in ('configs','inputs','profiles'):
            paths += list((root/folder).glob('*'))
        paths += [root/'internal/provenance.json',root/'internal/verification.json']
        paths += list((root/'internal/source_snapshot').rglob('*'))
        for path in paths:
            if path.is_file():archive.write(path,path.relative_to(root))
        for filename in ('run_asymmetric_snr_study.py','write_asymmetric_snr_report.py','asymmetric_link_profile.cpp','local_comparison_report.py'):
            source=Path(__file__).resolve().parent/filename
            archive.write(source,'tools/'+filename)
    print(f'レポート完成: {out}',flush=True)


if __name__=='__main__':
    main()
