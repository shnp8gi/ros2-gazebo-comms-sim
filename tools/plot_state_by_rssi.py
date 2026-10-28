#!/usr/bin/env python3
"""RSSI帯を分母にした状態滞在率P(state | RSSI bin)と接続境界を可視化する。"""
import argparse
import math
from pathlib import Path
import csv
import os
os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib-drone-state-rssi')
import matplotlib
matplotlib.use('Agg')
import numpy as np
from matplotlib import font_manager
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from plot_protocol_occupancy import NAMES, TITLES, STATES, STATE_LABELS, COLORS, load_run, write_csv


def signal_segments(run):
    # イベント時刻の電力も含め、短い接続手順を10ms前のRSSIへ割り当てない。
    observations = {}
    for row in run['samples']:
        observations[float(row['time_s'])] = (float(row['rssi_ab_dbm']), float(row['rssi_ba_dbm']))
    for row in run['events']:
        t = float(row['observation_time_s'])
        if 0 <= t <= run['duration']:
            observations[t] = (float(row['rssi_ab_dbm']), float(row['rssi_ba_dbm']))
    observed_times = np.array(sorted(observations))
    power = np.array([observations[t] for t in observed_times])
    times = np.unique(np.concatenate((observed_times, [t for t,_ in run['changes']])))
    rssi = np.column_stack([np.interp(times, observed_times, power[:,i]) for i in (0,1)])
    changes = np.array([t for t,_ in run['changes']])
    phase = np.searchsorted(changes, times[:-1], side='right')-1
    stages = np.array([STATES.index(s) for _,s in run['changes']])[phase]
    return times, rssi, stages


def duration_matrix(segments, edges, direction):
    times, rssi, stages = segments
    matrix = np.zeros((len(edges)-1, len(STATES)))
    for t0,t1,r0,r1,state in zip(times[:-1],times[1:],rssi[:-1,direction],rssi[1:,direction],stages):
        cuts = [0.,1.]
        if r1 != r0:
            lo,hi = sorted((r0,r1))
            first,last = np.searchsorted(edges,[lo,hi],side='right')
            cuts += [(edge-r0)/(r1-r0) for edge in edges[first:last] if 0<(edge-r0)/(r1-r0)<1]
        cuts.sort()
        for a,b in zip(cuts,cuts[1:]):
            value = r0+(r1-r0)*(a+b)/2
            index = np.searchsorted(edges,value,side='right')-1
            if 0<=index<len(matrix): matrix[index,state] += (t1-t0)*(b-a)
    return matrix


def draw(runs, segments, output, edges, stem, detail):
    fig,axes=plt.subplots(4,2,figsize=(16,13),sharex=True,sharey=True)
    fig.subplots_adjust(left=.075,right=.99,bottom=.105,top=.92,wspace=.17,hspace=.5)
    records=[]
    for row,run in enumerate(runs):
        for direction,side in enumerate(('ab','ba')):
            matrix=duration_matrix(segments[run['name']],edges,direction)
            if not detail:
                assert math.isclose(float(matrix.sum()),run['duration'],abs_tol=1e-7)
                for state in STATES:
                    expected=sum(end-start for start,end,s in run['intervals'] if s==state)
                    assert math.isclose(float(matrix[:,STATES.index(state)].sum()),expected,abs_tol=1e-7)
            totals=matrix.sum(axis=1)
            percentages=np.divide(matrix,totals[:,None],out=np.zeros_like(matrix),where=totals[:,None]>0)*100
            assert np.allclose(percentages[totals>0].sum(axis=1),100)
            ax=axes[row,direction];bottom=np.zeros(len(totals));present=totals>0
            for index,state in enumerate(STATES):
                ax.bar((edges[:-1]+edges[1:])[present]/2,percentages[present,index],
                       width=np.diff(edges)[present]*.94,bottom=bottom[present],color=COLORS[index],linewidth=0)
                bottom += percentages[:,index]
                for i,total in enumerate(totals):
                    records.append(dict(condition=run['name'],direction=side,rssi_low_dbm=edges[i],rssi_high_dbm=edges[i+1],
                        state=state,duration_s=matrix[i,index],rssi_bin_duration_s=total,
                        state_percentage=percentages[i,index] if total>0 else ''))
            frequency=float(run['config'][f'radio_{side}']['frequency_hz'])/1e9
            ax.axvline(run['config'][f'radio_{side}']['sensitivity_dbm'],color='black',ls='--',lw=1)
            ax.set_title(f'{run["title"]} — {"A→B" if side=="ab" else "B→A"}（{frequency:g} GHz）')
            ax.set_ylim(0,102);ax.set_xlim(edges[0],edges[-1]);ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
            ax.set_ylabel('このRSSI帯の中での状態滞在率 [%]')
            ax.set_xlabel('TJX入力信号電力／RSSI [dBm]')
            if not present.any(): ax.text(.5,.5,'このRSSI範囲の観測なし',ha='center',transform=ax.transAxes)
    title = 'RSSIごとの状態滞在率：' + ('受信閾値付近を1 dB刻みで表示' if detail else '基本5 dB刻み・受信閾値で区間を分割')
    fig.suptitle(title+'\n分母は各RSSI帯の滞在時間。観測した帯ごとに合計100%。空白は観測なし。破線は受信閾値。',fontsize=13,y=.985)
    fig.legend(handles=[Patch(color=c,label=l) for c,l in zip(COLORS,STATE_LABELS)],
               loc='lower center',bbox_to_anchor=(.5,.01),ncol=5,fontsize=10)
    for extension in ('png','pdf'):fig.savefig(output/f'{stem}.{extension}',dpi=170)
    plt.close(fig)
    write_csv(output/f'{stem}.csv',records,records[0].keys())


def boundaries(runs,output):
    fig,axes=plt.subplots(4,2,figsize=(16,12),constrained_layout=True)
    records=[]
    for row,run in enumerate(runs):
        events=run['events'];connected=[e for e in events if e['event']=='CONNECTED']
        waits=[e for e in events if e['event']=='WAIT_STARTED' and e['frame'] in ('DATA','DATA_ACK')]
        disconnects=[e for e in events if e['event']=='DISCONNECTED']
        nonbeacon=[e for e in events if e['event']=='RX_FAILED' and e['frame']!='BEACON']
        awgn=[e for e in nonbeacon if e['reason']=='AWGN_PACKET_ERROR']
        records.append(dict(condition=run['name'],connections=len(connected),disconnections=len(disconnects),
                            setup_rx_failures=sum(e['frame'] in ('REQUEST','RESPONSE','ACK') for e in nonbeacon),
                            data_rx_failures=sum(e['frame'] in ('DATA','DATA_ACK') for e in nonbeacon),
                            awgn_nonbeacon_failures=len(awgn)))
        for col,ax in enumerate(axes[row]):
            ax.set_title(run['title'] + (' — 最初の接続付近' if col==0 else ' — 最初のデータ失敗と待ち'))
            ax.set_xlabel('シミュレーション時刻 [s]');ax.set_ylabel('TJX入力信号電力 [dBm]');ax.grid(alpha=.2)
            anchor = float(connected[0]['time_s']) if connected and col==0 else (float(waits[0]['time_s']) if waits and col==1 else None)
            if anchor is None:
                ax.text(.5,.5,'該当する接続／データ失敗なし',ha='center',transform=ax.transAxes)
                continue
            start,end=(anchor-.015,anchor+.015) if col==0 else (anchor-.05,anchor+.55)
            ts=[float(r['time_s']) for r in run['samples']]
            for side,color in (('ab','#b44a3e'),('ba','#2878b5')):
                ax.plot(ts,[float(r[f'rssi_{side}_dbm']) for r in run['samples']],color=color,lw=1,label=side.upper())
            ax.axhline(-68.5,color='black',ls='--',lw=1)
            for event in events:
                t=float(event['time_s'])
                if not start<=t<=end:continue
                if event['event'] in ('RX_FAILED','RX_OK'):
                    key='rssi_ab_dbm' if event['direction']=='A_TO_B' else 'rssi_ba_dbm'
                    point_t=float(event['observation_time_s'])
                    ax.scatter(point_t,float(event[key]),marker='x' if event['event']=='RX_FAILED' else 'o',
                               color='#c0392b' if event['event']=='RX_FAILED' else '#24864a',s=22,zorder=4)
                elif event['event']=='CONNECTED':ax.axvline(t,color='#24864a',lw=1)
                elif event['event']=='TIMEOUT' and event['reason']!='INACTIVITY_TIMEOUT':
                    ax.axvline(t,color='#c0392b',ls=':',lw=1.5)
                elif event['event']=='DISCONNECTED':ax.axvline(t,color='#2878b5',ls=':',lw=1.5)
            if col==1:
                timeout=float(waits[0]['deadline_s'])
                ax.axvspan(anchor,timeout,color='#dc7e30',alpha=.12)
            ax.set_xlim(start,end)
            subset=[r for r in run['samples'] if start-.01<=float(r['time_s'])<=end+.01]
            vals=[float(r[f'rssi_{side}_dbm']) for r in subset for side in ('ab','ba')]
            if vals:ax.set_ylim(min(min(vals),-68.5)-1,max(max(vals),-68.5)+1)
            if row==0:ax.legend(loc='best')
    fig.suptitle('接続境界のログ確認：緑丸＝制御フレーム受信成功、赤×＝受信失敗\n'
                 '緑縦線＝接続成立、赤点線＝接続前タイムアウト、青点線＝切断、薄橙＝失敗後の待ち。',fontsize=13)
    for extension in ('png','pdf'):fig.savefig(output/f'connection_boundaries.{extension}',dpi=170,bbox_inches='tight')
    plt.close(fig)
    write_csv(output/'connection_boundary_summary.csv',records,records[0].keys())
    return records


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('directory',type=Path)
    args=parser.parse_args()
    font=Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    if font.is_file():
        font_manager.fontManager.addfont(str(font));plt.rcParams['font.family']=font_manager.FontProperties(fname=str(font)).get_name()
    plt.rcParams.update({'font.size':10,'axes.unicode_minus':False,'pdf.fonttype':3})
    runs=[load_run(args.directory,n,t) for n,t in zip(NAMES,TITLES)]
    segments={r['name']:signal_segments(r) for r in runs}
    minimum=min(float(s[1].min()) for s in segments.values());maximum=max(float(s[1].max()) for s in segments.values())
    edges=np.arange(math.floor(minimum/5)*5,(math.floor(maximum/5)+2)*5,5.)
    thresholds=[float(r['config'][f'radio_{side}']['sensitivity_dbm']) for r in runs for side in ('ab','ba')]
    edges=np.unique(np.concatenate((edges,thresholds)))
    output=args.directory/'protocol_analysis';output.mkdir(exist_ok=True)
    draw(runs,segments,output,edges,'state_by_rssi',False)
    draw(runs,segments,output,np.arange(-78.5,-58.4,1),'state_by_rssi_boundary',True)
    records=boundaries(runs,output)
    (output/'state_by_rssi_readme.txt').write_text('''RSSIごとの状態滞在率
割合 = そのRSSI帯でその状態にいた時間 / そのRSSI帯にいた総時間 × 100。
全40秒を分母にしたRSSI分布とは異なる。各方向は同じプロトコル状態を、その方向の信号電力で分類している。
状態はevents.csvから復元。信号電力は10msログにイベント観測値も加え、観測間を線形補間する。
状態境界とRSSI帯の横断時刻で時間を分割。短い接続手順も10ms前の電力へ丸めない。
補間による近似を含む。未観測RSSI帯は空欄、観測した帯は合計100%。
全体図は基本5dB刻みで受信閾値の位置に追加境界。境界拡大は1dB刻み。
接続手順は約0.11msと短く、棒では細くなるがCSVには全段階の秒数と割合を保存。
探索には未受信Beaconの送信/待機を含み、IFSは直前フレーム段階に含む。
接続中・データ交換には失敗検出までの時間が含まれる。無応答待ち中は論理接続も維持される。
時間滞在率は条件付きの集計であり、将来の接続成功確率ではない。
''')
    print('状態ごとの時間合計・RSSI帯内割合100%を検証済み')
    for row in records:print(row)
    print('保存先:',output)


if __name__=='__main__':main()
