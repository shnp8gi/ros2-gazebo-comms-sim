#!/usr/bin/env python3
"""4条件のプロトコル状態遷移と、時間加重RSSI滞在割合を既存ログから作図する。"""
import argparse
from bisect import bisect_right
import csv
from pathlib import Path
import os
import math

os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib-drone-protocol')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np
import yaml

NAMES = ('parallel_fixed', 'parallel_tracking', 'cross_fixed', 'cross_tracking')
TITLES = ('並行・固定', '並行・追従', '十字・固定（横向き）', '十字・追従')
STATES = ('SCANNING', 'BEACON', 'REQUEST', 'RESPONSE', 'ACK', 'CONNECTED',
          'WAIT_RESPONSE', 'WAIT_ACK', 'WAIT_DATA_ACK')
STATE_LABELS = ('探索／Beacon未受信', 'Beacon段階', 'Request段階', 'Response段階',
                '接続ACK段階', '接続中／データ交換', 'Response待ち', '接続ACK待ち', '無応答・切断待ち')
COLORS = ('#52677a', '#8a64ab', '#3978bd', '#238d90', '#72a74c', '#24864a',
          '#c68b29', '#ba7728', '#dc7e30')


def read_csv(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def write_csv(path, rows, fields):
    with path.open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def reconstruct(events, duration):
    """短い接続段階を10msログから推測せず、TX/RX/待ちイベントから復元する。"""
    changes = [(0., 'SCANNING')]
    timeouts, disconnections, handshake = [], [], []
    beacon_tx = None
    indexed = bool(events and 'beacon_index' in events[0])
    beacon_times = {}

    def change(t, state):
        if t < changes[-1][0] - 1e-9:
            raise ValueError('状態遷移の時刻が逆行しています')
        if state == changes[-1][1]:
            return
        if t == changes[-1][0]:
            changes[-1] = (t, state)
        else:
            changes.append((t, state))

    for event in events:
        kind, frame, t = event['event'], event['frame'], float(event['time_s'])
        if kind == 'TX' and frame == 'BEACON':
            if indexed:
                beacon_times[event['beacon_index']] = t
            else:
                beacon_tx = t
        elif (kind == 'BEACON_SELECTED' if indexed else kind == 'RX_OK' and frame == 'BEACON'):
            if indexed:
                beacon_tx = beacon_times[event['beacon_index']]
            if beacon_tx is None:
                raise ValueError('成功Beaconの送信時刻がありません')
            # 未受信Beaconの周期送信は探索としてまとめる。成功した交換のみ表示。
            # タイムアウト直前に送られ、その後受信したBeaconでも時刻を逆行させない。
            change(max(beacon_tx, changes[-1][0]), 'BEACON')
        elif kind == 'TX' and frame in ('REQUEST', 'RESPONSE', 'ACK'):
            change(t, frame)
        elif kind == 'CONNECTED':
            change(t, 'CONNECTED')
            handshake.append((beacon_tx, t))
        elif kind == 'WAIT_STARTED':
            state = {'REQUEST': 'WAIT_RESPONSE', 'RESPONSE': 'WAIT_RESPONSE',
                     'ACK': 'WAIT_ACK', 'DATA': 'WAIT_DATA_ACK', 'DATA_ACK': 'WAIT_DATA_ACK'}[frame]
            change(t, state)
        elif kind == 'TIMEOUT' and event['reason'] != 'INACTIVITY_TIMEOUT':
            timeouts.append((t, changes[-1][1], float(event['wait_start_s'])))
            change(t, 'SCANNING')
        elif kind == 'DISCONNECTED':
            disconnections.append((t, changes[-1][1], float(event['wait_start_s'])))
            change(t, 'SCANNING')
    intervals = [(start, changes[i+1][0] if i+1 < len(changes) else duration, state)
                 for i, (start, state) in enumerate(changes)]
    if any(end < start for start, end, _ in intervals):
        raise ValueError('状態区間に負の時間があります')
    return changes, intervals, timeouts, disconnections, handshake


def load_run(root, name, title):
    directory = root/name
    if not (directory/'completed').is_file():
        raise ValueError(f'実行が未完了です: {directory}')
    config = yaml.safe_load((directory/'config.yaml').read_text())
    events, samples = read_csv(directory/'events.csv'), read_csv(directory/'timeseries.csv')
    summary = read_csv(directory/'summary.csv')[0]
    duration = float(config['duration_s'])
    changes, intervals, timeouts, disconnections, handshake = reconstruct(events, duration)
    assert math.isclose(sum(end-start for start,end,_ in intervals), duration, abs_tol=1e-8)
    connected = sum(end-start for start,end,state in intervals if state in ('CONNECTED','WAIT_DATA_ACK'))
    assert math.isclose(connected, float(summary['connected_time_s']), abs_tol=2e-8)
    assert len(handshake) == int(summary['connections'])
    if 'disconnections' in summary:
        assert len(timeouts) == int(summary['timeouts'])
        assert len(disconnections) == int(summary['disconnections'])
    else:
        assert len(timeouts)+len(disconnections) == int(summary['timeouts'])
    timestamps = [t for t,_ in changes]
    # 復元した区間を既存の時系列状態とも照合する。CSV丸めの境界は隣接状態を許可。
    for sample in samples:
        t = float(sample['time_s'])
        state = changes[max(0, bisect_right(timestamps,t)-1)][1]
        observed = sample['state']
        same = state == observed or (state == 'SCANNING' and observed == 'BEACON')
        boundary = any(abs(t-timestamps[j]) < 1.1e-9 for j in
                       range(max(0,bisect_right(timestamps,t)-1), min(len(timestamps),bisect_right(timestamps,t)+1)))
        if not same and not boundary:
            raise ValueError(f'状態ログの復元が不一致: {name} t={t}: {state}/{observed}')
    return dict(name=name,title=title,config=config,events=events,samples=samples,
                duration=duration,changes=changes,intervals=intervals,timeouts=timeouts,
                disconnections=disconnections,handshake=handshake)


def save_figure(fig, output, stem):
    fig.savefig(output/f'{stem}.png', dpi=170)
    fig.savefig(output/f'{stem}.pdf')
    plt.close(fig)


def protocol_plots(runs, output):
    fig, axes = plt.subplots(4, 2, figsize=(17, 14), sharey=True, constrained_layout=True)
    for row, run in enumerate(runs):
        for col, ax in enumerate(axes[row]):
            xs = [t for t,_ in run['changes']] + [run['duration']]
            ys = [STATES.index(s) for _,s in run['changes']]
            ax.step(xs, ys+[ys[-1]], where='post', color='#a0a8b0', lw=1)
            for start,end,state in run['intervals']:
                ax.hlines(STATES.index(state),start,end,color=COLORS[STATES.index(state)],lw=2.7)
            for t,state,_ in run['timeouts']:
                ax.plot(t,STATES.index(state),'x',color='#c0392b',ms=8,mew=2)
            for t,state,_ in run['disconnections']:
                ax.plot(t,STATES.index(state),'D',color='#2878b5',ms=6)
            ax.axvline(run['config']['encounter_time_s'],color='#79838d',ls=':',lw=1)
            ax.set_xlim((0,run['duration']) if col==0 else
                        (run['config']['encounter_time_s']-3,run['config']['encounter_time_s']+3))
            ax.set_ylim(-.5,8.6)
            ax.set_yticks(range(len(STATES)), STATE_LABELS, fontsize=9)
            ax.grid(alpha=.2)
            ax.set_xlabel('シミュレーション時刻 [s]')
            ax.set_title(run['title'] + (' — 全時間' if col==0 else ' — 最接近付近'))
            if not run['handshake']:
                only_scanning = all(s=='SCANNING' for _,_,s in run['intervals'])
                message = (f'接続未成立：全{run["duration"]:g}秒を探索段階で経過' if only_scanning
                           else '接続未成立：接続手順で失敗')
                ax.text(.5,.48,message,ha='center',transform=ax.transAxes,color='#52677a')
    fig.suptitle('リンク確立プロトコルの状態遷移（イベントログから復元）\n'
                 '点線：最接近20秒 ／ 赤×：接続前タイムアウト ／ 青◆：接続後の切断',fontsize=14)
    save_figure(fig,output,'protocol_states')

    fig, axes = plt.subplots(2,2,figsize=(14,9),constrained_layout=True)
    for ax,run in zip(axes.flat,runs):
        ax.set_title(run['title'])
        ax.set_yticks(range(5), ('Beacon段階','Request段階','Response段階','接続ACK段階','接続成立'))
        ax.set_ylim(-.4,4.6);ax.set_xlim(-5,130);ax.grid(alpha=.2)
        ax.set_xlabel('成功Beaconの送信開始からの経過時間 [µs]')
        if not run['handshake']:
            ax.text(.5,.5,'接続未成立\n成功した接続手順はありません',ha='center',va='center',transform=ax.transAxes)
            continue
        origin,end = run['handshake'][0]
        selected=[(t,s) for t,s in run['changes'] if origin<=t<=end]
        xs=[(t-origin)*1e6 for t,_ in selected]+[130]
        ys=[STATES.index(s)-1 for _,s in selected]
        ax.step(xs,ys+[ys[-1]],where='post',color='#24864a',lw=2)
        for event in run['events']:
            t=float(event['time_s'])
            if origin<=t<=end and event['event']=='RX_OK' and event['frame'] in STATES[1:5]:
                x=(t-origin)*1e6;y=STATES.index(event['frame'])-1
                ax.scatter(x,y,color='#3978bd',s=25,zorder=3)
                if event['frame']!='ACK':
                    ax.axvspan(x,x+run['config']['protocol']['ifs_us'],color='#adb5bd',alpha=.18)
        ax.scatter((end-origin)*1e6,4,color='#24864a',s=35,zorder=3)
        ax.text(.02,.96,f'Beacon開始：{origin:.6f} s\n確立時間：{(end-origin)*1000:.5f} ms',
                va='top',transform=ax.transAxes,fontsize=10,bbox=dict(facecolor='white',edgecolor='#ddd',alpha=.9))
    fig.suptitle('接続手順の拡大図（各条件の最初の成功接続）\n青点：受信完了 ／ 灰色の帯：IFS（直前の段階に含めて表示）',fontsize=14)
    save_figure(fig,output,'protocol_handshake_zoom')


def occupancy(runs, output, bin_width):
    values=[float(r[f'rssi_{side}_dbm']) for run in runs for r in run['samples'] for side in ('ab','ba')]
    low=math.floor(min(values)/bin_width)*bin_width
    high=(math.floor(max(values)/bin_width)+1)*bin_width
    edges=np.arange(low,high+bin_width*.5,bin_width)
    records, summaries, histograms = [], [], {}
    for run in runs:
        samples=run['samples']
        active=[(s,e) for s,e,state in run['intervals'] if state=='CONNECTED']
        for scope in ('all','connected_without_wait'):
            weights=[]
            for a,b in zip(samples,samples[1:]):
                start,end=float(a['time_s']),float(b['time_s'])
                if end<=start:raise ValueError('時系列のサンプル間隔が不正です')
                weights.append(end-start if scope=='all' else
                               sum(max(0.,min(end,y)-max(start,x)) for x,y in active))
            total=sum(weights)
            expected=run['duration'] if scope=='all' else sum(y-x for x,y in active)
            assert math.isclose(total,expected,abs_tol=1e-8)
            for side in ('ab','ba'):
                data=np.array([float(r[f'rssi_{side}_dbm']) for r in samples[:-1]])
                times,_=np.histogram(data,bins=edges,weights=weights)
                assert math.isclose(float(sum(times)),total,abs_tol=1e-8)
                percentages=times/total*100 if total>0 else np.zeros_like(times)
                threshold=run['config'][f'radio_{side}']['sensitivity_dbm']
                above=sum(w for v,w in zip(data,weights) if v>=threshold)
                histograms[(run['name'],scope,side)]=(percentages,total)
                for i,seconds in enumerate(times):
                    records.append(dict(condition=run['name'],scope=scope,direction=side,
                        rssi_low_dbm=edges[i],rssi_high_dbm=edges[i+1],duration_s=float(seconds),
                        denominator_s=total,percentage=float(percentages[i]) if total>0 else ''))
                summaries.append(dict(condition=run['name'],scope=scope,direction=side,
                    denominator_s=total,sensitivity_dbm=threshold,above_threshold_s=above,
                    above_threshold_percentage=above/total*100 if total>0 else '',
                    mean_rssi_dbm=sum(v*w for v,w in zip(data,weights))/total if total>0 else ''))
    write_csv(output/'rssi_occupancy.csv',records,records[0].keys())
    write_csv(output/'rssi_threshold_summary.csv',summaries,summaries[0].keys())
    fig,axes=plt.subplots(4,2,figsize=(16,13),sharex=True,constrained_layout=True)
    ymax=max(10, math.ceil(max(float(max(p)) for p,_ in histograms.values())*1.1/10)*10)
    for row,run in enumerate(runs):
        for col,scope in enumerate(('all','connected_without_wait')):
            ax=axes[row,col]
            for side,color in (('ab','#b44a3e'),('ba','#2878b5')):
                percentage,total=histograms[(run['name'],scope,side)]
                frequency=float(run['config'][f'radio_{side}']['frequency_hz'])/1e9
                if total>0:
                    ax.stairs(percentage,edges,color=color,lw=1.7,label=f'{"A→B" if side=="ab" else "B→A"}：{frequency:g} GHz')
            ax.axvline(run['config']['radio_ab']['sensitivity_dbm'],color='#454545',ls='--',lw=1,label='受信閾値 −68.5 dBm')
            if total==0:ax.text(.5,.5,'接続未成立：分母なし',ha='center',transform=ax.transAxes)
            ax.set_title(f'{run["title"]} — {"全時間" if col==0 else "接続中・切断待ち除外"}（{total:.3f} s）')
            ax.set_ylim(0,ymax);ax.set_xlim(low,high);ax.grid(alpha=.2)
            ax.set_ylabel('滞在割合 [%]');ax.set_xlabel('TJX入力信号電力／RSSI [dBm]')
            if row==0:ax.legend(fontsize=9,loc='upper left' if col else 'upper right')
    fig.suptitle(f'RSSI帯ごとの滞在割合（{bin_width:g} dB刻み、方向ごとに合計100%）\n'
                 '10 msログを前値保持して時間加重。右列は接続中の状態区間だけを使用。',fontsize=14)
    save_figure(fig,output,'rssi_occupancy')
    return summaries


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    parser.add_argument('--bin-db',type=float,default=5)
    args=parser.parse_args()
    if not math.isfinite(args.bin_db) or args.bin_db<=0:
        parser.error('--bin-db は正の有限値が必要です')
    font=Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    if font.is_file():
        font_manager.fontManager.addfont(str(font))
        plt.rcParams['font.family']=font_manager.FontProperties(fname=str(font)).get_name()
    # Ubuntu 22.04のmatplotlibではTTCのType42サブセット処理が失敗するためType3を使う。
    plt.rcParams.update({'font.size':10,'axes.unicode_minus':False,'pdf.fonttype':3})
    runs=[load_run(args.directory,name,title) for name,title in zip(NAMES,TITLES)]
    output=args.directory/'protocol_analysis'
    output.mkdir(exist_ok=True)
    records=[]
    for run in runs:
        for start,end,state in run['intervals']:
            records.append(dict(condition=run['name'],state=state,start_s=start,end_s=end,duration_s=end-start))
    write_csv(output/'state_intervals.csv',records,records[0].keys())
    protocol_plots(runs,output)
    summaries=occupancy(runs,output,args.bin_db)
    lines=['プロトコル状態遷移とRSSI滞在割合','',
        '状態はevents.csvのTX/RX/CONNECTED/WAIT_STARTED/TIMEOUTから復元。10msサンプルで短い接続段階を補間していない。',
        '未受信Beaconの周期送信・待機は探索段階に集約。成功Beaconは送信開始から独立表示。',
        'IFSは実装に合わせ、直前のフレーム段階に含める。接続前のTIMEOUTは赤×、接続後の切断は青◆。直後は探索へ戻る。',
        '接続中にはDATA/DATA_ACK交換を含む。成功DATAの個別イベントは元ログで省略されているため個々の交換波形は復元しない。',
        '無応答待ち状態はフレーム失敗検出後から。500msタイマーの起点は最後の成功ACKであり、待ち状態自体は厳密に500msとは限らない。',
        '全時間と最接近付近の図は秒単位、接続手順の拡大図は成功Beacon送信開始からのµs単位。',
        '',f'RSSIは{args.bin_db:g}dB刻み（下端以上、上端未満）。値はTJX入力の信号電力。',
        '各10msサンプルの値を次サンプルまで保持し、区間の秒数を重みとして集計。最終サンプルには追加の時間を与えない。',
        '全時間は40秒を分母、接続中はCONNECTED状態の区間だけを分母とし切断待ちを除く。',
        '接続中区間にはフレーム失敗を検出するまでの短い区間も含む。最後の成功ACKで区切った転送区間とはわずかに異なる。',
        'RSSI閾値横断の時間は10msログの分解能に依存する概算。接続未成立の条件は接続中割合の分母なし（空欄）。',
        'RSSI滞在割合はパケット成功率・リンク成立確率ではない。SNRは受信判定に使用していない。','',
        '全40秒に対する受信閾値以上の時間・割合：']
    for r in summaries:
        if r['scope']=='all':
            lines.append(f"{r['condition']} / {r['direction']}: {r['above_threshold_s']:.3f} s, {r['above_threshold_percentage']:.3f}%")
    (output/'readme.txt').write_text('\n'.join(lines)+'\n')
    print('\n'.join(lines[-9:]))
    print('プロット・区間CSV・滞在割合CSVを保存:',output)


if __name__=='__main__':
    main()
