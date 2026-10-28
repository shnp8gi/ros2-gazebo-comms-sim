#!/usr/bin/env python3
"""既存ログの時間加重goodputをSNR/RSSI帯別に集計し、定義未確認のMCS表と参考比較する。"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib-drone-throughput')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np
from plot_protocol_occupancy import NAMES, TITLES, load_run, read_csv, write_csv


def residence(times, values, states, edges):
    total = np.zeros(len(edges)-1)
    waiting = total.copy()
    for a,b,x,y,state in zip(times[:-1],times[1:],values[:-1],values[1:],states):
        cuts = [0.,1.]
        if x != y:
            lo,hi = sorted((x,y))
            for edge in edges[np.searchsorted(edges,lo,'right'):np.searchsorted(edges,hi,'left')]:
                cuts.append((edge-x)/(y-x))
        cuts.sort()
        for u,v in zip(cuts,cuts[1:]):
            index = np.searchsorted(edges,x+(y-x)*(u+v)/2,'right')-1
            if not 0 <= index < len(total):
                raise ValueError('信号ビンが観測値を包含していません')
            duration = (b-a)*(v-u)
            total[index] += duration
            if state.startswith('WAIT_'):
                waiting[index] += duration
    return total, waiting


def observations(run):
    observed = {float(r['time_s']):(float(r['rssi_ab_dbm']),float(r['ab_snr_db'])) for r in run['samples']}
    for row in run['events']:
        t = float(row['observation_time_s'])
        if 0 <= t <= run['duration']:
            observed[t] = (float(row['rssi_ab_dbm']),float(row['snr_ab_db']))
    knots = np.array(sorted(observed))
    values = np.array([observed[t] for t in knots])
    times = np.unique(np.concatenate((knots,[t for t,_ in run['changes']])))
    change_times = np.array([t for t,_ in run['changes']])
    states = [run['changes'][i][1] for i in np.searchsorted(change_times,times[:-1],'right')-1]
    return times, np.column_stack([np.interp(times,knots,values[:,i]) for i in (0,1)]), states


def delivered(frames):
    pending = {}
    result = []
    for row in frames:
        index = row['beacon_index']
        if row['frame'] == 'DATA':
            pending.pop(index,None)
            if row['success'] == '1':
                pending[index] = row
        elif row['frame'] == 'DATA_ACK':
            packet = pending.pop(index,None)
            if row['success'] == '1':
                if packet is None or int(row['wire_reference']) != int(packet['wire_sequence']):
                    raise ValueError('ACKとDATAの対応が不一致です')
                result.append((float(row['frame_end_s']),int(packet['frame_bits'])))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--table', type=Path, default=Path(__file__).resolve().parents[1]/'src/comms_sim_pkg/config/MCStable.csv')
    args = parser.parse_args()
    font = Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    if font.exists():
        font_manager.fontManager.addfont(str(font))
        plt.rcParams['font.family'] = font_manager.FontProperties(fname=str(font)).get_name()
    plt.rcParams.update({'axes.unicode_minus':False,'font.size':10})
    table = np.array(sorted(tuple(map(float,row)) for row in csv.reader(args.table.open()) if row))
    def table_rate(rssi):
        idx = np.searchsorted(table[:,0],rssi,'right')-1
        return np.where(idx<0,0,table[np.maximum(idx,0),1])
    # 解析的に既知の直線で、ビン境界における時間分割を確認。
    t,w = residence([0,2],[0,2],['WAIT_DATA_ACK'],[-1,0,1,2,3])
    assert np.allclose(t,[0,1,1,0]) and np.array_equal(t,w)
    output = args.directory/'signal_throughput'
    output.mkdir(exist_ok=True)
    all_rows, totals, figures = [], [], {}
    for metric, label in [('snr','A→B SNR [dB]'),('ebno','A→B Eb/N0 [dB]'),('rssi','A→B RSSI [dBm・TJX入力]')]:
        fig, axes = plt.subplots(2,2,figsize=(12,8),constrained_layout=True)
        figures[metric] = (fig,axes,label)
    for number,(name,title) in enumerate(zip(NAMES,TITLES)):
        run = load_run(args.directory,name,title)
        frames = read_csv(args.directory/name/'frames.csv')
        acknowledgments = delivered(frames)
        summary = read_csv(args.directory/name/'summary.csv')[0]
        bit_total = sum(bits for _,bits in acknowledgments)
        assert math.isclose(bit_total/8e6,float(summary['total_data_MB']),abs_tol=1e-8)
        times,signals,states = observations(run)
        c = run['config']
        offset = 10*math.log10(float(c['radio_ab']['noise_bandwidth_hz'])/float(c['awgn']['bit_rate_bps']))
        trials = [r for r in frames if r['frame']=='DATA']
        aggregate_wait = 0
        table_integral = 0
        for metric,column,delta in [('rssi',0,0),('snr',1,0),('ebno',1,offset)]:
            values = signals[:,column]+delta
            edges = np.arange(math.floor(values.min())-1,math.ceil(values.max())+2,dtype=float)
            if metric=='rssi':
                edges=np.unique(np.concatenate((edges,table[:,0])))
            dwell,wait = residence(times,values,states,edges)
            credited = np.zeros(len(dwell),dtype=np.int64)
            sent = credited.copy(); received = credited.copy()
            for time,bits in acknowledgments:
                value = np.interp(time,times,values)
                credited[np.searchsorted(edges,value,'right')-1] += bits
            for row in trials:
                # 試行成功率はDATA開始時の信号で分類。goodputはACK完了時に分類。
                value=np.interp(float(row['frame_start_s']),times,values)
                index=np.searchsorted(edges,value,'right')-1
                sent[index]+=1;received[index]+=row['success']=='1'
            assert math.isclose(dwell.sum(),run['duration'],abs_tol=1e-8)
            assert credited.sum()==bit_total and sent.sum()==len(trials)
            mids=(edges[:-1]+edges[1:])/2
            rate=np.divide(credited,dwell,out=np.full(len(dwell),np.nan),where=dwell>0)/1e6
            for i in range(len(dwell)):
                if dwell[i] == 0 and sent[i] == 0:
                    continue
                all_rows.append(dict(condition=name,metric=metric,lower=edges[i],upper=edges[i+1],
                    dwell_s=dwell[i],wait_s=wait[i],credited_payload_bits=int(credited[i]),
                    goodput_mbps=rate[i],data_trials=int(sent[i]),data_rx_success=int(received[i]),
                    data_rx_success_ratio=received[i]/sent[i] if sent[i] else '',
                    reference_table_gbps=float(table_rate(mids[i])) if metric=='rssi' else ''))
            fig,axes,label=figures[metric];ax=axes.flat[number]
            mask=dwell>0
            ax.bar(mids[mask],rate[mask],width=np.diff(edges)[mask]*.85,color='#277da1',label='実効goodput（待機を含む）')
            if metric=='rssi':
                xmin=min(-72,edges.min());xmax=max(-48,edges.max())
                x=np.unique(np.concatenate(([xmin,xmax],table[:,0])))
                ax.step(x,table_rate(x)*1000,where='post',color='#c45c36',label='MCS表（速度の定義は未確認）')
                table_integral=float(np.sum(table_rate(mids)*dwell))
            if metric=='snr':
                aggregate_wait=float(wait.sum())
            ax.set_title(title);ax.set_xlabel(label);ax.set_ylabel('速度 [Mbps]');ax.grid(alpha=.2)
            ax.set_ylim(bottom=0)
            if metric!='rssi' and not np.any(rate[mask]>0):
                ax.set_ylim(0,1)
                ax.scatter(mids[mask],np.zeros(np.sum(mask)),s=8,color='#277da1')
            ax.text(.02,.96,f"全時間平均 {bit_total/run['duration']/1e6:.3f} Mbps\n観測した信号帯だけ棒を表示",transform=ax.transAxes,va='top',fontsize=9)
            ax.legend(loc='upper right',fontsize=8)
        totals.append(dict(condition=name,duration_s=run['duration'],data_mb=bit_total/8e6,
            average_goodput_mbps=bit_total/run['duration']/1e6,wait_s=aggregate_wait,
            wait_percent=100*aggregate_wait/run['duration'],reference_table_time_average_gbps=table_integral/run['duration'],
            ab_rssi_min_dbm=float(signals[:,0].min()),ab_rssi_max_dbm=float(signals[:,0].max()),
            ab_snr_min_db=float(signals[:,1].min()),ab_snr_max_db=float(signals[:,1].max())))
    write_csv(output/'signal_bins.csv',all_rows,list(all_rows[0]))
    write_csv(output/'comparison.csv',totals,list(totals[0]))
    for metric,(fig,axes,label) in figures.items():
        fig.suptitle('単一走行の信号帯別スループット：各1.1秒・固定QPSK・payload FECなし・500ms待機あり\nACK完了時に受領bitを計上／各信号帯の全滞在時間で除算',fontsize=13)
        fig.savefig(output/f'{metric}_goodput.png',dpi=170)
        fig.savefig(output/f'{metric}_goodput.pdf')
        plt.close(fig)
    (output/'validation.json').write_text(json.dumps(dict(duration_conservation=True,payload_conservation=True,
        data_ack_reference_match=True,data_trial_count_match=True,linear_crossing_fixture=True,
        reference_table=str(args.table.resolve()),reference_rate_definition='unknown',single_seed=True),indent=2)+'\n')
    print(json.dumps(totals,ensure_ascii=False,indent=2))
    print('保存:',output)


if __name__=='__main__':
    main()
