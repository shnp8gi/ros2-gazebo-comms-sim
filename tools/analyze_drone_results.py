#!/usr/bin/env python3
"""失敗時と判定時を区別したタイムアウト一覧・条件表・時系列図を保存。"""
import argparse
import csv
from pathlib import Path
import yaml

SNAPSHOT = ('observation_time_s', 'distance_m', 'rssi_ab_dbm', 'rssi_ba_dbm',
            'margin_ab_db', 'margin_ba_db', 'off_axis_a_deg', 'off_axis_b_deg',
            'snr_ab_db', 'snr_ba_db', 'ab_reachable', 'ba_reachable')


def write_rows(path, rows, fields):
    with path.open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def analyze(directory, plot=True):
    directory = Path(directory)
    if not (directory / 'completed').is_file():
        raise ValueError('未完了の実験は集計できません')
    c = yaml.safe_load((directory / 'config.yaml').read_text())
    conditions = {'scenario': c['scenario'], 'antenna_mode': c['antenna_mode'],
                  'speed_a_kmh': c['drone_a']['speed_kmh'], 'speed_b_kmh': c['drone_b']['speed_kmh'],
                  'altitude_a_m': c['drone_a']['altitude_m'], 'altitude_b_m': c['drone_b']['altitude_m'],
                  'height_difference_m': c['drone_b']['altitude_m']-c['drone_a']['altitude_m'],
                  'lateral_separation_m': c['lateral_separation_m'] if c['scenario']=='parallel' else 0,
                  'duration_s': c['duration_s'], 'encounter_time_s': c['encounter_time_s']}
    for side in ('ab','ba'):
        conditions.update({f'{key}_{side}': value for key,value in c[f'radio_{side}'].items()})
    conditions.update(c['protocol'])
    conditions['phy_error_model'] = c.get('phy_error_model', 'legacy')
    conditions.update({f'lab_phy_{key}':value for key,value in c.get('lab_phy',{}).items()})
    conditions.update({f'awgn_{key}':value for key,value in c.get('awgn',{}).items()})
    write_rows(directory/'conditions.csv', [conditions], conditions.keys())
    with (directory/'events.csv').open() as stream:
        events = list(csv.DictReader(stream))
    # 成功した接続ごとに、探索待ちとハンドシェイク自体の所要時間を分離する。
    connections = []
    setups = {}
    requests = {}
    for e in events:
        # 新ログは並行Beaconを番号で分離。旧ログには番号がないので従来どおり一系列。
        index = e.get('beacon_index', '')
        if e['event']=='TX' and e['frame']=='BEACON':
            setups[index] = {'beacon_tx_s': float(e['time_s'])}
        setup = setups.setdefault(index, {})
        for event, frame, key in (('RX_OK','BEACON','beacon_rx_s'),
                                  ('TX','REQUEST','request_tx_s'),
                                  ('RX_OK','RESPONSE','response_rx_s'),
                                  ('RX_OK','ACK','ack_rx_s')):
            if e['event']==event and e['frame']==frame:
                setup[key] = float(e['time_s'])
        if e['event']=='CONNECTED':
            required = ('beacon_tx_s','beacon_rx_s','request_tx_s','response_rx_s','ack_rx_s')
            if any(key not in setup for key in required):
                raise ValueError('接続成立に対応するハンドシェイク時刻が不足しています')
            connected = float(e['time_s'])
            connections.append({'connection_index':len(connections)+1, 'beacon_index':index, **setup,
                                'connected_s':connected,
                                'setup_ms':(connected-setup['beacon_tx_s'])*1000,
                                'after_beacon_rx_ms':(connected-setup['beacon_rx_s'])*1000,
                                'since_sim_start_s':connected})
        if index:
            if e['event']=='TX' and e['frame']=='REQUEST':
                requests[index] = {'beacon_index':index, 'beacon_tx_s':setup['beacon_tx_s'],
                                   'beacon_rx_s':setup['beacon_rx_s'], 'request_tx_s':e['time_s']}
            trace = requests.get(index)
            if trace is not None:
                if e['frame']=='REQUEST' and e['event'] in ('RX_OK','RX_FAILED'):
                    trace.update(request_result=e['event'],request_failure_reason=e['reason'])
                    if e['event']=='RX_OK': trace['request_rx_s']=e['time_s']
                if e['event']=='RX_OK' and e['frame'] in ('RESPONSE','ACK'):
                    trace[e['frame'].lower()+'_rx_s']=e['time_s']
                if e['event']=='CONNECTED': trace['connected_s']=e['time_s']
                if e['event']=='TIMEOUT' and e['reason']!='INACTIVITY_TIMEOUT':
                    trace.update(timeout_s=e['time_s'],timeout_reason=e['reason'])
    connection_fields = ('connection_index','beacon_index','beacon_tx_s','beacon_rx_s','request_tx_s',
                         'response_rx_s','ack_rx_s','connected_s','setup_ms',
                         'after_beacon_rx_ms','since_sim_start_s')
    write_rows(directory/'connections.csv', connections, connection_fields)
    write_rows(directory/'beacon_requests.csv', requests.values(),
               ('beacon_index','beacon_tx_s','beacon_rx_s','request_tx_s','request_rx_s',
                'request_result','request_failure_reason','response_rx_s','ack_rx_s',
                'connected_s','timeout_s','timeout_reason'))
    failures = {}
    timeouts, disconnections = [], []
    pending = None
    counts = {f'{name.lower()}_count': 0 for name in ('RESPONSE_TIMEOUT','ACK_TIMEOUT')}
    counts['disconnection_count'] = 0
    counts['beacon_rx_failures'] = 0
    counts['beacon_ignored_count'] = sum(e['event']=='BEACON_IGNORED' for e in events)
    counts['half_duplex_rx_failures'] = sum(e['event']=='RX_FAILED' and e['reason']=='RECEIVER_TRANSMITTING' for e in events)
    for e in events:
        if e['event']=='RX_FAILED':
            failures[(e.get('beacon_index',''),e['frame'],e['failure_sample_s'])] = e
            if e['frame']=='BEACON': counts['beacon_rx_failures'] += 1
        elif e['event']=='WAIT_STARTED':
            pending = e
        elif e['event']=='TIMEOUT' and e['reason']=='INACTIVITY_TIMEOUT':
            # 旧ログは同時刻にTIMEOUTとDISCONNECTEDを出力。切断側だけを採用する。
            continue
        elif e['event'] in ('TIMEOUT','DISCONNECTED'):
            is_disconnect = e['event']=='DISCONNECTED'
            failure = failures.get((e.get('beacon_index',''),e['frame'],e['failure_sample_s']))
            if failure is None: raise ValueError('待ち終了に対応する失敗イベントがありません')
            counts['disconnection_count' if is_disconnect else e['reason'].lower()+'_count'] += 1
            row = {'scenario':c['scenario'],'antenna_mode':c['antenna_mode'],'beacon_index':e.get('beacon_index',''),
                   'type':e['reason'],'failed_frame':failure['frame'],'failed_direction':failure['direction'],
                   'failure_reason':failure['reason'],'wait_start_s':e['wait_start_s'],
                   'failure_detected_s':failure['time_s'],'failure_sample_s':failure['failure_sample_s'],
                   'timeout_s':e['time_s'],'wait_duration_ms':(float(e['time_s'])-float(e['wait_start_s']))*1000,
                   'failure_relative_to_encounter_s':float(failure['failure_sample_s'])-c['encounter_time_s']}
            for prefix, source in (('failure',failure),('timeout',e)):
                row.update({f'{prefix}_{key}':source.get(key, '') for key in SNAPSHOT})
            row['both_directions_reachable_at_timeout'] = (
                int(e['ab_reachable']=='1' and e['ba_reachable']=='1') if 'ab_reachable' in e else
                int(float(e['margin_ab_db'])>=0 and float(e['margin_ba_db'])>=0))
            if is_disconnect:
                row = {key.replace('timeout','disconnect'):value for key,value in row.items()}
                row['type'] = 'INACTIVITY_DISCONNECT'
                disconnections.append(row)
            else:
                timeouts.append(row)
            pending = None
    fields = ['scenario','antenna_mode','beacon_index','type','failed_frame','failed_direction','failure_reason','wait_start_s',
              'failure_detected_s','failure_sample_s','timeout_s','wait_duration_ms','failure_relative_to_encounter_s']
    fields += [f'{prefix}_{key}' for prefix in ('failure','timeout') for key in SNAPSHOT]
    fields += ['both_directions_reachable_at_timeout']
    write_rows(directory/'timeouts.csv',timeouts,fields)
    write_rows(directory/'disconnections.csv',disconnections,[key.replace('timeout','disconnect') for key in fields])
    # 終了時点で500ms未経過の待ちは、タイムアウトなしとは区別する。
    write_rows(directory/'pending_waits.csv', [pending] if pending else [], events[0].keys())
    counts['pending_waits_at_end'] = int(pending is not None)
    with (directory/'summary.csv').open() as stream:
        summary=next(csv.DictReader(stream))
    if len(connections) != int(summary['connections']):
        raise ValueError('接続確立時間の件数とサマリーが一致しません')
    if connections and abs(connections[-1]['setup_ms']-float(summary['last_setup_ms']))>2e-6:
        raise ValueError('イベント列とサマリーの接続確立時間が一致しません')
    if 'disconnections' in summary:
        if len(timeouts)!=int(summary['timeouts']) or len(disconnections)!=int(summary['disconnections']):
            raise ValueError('接続前タイムアウト/接続後切断とサマリーが一致しません')
    elif len(timeouts)+len(disconnections)!=int(summary['timeouts']):
        raise ValueError('旧サマリーのタイムアウト合計と再分類結果が一致しません')
    classified = dict(summary, timeouts=len(timeouts), disconnections=len(disconnections),
                      timeout_counting_revision='setup_only_v2')
    write_rows(directory/'classification_summary.csv',[classified],classified.keys())
    counts.update({'attempts':int(summary['attempts']),'connections':int(summary['connections']),
                   'total_data_MB':float(summary['total_data_MB']),
                   'timeouts':len(timeouts),'disconnections':len(disconnections)})
    counts.update({'mean_setup_ms':sum(r['setup_ms'] for r in connections)/len(connections) if connections else '',
                   'first_connection_time_s':connections[0]['connected_s'] if connections else ''})
    write_rows(directory/'timeout_counts.csv',[counts],counts.keys())
    if plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        with (directory/'timeseries.csv').open() as stream:
            samples=list(csv.DictReader(stream))
        t=[float(r['time_s'])-c['encounter_time_s'] for r in samples]
        fig, axes=plt.subplots(3,1,sharex=True,figsize=(11,8),constrained_layout=True)
        for side, direction in (('ab','A -> B'),('ba','B -> A')):
            label = f"{direction}: {float(c[f'radio_{side}']['frequency_hz']) / 1e9:g} GHz"
            axes[0].plot(t,[float(r[f'rssi_{side}_dbm']) for r in samples],label=label)
            axes[0].axhline(c[f'radio_{side}']['sensitivity_dbm'],color='gray',ls='--',lw=.8)
        for side in ('a','b'):
            axes[1].plot(t,[float(r[f'off_axis_{side}_deg']) for r in samples],label=f'Drone {side.upper()}')
        axes[1].set_ylim(0, 180)  # 理想追従の丸め誤差を実際の振動として拡大しない。
        axes[2].plot(t,[float(r['total_data_MB']) for r in samples],color='green',label='ACK-confirmed payload')
        for group,field,color,label in ((timeouts,'timeout_s','red','Setup timeout'),
                                        (disconnections,'disconnect_s','blue','Disconnection')):
            for i,row in enumerate(group):
                failure=float(row['failure_sample_s'])-c['encounter_time_s']
                end=float(row[field])-c['encounter_time_s']
                for ax in axes:
                    ax.axvline(failure,color='orange',ls=':',alpha=.6)
                    ax.axvline(end,color=color,ls='--',alpha=.6,label=label if ax is axes[0] and i==0 else None)
        for ax,label in zip(axes,('RSSI (dBm)','Boresight error (deg)','Delivered payload (MB)')):
            ax.set_ylabel(label);ax.grid(alpha=.2);ax.legend(loc='best')
        axes[0].set_title(f"{c['scenario']} / {c['antenna_mode']} | A={c['drone_a']['speed_kmh']} km/h, B={c['drone_b']['speed_kmh']} km/h")
        axes[-1].set_xlabel('Time relative to closest approach (s)')
        fig.savefig(directory/'timeline.png',dpi=150)
        plt.close(fig)
    print(f'原因解析を保存: {directory.name} / timeout={len(timeouts)}, disconnection={len(disconnections)}, pending={counts["pending_waits_at_end"]}',flush=True)
    return counts


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directories',nargs='+')
    parser.add_argument('--no-plot',action='store_true')
    args=parser.parse_args()
    for directory in args.directories: analyze(directory,not args.no_plot)
