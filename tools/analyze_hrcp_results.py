#!/usr/bin/env python3
"""連続HRCP受信機と独立端点MACのログを集計する。BにBeacon番号の知識を仮定しない。"""
import csv
from pathlib import Path


def read(path):
    with path.open() as f:
        return list(csv.DictReader(f))


def write(path, rows, fields):
    with path.open('w') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def analyze(directory, config):
    directory = Path(directory)
    events = read(directory / 'events.csv')
    frames = read(directory / 'frames.csv')
    summary = read(directory / 'summary.csv')[0]
    beacons = {}
    connections, requests, expiries = [], [], []
    attempt = None
    for e in events:
        t = float(e['time_s'])
        if e['event'] == 'TX' and e['frame'] == 'BEACON':
            beacons[e['beacon_index']] = t
        if e['event'] == 'ATTEMPT':
            attempt = {'first_request_tx_s': t, 'beacon_index': e['beacon_index'],
                       'beacon_tx_s': beacons.get(e['beacon_index'], '')}
        if e['event'] == 'TX' and e['frame'] == 'REQUEST':
            requests.append({'beacon_index': e['beacon_index'], 'request_tx_s': t,
                             'beacon_tx_s': beacons.get(e['beacon_index'], ''),
                             'attempt_first_request_s': attempt['first_request_tx_s'] if attempt else ''})
        if e['event'] == 'CONNECTED':
            if attempt is None:
                raise ValueError('初回Requestのない接続イベントです')
            row = dict(attempt, connection_index=len(connections)+1, connected_s=t,
                       setup_from_first_request_ms=(t-attempt['first_request_tx_s'])*1000)
            row['setup_from_beacon_tx_ms'] = ((t-attempt['beacon_tx_s'])*1000
                                             if attempt['beacon_tx_s'] != '' else '')
            connections.append(row)
        if e['event'] in ('TIMEOUT', 'ATP_DISCONNECT', 'DISCONNECTED'):
            expiries.append({k:e[k] for k in ('time_s','event','direction','reason','wait_start_s','deadline_s')})
    if len(connections) != int(summary['connections']):
        raise ValueError('接続回数が不一致です')
    if connections and abs(connections[-1]['setup_from_first_request_ms']-float(summary['last_setup_ms'])) > 2e-6:
        raise ValueError('初回Requestからの確立時間が不一致です')
    timeouts = sum(e['event']=='TIMEOUT' for e in events)
    disconnections = sum(e['event']=='DISCONNECTED' for e in events)
    if timeouts != int(summary['timeouts']) or disconnections != int(summary['disconnections']):
        raise ValueError('ATPと切断の集計が不一致です')
    for f in frames:
        if f['phy_model'] != 'hrcp_iq_continuous_ldpc_v1':
            raise ValueError('異なるPHYの結果が混在しています')
        start, size, arrival = map(int, (f['tx_start_sample'], f['tx_samples'], f['rx_start_sample']))
        if arrival < start or abs(float(f['frame_end_s'])-(arrival+size)/3.52e9)>1e-12:
            raise ValueError('伝搬とフレーム終了sampleが不一致です')
    if len(frames) != int(summary['awgn_trials']):
        raise ValueError('フレーム観測数が不一致です')
    write(directory/'connections.csv', connections,
          ('connection_index','beacon_index','beacon_tx_s','first_request_tx_s','connected_s',
           'setup_from_first_request_ms','setup_from_beacon_tx_ms'))
    write(directory/'beacon_requests.csv', requests,
          ('beacon_index','beacon_tx_s','request_tx_s','attempt_first_request_s'))
    write(directory/'endpoint_expiries.csv', expiries,
          ('time_s','event','direction','reason','wait_start_s','deadline_s'))
    conditions = {'scenario':config['scenario'],'antenna_mode':config['antenna_mode'],
                  'duration_s':config['duration_s'], 'phy_error_model':config['phy_error_model']}
    for section in ('hrcp','radio_ab','radio_ba','drone_a','drone_b'):
        conditions.update({f'{section}_{k}':v for k,v in config[section].items()})
    write(directory/'conditions.csv',[conditions],conditions.keys())
    result = {'timeouts':timeouts,'disconnections':disconnections,
              'mean_setup_ms':sum(r['setup_from_first_request_ms'] for r in connections)/len(connections) if connections else '',
              'beacon_tx_count':sum(e['event']=='TX' and e['frame']=='BEACON' for e in events),
              'request_tx_count':len(requests),
              'rifs_retries':sum(e['event']=='RIFS_RETRY' for e in events),
              'waveform_frames':len(frames),
              'waveform_successes':sum(f['success']=='1' for f in frames)}
    write(directory/'hrcp_counts.csv',[result],result.keys())
    return result
