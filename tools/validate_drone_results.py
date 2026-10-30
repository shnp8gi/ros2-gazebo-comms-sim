#!/usr/bin/env python3
"""実Gazebo位置、アンテナ向き、CSV集計と正常終了を検証する。"""
import csv
import math
from pathlib import Path
import yaml


def validate(directory):
    directory = Path(directory)
    if not (directory / 'completed').is_file():
        raise ValueError(f'完了マーカーがありません: {directory}')
    c = yaml.safe_load((directory / 'config.yaml').read_text())
    count = 0
    max_error = 0.0
    last_time = -1
    for row in csv.DictReader((directory / 'gazebo_poses.csv').open()):
        t = float(row['time_s'])
        last_time = max(last_time, t)
        dt = t - c['encounter_time_s']
        a = (c['drone_a']['speed_kmh'] / 3.6 * dt, 0, c['drone_a']['altitude_m'])
        b = ((-c['drone_b']['speed_kmh'] / 3.6 * dt, c['lateral_separation_m'], c['drone_b']['altitude_m'])
             if c['scenario'] == 'parallel' else
             (0, c['drone_b']['speed_kmh'] / 3.6 * dt, c['drone_b']['altitude_m']))
        is_a = row['model'].endswith('_a')
        expected = a if is_a else b
        actual = tuple(float(row[key]) for key in ('x_m', 'y_m', 'z_m'))
        max_error = max(max_error, math.dist(actual, expected))
        pitch, yaw = float(row['pitch_rad']), float(row['yaw_rad'])
        direction = (math.cos(pitch)*math.cos(yaw), math.cos(pitch)*math.sin(yaw), -math.sin(pitch))
        if row['model'].startswith('antenna') and c['antenna_mode'] == 'tracking':
            target = b if is_a else a
            delta = tuple(target[i]-expected[i] for i in range(3))
            length = math.sqrt(sum(v*v for v in delta))
            goal = tuple(v/length for v in delta)
        else:
            elevation = (c['drone_a' if is_a else 'drone_b']['fixed_elevation_deg']
                         if row['model'].startswith('antenna') else 0)
            heading = 0 if is_a else (math.pi if c['scenario'] == 'parallel' else math.pi/2)
            if row['model'].startswith('antenna') and c['scenario'] == 'parallel':
                heading = math.pi/2 if is_a else -math.pi/2
            elif (row['model'].startswith('antenna') and c['scenario'] == 'cross'
                  and c.get('geometry_revision') == 'both_scenarios_sideways_fixed_v3'):
                heading = math.pi/2 if is_a else 0
            angle = math.radians(elevation)
            goal = (math.cos(angle)*math.cos(heading), math.cos(angle)*math.sin(heading), math.sin(angle))
        if math.dist(direction, goal) > 1e-6:
            raise ValueError(f'Gazeboアンテナ/機首方向が不一致: {row}')
        count += 1
    if max_error > 1e-6 or count < 4 or abs(last_time-c['duration_s']) > 1e-7:
        raise ValueError(f'Gazebo位置/終端時刻が不一致: error={max_error}, t={last_time}')
    summary = next(csv.DictReader((directory / 'summary.csv').open()))
    rows = list(csv.DictReader((directory / 'timeseries.csv').open()))
    if abs(float(rows[-1]['time_s'])-c['duration_s']) > 1e-7:
        raise ValueError('時系列が完了時刻までありません')
    if abs(float(rows[-1]['total_data_MB'])-float(summary['total_data_MB'])) > 1e-7:
        raise ValueError('時系列とサマリーの転送量が不一致')
    if any(float(b['total_data_MB']) < float(a['total_data_MB']) for a,b in zip(rows,rows[1:])):
        raise ValueError('累積転送量が減少しています')
    events = csv.DictReader((directory / 'events.csv').open())
    connections = 0
    timeouts = 0
    timeout_records = []
    disconnections = 0
    expected_direction = {'BEACON':'B_TO_A','REQUEST':'A_TO_B','RESPONSE':'B_TO_A','ACK':'A_TO_B'}
    expected_direction['DATA'] = 'B_TO_A' if c['protocol'].get('data_from_prc',False) else 'A_TO_B'
    expected_direction['DATA_ACK'] = 'A_TO_B' if c['protocol'].get('data_from_prc',False) else 'B_TO_A'
    hrcp = c.get('hrcp', {}).get('enabled', False)
    received = {}
    hrcp_received = set()
    for e in events:
        connections += e['event'] == 'CONNECTED'
        timeouts += e['event'] == 'TIMEOUT'
        disconnections += e['event'] == 'DISCONNECTED'
        if e['event']=='TIMEOUT':
            timeout_records.append({key:e.get(key,'') for key in ['time_s','frame','direction','reason']})
        # HRCPの本文なしDATA_ACKは、送信側のRIFS回復でも使用する。
        if e['frame'] in expected_direction and not (hrcp and e['frame']=='DATA_ACK') and e['direction'] != expected_direction[e['frame']]:
            raise ValueError('制御フレームの帯域方向が不一致')
        sequence = []
        if not hrcp and e['event'] in ('RX_OK', 'CONNECTED'):
            sequence = received.setdefault(e.get('beacon_index',''), [])
            if e['event'] == 'RX_OK': sequence.append(e['frame'])
        if hrcp and e['event'] == 'RX_OK': hrcp_received.add(e['frame'])
        if hrcp and e['event'] == 'CONNECTED':
            if not {'BEACON','REQUEST','RESPONSE'}.issubset(hrcp_received) or not ({'ACK','DATA'} & hrcp_received):
                raise ValueError('HRCP接続手順が不一致')
            hrcp_received.clear()
        if not hrcp and e['event'] == 'CONNECTED' and sequence[-4:] != ['BEACON','REQUEST','RESPONSE','ACK']:
            raise ValueError('接続手順が不一致')
    if connections != int(summary['connections']):
        raise ValueError('接続回数が不一致')
    if timeouts != int(summary['timeouts']) or disconnections != int(summary['disconnections']):
        raise ValueError('Timeoutまたは切断のイベント数とサマリーが不一致')
    first = float(summary['first_connected_s'])
    if any(float(r['total_data_MB']) > 0 and (first < 0 or float(r['time_s']) <= first) for r in rows):
        raise ValueError('接続前に転送量が加算されています')
    temp=directory/'.timeout_events_validator.tmp'
    with temp.open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=['time_s','frame','direction','reason'])
        writer.writeheader();writer.writerows(timeout_records)
    temp.replace(directory/'timeout_events.csv')
    print(f"検証成功: {directory.name}, {summary['total_data_MB']} MB, 位置誤差最大 {max_error:.3g} m", flush=True)
    return summary


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directories', nargs='+')
    for path in parser.parse_args().directories:
        validate(path)
