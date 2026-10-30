"""80条件のTimeout比較図と、接続表示とは別の配送時刻表を作る。"""
import argparse
import csv
import json
from pathlib import Path
from run_probability_campaign import plot, PATTERNS


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    args=parser.parse_args();root=args.directory.resolve()
    index=json.loads((root/'index.json').read_text())
    assert index['status']=='complete' and len(index['runs'])==80
    assert all(r['status']=='complete' for r in index['runs'])
    output=root/'analysis' if index.get('layout') else root
    deliveries=[]
    for case in index['runs']:
        previous=0.;first=None;last=None;latest=None
        directory=Path(case['output'])
        with (directory/'timeseries.csv').open() as stream:
            for row in csv.DictReader(stream):
                amount=float(row['total_data_MB']);seconds=float(row['time_s'])
                if amount>previous:
                    if first is None:first=seconds
                    last=seconds
                previous=amount;latest=row
        assert abs(previous-case['total_data_MB'])<1e-6
        with (directory/'summary.csv').open() as stream:
            raw=next(csv.DictReader(stream))
        with (directory/'timeout_events.csv').open() as stream:
            events=list(csv.DictReader(stream))
        total=int(raw['timeouts'])
        assert len(events)==total
        connected=float(raw['first_connected_s'])
        before=sum(connected<0 or float(e['time_s'])<connected for e in events)
        case.update(timeouts=before,timeouts_before_first_connection=before,timeouts_total=total,
                    timeouts_after_first_connection=total-before,connection_established=connected>=0)
        deliveries.append(dict(model=case['model'],scenario=case['scenario'],antenna_mode=case['antenna_mode'],
            speed_mps=case['speed_mps'],observation_duration_s=case['duration_s'],
            first_delivery_log_s=first,last_delivery_log_s=last,delivery_timing_resolution_s=.01,
            connected_at_end=bool(int(latest['connected'])),connections=case['connections'],
            disconnections=case['disconnections'],timeouts=before,timeouts_total=total,
            connection_established=connected>=0,attempts=case['attempts'],
            total_data_MB=case['total_data_MB'],
            accounting='ACK_confirmed' if case['model']=='threshold' else 'unique_payload_received',
            replay=case['replay']))
    with (output/'delivery_windows.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(deliveries[0]));writer.writeheader();writer.writerows(deliveries)
    lookup={(r['model'],r['scenario'],r['antenna_mode'],r['speed_mps']):r['timeouts'] for r in index['runs']}
    keys=[(m,s,a) for m in ['threshold','statistical'] for s,a in PATTERNS]
    with (output/'timeouts_by_speed.csv').open('w',newline='') as stream:
        writer=csv.writer(stream)
        writer.writerow(['speed_mps']+[f'{m}_{s}_{a}' for m,s,a in keys])
        for speed in range(1,11):writer.writerow([speed]+[lookup[m,s,a,speed] for m,s,a in keys])
    index['timeout_definition']='TIMEOUT events strictly before first CONNECTED; if never connected, entire observation; raw per-run summary retains total'
    temporary=root/'index.json.tmp'
    temporary.write_text(json.dumps(index,ensure_ascii=False,indent=2));temporary.replace(root/'index.json')
    fields=list(dict.fromkeys(k for r in index['runs'] for k in r))
    with (output/'summary.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows(index['runs'])
    plot(output,index['runs'])
    print('80条件のTimeout時刻索引と配送量を照合し、比較図・速度別表・配送時刻表を保存しました')


if __name__=='__main__':main()
