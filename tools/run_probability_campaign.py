"""①/②×4飛行条件×1〜10m/sを実行し、再生とTimeout比較を保存する。"""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import copy
import csv
from datetime import datetime
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import threading
import time
import uuid
import yaml

ROOT = Path(__file__).resolve().parents[1]
PATTERNS = [('parallel','fixed'), ('parallel','tracking'), ('cross','fixed'), ('cross','tracking')]
BASE = {'threshold':ROOT/'src/comms_sim_pkg/config/drone_threshold_parallel_1mps.yaml',
        'statistical':ROOT/'src/comms_sim_pkg/config/drone_statistical_measured_provisional.yaml'}


def container_path(path):
    return '/workspace/' + str(path.resolve().relative_to(ROOT))


def make_config(model, scenario, mode, speed):
    source = BASE[model]
    c = yaml.safe_load(source.read_text())
    for field in ['e_plane','h_plane']:
        c['antenna'][field] = container_path(source.parent/c['antenna'][field])
    c['mcs_table'] = container_path(source.parent/c['mcs_table'])
    if model == 'statistical':
        c['empirical_phy']['table'] = container_path(source.parent/c['empirical_phy']['table'])
        assert c['empirical_phy']['control_mcs'] == 3
    c['scenario'], c['antenna_mode'] = scenario, mode
    c['drone_a']['speed_kmh'] = c['drone_b']['speed_kmh'] = speed*3.6
    c['duration_s'], c['encounter_time_s'] = 50/speed, 25/speed
    c['campaign_note'] = '各機50mの同じ経路。Timeoutは接続前TIMEOUTイベント数。1条件1回、①②のMACは異なる。'
    return c


def plot(root, rows):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.ticker import MaxNLocator
    plt.rcParams['font.family'] = 'Noto Sans CJK JP'
    fig, ax = plt.subplots(figsize=(12,7), layout='constrained')
    names = {('parallel','fixed'):'並行・固定',('parallel','tracking'):'並行・追従',
             ('cross','fixed'):'十字・固定',('cross','tracking'):'十字・追従'}
    for idx, (scenario, mode) in enumerate(PATTERNS):
        for model, prefix, style, marker in [('threshold','① 閾値','-','o'),('statistical','② 実測PER＋仮定','--','x')]:
            subset = sorted([r for r in rows if r['model']==model and r['scenario']==scenario and r['antenna_mode']==mode], key=lambda r:r['speed_mps'])
            ax.plot([r['speed_mps'] for r in subset], [r['timeouts'] for r in subset],
                    color=f'C{idx}', linestyle=style, marker=marker, label=f'{prefix} / {names[scenario,mode]}')
    ax.set(xlabel='各機の飛行速度 [m/s]', ylabel='初回リンク確立までのTimeout回数',
           title='①・② × 4シナリオ：飛行速度とTimeout（各条件1回）', xticks=range(1,11), xlim=(.7,10.3))
    ax.yaxis.set_major_locator(MaxNLocator(integer=True))
    ax.set_ylim(bottom=-.2)
    ax.grid(True, alpha=.25)
    ax.legend(ncol=2, fontsize=9)
    fig.supxlabel('各機50 m／観測時間50÷速度 秒。初回確立後の再接続Timeoutは除外。①②はMAC処理も異なる。\n未接続では観測終了までを集計。0回でも未試行の場合があるため、接続成否も確認。',fontsize=10)
    for ext in ['png','pdf','svg']:
        fig.savefig(root/f'timeout_comparison.{ext}', dpi=180)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--jobs',type=int,default=4)
    parser.add_argument('--resume',action='store_true')
    args=parser.parse_args()
    if not 1<=args.jobs<=8:parser.error('jobsは1〜8')
    root=args.output.resolve()
    root.relative_to(ROOT)
    if args.resume and (root/'index.json').exists():
        saved=json.loads((root/'index.json').read_text())
        if saved.get('layout')=='model/scenario/speed' and saved.get('status')=='complete':
            print('全80条件が完了・整理済みです:',root);return
    root.mkdir(parents=True,exist_ok=args.resume)
    (root/'configs').mkdir(exist_ok=args.resume)
    (root/'logs').mkdir(exist_ok=args.resume)
    index_path=root/'index.json'
    if args.resume:
        index=json.loads(index_path.read_text())
        if any(r['status']=='running' for r in index['runs']):
            raise ValueError('running状態が残っています。元プロセスの終了と状態を確認してから再開してください')
    else:
        index=dict(schema='probability_campaign_v1',started_at=datetime.now().astimezone().isoformat(),
                   status='running',jobs=args.jobs,distance_per_drone_m=50,
                   timeout_definition='TIMEOUT events before first CONNECTED; never-connected runs count entire observation',
                   repetitions=1,statistical_seed=20261029,runs=[])
        for speed in range(10,0,-1):
            for scenario,mode in PATTERNS:
                for model in BASE:
                    index['runs'].append(dict(name=f'{model}_{scenario}_{mode}_{speed}mps',model=model,
                        scenario=scenario,antenna_mode=mode,speed_mps=speed,duration_s=50/speed,status='pending'))
    lock=threading.Lock()
    def save():
        temp=index_path.with_suffix('.json.tmp')
        temp.write_text(json.dumps(index,ensure_ascii=False,indent=2))
        temp.replace(index_path)
    def update(row,**data):
        with lock:row.update(data);save()
    save()

    def run(row):
        name=row['name'];start=time.monotonic();output=root/name
        try:
            if row['status']=='complete':return
            update(row,status='running',started_at=datetime.now().astimezone().isoformat())
            config=make_config(row['model'],row['scenario'],row['antenna_mode'],row['speed_mps'])
            config_path=root/'configs'/f'{name}.yaml'
            if not (output/'completed').exists():
                if output.exists() and any(output.iterdir()):
                    raise ValueError('未完了の出力が存在します。上書きしません')
                config_path.write_text(yaml.safe_dump(config,allow_unicode=True,sort_keys=False))
                command='source /opt/ros/humble/setup.bash && source install/no_expected_rssi/setup.bash && '+shlex.join([
                    'ros2','launch','comms_sim_pkg','sim_launch.py','headless:=true','bridge:=false',
                    'config_file:='+container_path(config_path),'output_dir:='+container_path(output)])
                print('開始:',name,flush=True)
                with (root/'logs'/f'{name}.launch.log').open('w') as log:
                    subprocess.run(['docker','run','--rm','--user',f'{os.getuid()}:{os.getgid()}',
                        '-e','HOME=/tmp','-e','DISPLAY=','-e','OMP_WAIT_POLICY=PASSIVE',
                        '-e','GZ_PARTITION=probability-'+uuid.uuid4().hex,
                        '-v',f'{ROOT}:/workspace','-w','/workspace','ros2-gazebo-comms-sim:humble-harmonic',
                        'bash','-c',command],stdout=log,stderr=subprocess.STDOUT,check=True)
                if not (output/'completed').exists():raise ValueError('完了マーカーなし')
                (output/'runtime_seconds.txt').write_text(f'{time.monotonic()-start:.3f}\n')
            with (root/'logs'/f'{name}.verify.log').open('w') as log:
                subprocess.run([sys.executable,str(ROOT/'tools/validate_drone_results.py'),str(output)],stdout=log,stderr=subprocess.STDOUT,check=True)
                if row['model']=='statistical':
                    subprocess.run([sys.executable,str(ROOT/'tools/analyze_statistical_results.py'),str(output)],stdout=log,stderr=subprocess.STDOUT,check=True)
            snapshot=json.loads((output/'live_signals.json').read_text())
            assert snapshot['complete']
            assert abs((snapshot['sample']+1)/snapshot['sample_rate_hz']-row['duration_s'])<1e-7
            assert (output/'monitor_replay.jsonl').stat().st_size>0
            summary=next(csv.DictReader((output/'summary.csv').open()))
            assert snapshot['timeouts']==int(summary['timeouts'])
            first_connected=float(summary['first_connected_s'])
            with (output/'timeout_events.csv').open() as stream:
                initial_timeouts=sum(first_connected<0 or float(e['time_s'])<first_connected for e in csv.DictReader(stream))
            replay='#!/usr/bin/env bash\nset -e\nexec '+shlex.join([sys.executable,str(ROOT/'tools/monitor_hrcp_signals.py'),
                str(output/'live_signals.json'),'--replay','--autoplay','--window','full','--details','--playback-speed','2'])+'\n'
            (output/'replay.sh').write_text(replay);(output/'replay.sh').chmod(0o755)
            # 終端でまだ接続中の場合、観測区間内では切断していないと明示する。
            last=None
            for sample in csv.DictReader((output/'timeseries.csv').open()):last=sample
            update(row,status='complete',timeouts=initial_timeouts,
                timeouts_before_first_connection=initial_timeouts,timeouts_total=int(summary['timeouts']),
                timeouts_after_first_connection=int(summary['timeouts'])-initial_timeouts,
                connection_established=first_connected>=0,connections=int(summary['connections']),
                disconnections=int(summary['disconnections']),attempts=int(summary['attempts']),
                total_data_MB=float(summary['total_data_MB']),first_connected_s=float(summary['first_connected_s']),
                connected_at_end=bool(int(last['connected'])),wall_time_s=time.monotonic()-start,
                output=str(output),replay=str(output/'replay.sh'))
            print(f"完了: {name} Timeout={summary['timeouts']} 配送={float(summary['total_data_MB']):.3f} MB",flush=True)
        except Exception as exc:
            update(row,status='failed',error=str(exc),wall_time_s=time.monotonic()-start)
            print('失敗:',name,str(exc),flush=True)

    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        futures=[pool.submit(run,r) for r in index['runs'] if r['status']!='complete']
        for future in as_completed(futures):future.result()
    finished=[r for r in index['runs'] if r['status']=='complete']
    fields=['name','model','scenario','antenna_mode','speed_mps','duration_s','timeouts','timeouts_before_first_connection',
            'timeouts_total','timeouts_after_first_connection','connection_established','connections','disconnections',
            'attempts','total_data_MB','first_connected_s','connected_at_end','wall_time_s','output','replay']
    with (root/'summary.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields,extrasaction='ignore');writer.writeheader();writer.writerows(finished)
    index['status']='complete' if len(finished)==80 else 'failed'
    index['finished_at']=datetime.now().astimezone().isoformat();save()
    if index['status']!='complete':raise RuntimeError(f'{len(finished)}/80完了。index.jsonとlogsを確認してください')
    plot(root,finished)
    # シナリオ・速度を指定して再生するための小さな入口。
    launcher='''#!/usr/bin/env python3
import argparse,subprocess
from pathlib import Path
p=argparse.ArgumentParser(description='保存済みの①/②・4条件・速度を選んでモニター再生')
p.add_argument('model',choices=['1','2'])
p.add_argument('scenario',choices=['parallel','cross'])
p.add_argument('antenna',choices=['fixed','tracking'])
p.add_argument('speed',type=int,choices=range(1,11))
a=p.parse_args()
model='threshold' if a.model=='1' else 'statistical'
subprocess.run(['bash',str(Path(__file__).resolve().parent/f'{model}_{a.scenario}_{a.antenna}_{a.speed}mps/replay.sh')],check=True)
'''
    (root/'replay.py').write_text(launcher)
    print('全80条件完了:',root,flush=True)


if __name__=='__main__':main()
