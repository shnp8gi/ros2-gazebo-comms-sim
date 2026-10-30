"""保存済みQしきい値5秒前開始条件を、通常Gazeboで実行・検証・再生保存する。"""
import argparse
from concurrent.futures import ThreadPoolExecutor,as_completed
import csv
from datetime import datetime
import fcntl
import hashlib
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
from run_probability_campaign import ROOT,container_path


def plot(root,rows):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams['font.family']='Noto Sans CJK JP'
    fig,ax=plt.subplots(figsize=(11,6),layout='constrained')
    for model,prefix,color in [('threshold','① 閾値','C0'),('statistical','② 実測PER＋仮定','C1')]:
        for scenario,title,style,marker in [('parallel','並行・追従','-','o'),('cross','十字・追従','--','x')]:
            s=sorted([r for r in rows if r['model']==model and r['scenario']==scenario],key=lambda r:r['speed_mps'])
            ax.plot([r['speed_mps'] for r in s],[r['timeouts'] for r in s],color=color,linestyle=style,marker=marker,label=f'{prefix} / {title}')
    ax.set(xlabel='各機の速度 [m/s]',ylabel='初回リンク確立までのTimeout回数',
        title='Qのしきい値到達5秒前から開始 — 通常Gazebo・各条件1試行',xticks=range(1,11),ylim=(0,max(r['timeouts'] for r in rows)+2))
    ax.grid(alpha=.25);ax.legend(loc='lower right')
    fig.supxlabel('接続待機500 ms。全速度でQが−68.5 dBmへ達する時刻を5秒に固定。\nPは開始時点から受信可能。重なる線は同じ回数。①②はMAC・再送処理も異なる。',fontsize=10)
    for extension in ['png','pdf','svg']:fig.savefig(root/'analysis'/f'timeout_comparison.{extension}',dpi=180)
    plt.close(fig)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path);p.add_argument('--jobs',type=int,default=4)
    a=p.parse_args()
    if not 1<=a.jobs<=8:p.error('jobsは1〜8')
    root=a.directory.resolve();root.relative_to(ROOT);metadata=root/'metadata';metadata.mkdir(exist_ok=True)
    guard=(metadata/'execution.lock').open('a')
    fcntl.flock(guard,fcntl.LOCK_EX|fcntl.LOCK_NB)
    index_path=root/'index.json';index=json.loads(index_path.read_text())
    assert index['policy']=='q_threshold_minus_lead_time' and index['q_threshold_lead_s']==5
    verified=json.loads((root/'analysis/start_verification.json').read_text())
    for path,digest in verified['configs_sha256'].items():
        assert hashlib.sha256((root/path).read_bytes()).hexdigest()==digest,path
    reference={ (r['model'],r['scenario'],int(float(r['speed_mps']))):r
        for r in csv.DictReader((root/'analysis/summary.csv').open())}
    lock=threading.Lock()
    def save():
        temp=index_path.with_suffix('.json.tmp');temp.write_text(json.dumps(index,ensure_ascii=False,indent=2));temp.replace(index_path)
    def update(row,**values):
        with lock:row.update(values);save()
    for row in index['runs']:
        row['antenna_mode']='tracking'
        row['output']=str(root/('model_1_threshold' if row['model']=='threshold' else 'model_2_statistical')/(row['scenario']+'_tracking')/f"{row['speed_mps']:02}mps")
        row.setdefault('status','pending')
    index.update(status='running',execution_started_at=index.get('execution_started_at',datetime.now().astimezone().isoformat()),jobs=a.jobs,repetitions=1,
        timeout_definition='TIMEOUT events strictly before first CONNECTED; all events if never connected',execution_layout='model/scenario/speed')
    save()
    (root/'monitor.sh').write_text('#!/usr/bin/env bash\nset -e\nexec '+shlex.join([sys.executable,str(ROOT/'tools/replay_probability_campaign.py'),str(root)])+'\n')
    (root/'monitor.sh').chmod(0o755)
    def run(row):
        if row['status']=='complete':return
        start=time.monotonic();output=Path(row['output']);name=row['name']
        try:
            update(row,status='running');print('開始:',name,flush=True)
            config_path=ROOT/row['config'].removeprefix('/workspace/')
            if not (output/'completed').exists():
                if output.exists():raise ValueError('未完了の結果があります。上書きしません: '+str(output))
                output.parent.mkdir(parents=True,exist_ok=True)
                command='source /opt/ros/humble/setup.bash && source install/no_expected_rssi/setup.bash && '+shlex.join([
                    'ros2','launch','comms_sim_pkg','sim_launch.py','headless:=true','bridge:=false','config_file:='+row['config'],'output_dir:='+container_path(output)])
                with (metadata/f'{name}.launch.log').open('w') as log:
                    subprocess.run(['docker','run','--rm','--user',f'{os.getuid()}:{os.getgid()}',
                        '-e','HOME=/tmp','-e','DISPLAY=','-e','OMP_WAIT_POLICY=PASSIVE','-e','GZ_PARTITION=q-lead-'+uuid.uuid4().hex,
                        '-v',f'{ROOT}:/workspace','-w','/workspace','ros2-gazebo-comms-sim:humble-harmonic','bash','-c',command],stdout=log,stderr=subprocess.STDOUT,check=True)
            assert (output/'completed').exists()
            with (output/'verification.log').open('w') as log:
                subprocess.run([sys.executable,str(ROOT/'tools/validate_drone_results.py'),str(output)],stdout=log,stderr=subprocess.STDOUT,check=True)
                if row['model']=='statistical':
                    subprocess.run([sys.executable,str(ROOT/'tools/analyze_statistical_results.py'),str(output)],stdout=log,stderr=subprocess.STDOUT,check=True)
            with (output/'summary.csv').open() as f:summary=next(csv.DictReader(f))
            with (output/'timeout_events.csv').open() as f:events=list(csv.DictReader(f))
            first=float(summary['first_connected_s']);initial=sum(first<0 or float(e['time_s'])<first for e in events)
            assert len(events)==int(summary['timeouts'])
            ref=reference[row['model'],row['scenario'],row['speed_mps']]
            assert initial==int(ref['timeouts_before_first_connection']) and abs(first-float(ref['first_connected_s']))<1e-9
            with (output/'timeseries.csv').open() as f:samples=list(csv.DictReader(f))
            at_five=min(samples,key=lambda s:abs(float(s['time_s'])-5))
            assert abs(float(at_five['time_s'])-5)<1e-8 and abs(float(at_five['rssi_ab_dbm'])+68.5)<1e-7
            snapshot=json.loads((output/'live_signals.json').read_text());assert snapshot['complete']
            assert snapshot['timeouts']==int(summary['timeouts'])
            assert abs((snapshot['sample']+1)/snapshot['sample_rate_hz']-row['duration_s'])<1e-7
            assert (output/'monitor_replay.jsonl').stat().st_size>0
            replay=output/'replay.sh'
            replay.write_text('#!/usr/bin/env bash\nset -e\nexec '+shlex.join([sys.executable,str(ROOT/'tools/monitor_hrcp_signals.py'),str(output/'live_signals.json'),'--replay','--autoplay','--window','full','--playback-speed','2'])+'\n');replay.chmod(0o755)
            statistical=yaml.safe_load((output/'statistical_verification.yaml').read_text()) if row['model']=='statistical' else {}
            update(row,status='complete',timeouts=initial,timeouts_total=int(summary['timeouts']),timeouts_after_first_connection=int(summary['timeouts'])-initial,
                first_connected_s=first,connections=int(summary['connections']),disconnections=int(summary['disconnections']),attempts=int(summary['attempts']),
                total_data_MB=float(summary['total_data_MB']),connected_at_end=bool(int(samples[-1]['connected'])),replay=str(replay),
                checked_statistical_frames=statistical.get('checked_frames',0),q_rssi_at_5s_dbm=float(at_five['rssi_ab_dbm']),
                core_check_matches=True,input_sha256=hashlib.sha256(config_path.read_bytes()).hexdigest(),wall_time_s=time.monotonic()-start)
            print(f'完了: {name} 初回までTimeout={initial} 回',flush=True)
        except Exception as exc:
            update(row,status='failed',error=str(exc));print('失敗:',name,str(exc),flush=True)
    ordered=sorted(index['runs'],key=lambda r:(-r['speed_mps'],r['scenario'],r['model']))
    with ThreadPoolExecutor(max_workers=a.jobs) as pool:
        for f in as_completed([pool.submit(run,r) for r in ordered]):f.result()
    done=[r for r in index['runs'] if r['status']=='complete']
    index.update(status='complete' if len(done)==len(index['runs']) else 'failed',execution_finished_at=datetime.now().astimezone().isoformat());save()
    fields=['model','scenario','antenna_mode','speed_mps','initial_distance_m','duration_s','timeouts','timeouts_total','timeouts_after_first_connection',
        'first_connected_s','connections','disconnections','attempts','total_data_MB','connected_at_end','checked_statistical_frames','q_rssi_at_5s_dbm','core_check_matches','output','replay']
    with (root/'analysis/gazebo_summary.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');w.writeheader();w.writerows(done)
    if len(done)!=len(index['runs']):raise RuntimeError(f'{len(done)}/{len(index["runs"])}完了。index.jsonを確認してください')
    plot(root,done)
    result=dict(runs=len(done),shared_core_initial_counts_and_times_match=True,q_threshold_at_5s_verified=True,
        all_replays_available=True,statistical_frames_verified=sum(r['checked_statistical_frames'] for r in done),repetitions=1)
    (root/'analysis/gazebo_verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    print('全条件の通常Gazebo実行・検証・再生保存が完了:',root,flush=True)


if __name__=='__main__':main()
