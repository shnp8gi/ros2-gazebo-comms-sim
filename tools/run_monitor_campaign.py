#!/usr/bin/env python3
"""詳細PHYの4条件→固定並行1～10m/s。保存はモニター記録と再生設定・一覧だけ。ROS環境内で実行。"""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import copy
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time
import uuid
import yaml


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',default='src/comms_sim_pkg/config/drone_hrcp_fspl.yaml')
    parser.add_argument('--output',required=True)
    parser.add_argument('--jobs',type=int,default=1,help='同時に実行する条件数')
    parser.add_argument('--cpu-budget',type=int,default=0,help='各段階の復号スレッド総数。0なら設定値を維持')
    parser.add_argument('--decode-threads',type=int,help='各条件の復号スレッド数を明示する')
    parser.add_argument('--smoke-duration-s',type=float,help='動作確認専用。全条件を最接近付近の短区間で確認')
    args=parser.parse_args()
    if args.jobs<1 or args.cpu_budget<0:parser.error('jobsは1以上、cpu-budgetは0以上です')
    if args.decode_threads is not None and (not 1<=args.decode_threads<=64 or args.cpu_budget):
        parser.error('decode-threadsは1～64、cpu-budgetと併用できません')
    if args.smoke_duration_s is not None and not 0<args.smoke_duration_s<=0.1:
        parser.error('動作確認の時間は0秒超～0.1秒です')
    config_path=Path(args.config).resolve()
    base=yaml.safe_load(config_path.read_text())
    if args.decode_threads is not None:base['hrcp']['decode_threads']=args.decode_threads
    if base.get('preview_mode') or base['awgn']['enabled'] or not base['hrcp']['enabled'] or not base.get('monitor_only'):
        raise ValueError('この実験は無AWGN・詳細HRCP・monitor_onlyで実行してください')
    for key in ('e_plane','h_plane'):
        base['antenna'][key]=str((config_path.parent/base['antenna'][key]).resolve())
    base['mcs_table']=str((config_path.parent/base['mcs_table']).resolve())
    root=Path(args.output).resolve();root.mkdir(parents=True,exist_ok=False)
    cases=[(scenario,mode,1) for scenario in ('parallel','cross') for mode in ('fixed','tracking')]
    cases += [('parallel','fixed',speed) for speed in range(2,11)]
    index={'schema':'drone_monitor_campaign_v1','started_at':datetime.now().astimezone().isoformat(),
           'status':'running','distance_per_drone_m':None if args.smoke_duration_s else 50,'noise_enabled':False,'preview':False,
           'speed_1_reuses':'parallel_fixed_1mps','runs':[],
           'jobs':args.jobs,'cpu_budget':args.cpu_budget,'decode_threads':args.decode_threads,
           'scope':'smoke_test' if args.smoke_duration_s else 'full_route'}
    lock=threading.Lock()
    stopping=threading.Event()
    processes={}
    def save():
        tmp=root/'index.json.tmp';tmp.write_text(json.dumps(index,ensure_ascii=False,indent=2)+'\n');tmp.replace(root/'index.json')
    for scenario,mode,speed in cases:
        index['runs'].append({'name':f'{scenario}_{mode}_{speed}mps','scenario':scenario,'antenna_mode':mode,
                              'speed_mps':speed,'duration_s':args.smoke_duration_s or 50/speed,'status':'pending'})
    save()
    def update(row,**values):
        with lock:
            row.update(values)
            save()

    def terminate(process):
        if process.poll() is None:
            try:os.killpg(process.pid,signal.SIGTERM)
            except ProcessLookupError:pass

    def run(row,workers):
        start=time.monotonic()
        try:
            if stopping.is_set():
                update(row,status='interrupted')
                return
            c=copy.deepcopy(base)
            c['scenario']=row['scenario'];c['antenna_mode']=row['antenna_mode']
            c['drone_a']['speed_kmh']=c['drone_b']['speed_kmh']=row['speed_mps']*3.6
            c['duration_s']=row['duration_s'];c['encounter_time_s']=c['duration_s']/2
            if args.cpu_budget:c['hrcp']['decode_threads']=max(1,min(64,args.cpu_budget//workers))
            output=root/row['name']
            update(row,status='running',started_at=datetime.now().astimezone().isoformat(),
                   decode_threads=c['hrcp']['decode_threads'])
            print(f"実行開始: {row['name']} / 航行{c['duration_s']:g}秒",flush=True)
            # 起動用設定とROS/Gazeboのテキストログは/tmpに置く。再生用config.yamlはlaunchが保存する。
            with tempfile.TemporaryDirectory(prefix='drone_campaign_') as scratch:
                input_file=Path(scratch)/'input.yaml';input_file.write_text(yaml.safe_dump(c,sort_keys=False))
                command=['ros2','launch','comms_sim_pkg','sim_launch.py','headless:=true','bridge:=false',
                         f'config_file:={input_file}',f'output_dir:={output}']
                with (Path(scratch)/'launch.log').open('w') as log:
                    # 同時Gazeboの通信を条件ごとに隔離。物理・通信の計算刻みは変えない。
                    env=dict(os.environ,GZ_PARTITION=f'drone_campaign_{uuid.uuid4().hex}',
                             OMP_WAIT_POLICY='PASSIVE')
                    process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,
                                             env=env,start_new_session=True)
                    with lock:processes[row['name']]=process
                    update(row,launch_pid=process.pid)
                    try:
                        while process.poll() is None:
                            if stopping.wait(.5):
                                terminate(process)
                                try:process.wait(timeout=20)
                                except subprocess.TimeoutExpired:
                                    try:os.killpg(process.pid,signal.SIGKILL)
                                    except ProcessLookupError:pass
                                    process.wait()
                                break
                    finally:
                        with lock:processes.pop(row['name'],None)
                if process.returncode:
                    raise RuntimeError((Path(scratch)/'launch.log').read_text()[-6000:])
            snapshot=json.loads((output/'live_signals.json').read_text())
            if not (output/'completed').exists() or not snapshot['complete'] or snapshot['rssi_includes_noise']:
                raise RuntimeError('正常終了または無AWGN条件の検証に失敗しました')
            if abs((snapshot['sample']+1)/snapshot['sample_rate_hz']-c['duration_s'])>1e-8:
                raise RuntimeError('航行終端の時刻不一致')
            if not (output/'monitor_replay.jsonl').is_file():raise RuntimeError('再生記録がありません')
            update(row,status='complete',wall_time_s=time.monotonic()-start,
                       delivered_mb=snapshot['delivered_mb'],connections=snapshot['connections'],
                       timeouts=snapshot['timeouts'],disconnections=snapshot['disconnections'])
            print(f"完了: {row['name']} / {snapshot['delivered_mb']:.6f} MB",flush=True)
        except Exception as exc:
            update(row,status='interrupted' if stopping.is_set() else 'failed',
                   wall_time_s=time.monotonic()-start,error=str(exc))
            print(f"停止: {row['name']} / {exc}",flush=True)

    def interrupted(signum,frame):
        raise KeyboardInterrupt('停止要求を受信しました')
    signal.signal(signal.SIGTERM,interrupted)
    try:
        # 4シナリオを終えてから速度スイープへ進む。1m/sの固定並行は再利用する。
        for stage in (index['runs'][:4],index['runs'][4:]):
            workers=min(args.jobs,len(stage),args.cpu_budget or args.jobs)
            pool=ThreadPoolExecutor(max_workers=workers)
            try:
                futures=[pool.submit(run,row,workers) for row in stage]
                for future in as_completed(futures):future.result()
            except BaseException:
                stopping.set()
                with lock:
                    for process in processes.values():terminate(process)
                raise
            finally:
                pool.shutdown(wait=True)
        with lock:
            index['status']='complete' if all(r['status']=='complete' for r in index['runs']) else 'failed'
            save()
        if index['status']!='complete':raise RuntimeError('一部の条件が失敗しました。index.jsonを確認してください')
    except BaseException as exc:
        index['status']='interrupted' if isinstance(exc,KeyboardInterrupt) else 'failed'
        index['error']=str(exc);save();raise


if __name__=='__main__':main()
