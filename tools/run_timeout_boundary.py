"""初回リンク確立までのTimeout境界を、既存コアで独立seed反復する。"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import csv
import json
import os
from pathlib import Path
import subprocess
import time
import yaml
from run_probability_campaign import ROOT,container_path


def collect(batch):
    """親プロセスの中断後も、完了した全partitionを照合して回収できる。"""
    rows=[];expected=[]
    for manifest in sorted(batch.glob('jobs_*.yaml')):
        jobs=yaml.safe_load(manifest.read_text())['runs'];expected.extend(r['id'] for r in jobs)
        result=batch/manifest.name.replace('jobs_','results_').replace('.yaml','.csv')
        if not result.exists():raise ValueError('未完了: '+str(result))
        rows.extend(csv.DictReader(result.open()))
    assert expected and sorted(r['id'] for r in rows)==sorted(expected),'全試行が完了してから回収してください'
    rows.sort(key=lambda r:r['id'])
    with (batch/'trials.csv').open('w',newline='') as stream:
        w=csv.DictWriter(stream,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    (batch/'completed').write_text(f'{len(rows)} trials; all partition IDs verified\n')
    print(f'{batch.name}: {len(rows)}試行を回収しました',flush=True)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('directory',type=Path)
    p.add_argument('--name',required=True)
    p.add_argument('--model',choices=['threshold','statistical'])
    p.add_argument('--scenario',choices=['parallel','cross'],default='parallel')
    p.add_argument('--antenna',choices=['fixed','tracking'])
    p.add_argument('--speeds',type=float,nargs='+')
    p.add_argument('--collect-only',action='store_true')
    p.add_argument('--repetitions',type=int,default=1)
    p.add_argument('--seed-start',type=int,default=40000000)
    p.add_argument('--jobs',type=int,default=4)
    a=p.parse_args()
    if a.collect_only:
        collect(a.directory.resolve()/'batches'/a.name);return
    if not a.model or not a.antenna or not a.speeds:p.error('model、antenna、speedsを指定してください')
    if a.repetitions<1 or not 1<=a.jobs<=8 or any(v<=0 for v in a.speeds):p.error('正の速度・試行数、jobs 1〜8を指定してください')
    if a.model=='threshold' and a.repetitions!=1:p.error('①は決定論モデルなので同じ速度の反復は不要です')
    directory=a.directory.resolve();batch=directory/'batches'/a.name
    batch.mkdir(parents=True,exist_ok=False)
    parameters=vars(a).copy();parameters['directory']=str(directory)
    (batch/'parameters.json').write_text(json.dumps(parameters,ensure_ascii=False,indent=2))
    runs=[]
    for i,speed in enumerate(a.speeds):
        for repetition in range(a.repetitions):
            runs.append(dict(id=f'{a.name}_{i:03}_{repetition:05}',model=a.model,
                config=container_path(directory/'metadata'/f'{a.model}.yaml'),
                scenario=a.scenario,antenna_mode=a.antenna,speed_mps=speed,
                seed=a.seed_start+i*a.repetitions+repetition))
    # 分割は速度ごとでなくround-robin。各速度の途中経過も同程度に得られる。
    def run_partition(partition):
        manifest=batch/f'jobs_{partition}.yaml';result=batch/f'results_{partition}.csv'
        manifest.write_text(yaml.safe_dump(dict(runs=runs[partition::a.jobs]),sort_keys=False))
        command=['docker','run','--rm','--user',f'{os.getuid()}:{os.getgid()}',
            '-e','HOME=/tmp','-e','OMP_WAIT_POLICY=PASSIVE','-v',f'{ROOT}:/workspace','-w','/workspace',
            'ros2-gazebo-comms-sim:humble-harmonic','build/no_expected_rssi/comms_sim_pkg/timeout_boundary_sweep',
            container_path(manifest),container_path(result)]
        with (batch/f'run_{partition}.log').open('w') as log:
            subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True)
        rows=list(csv.DictReader(result.open()))
        assert len(rows)==len(runs[partition::a.jobs])
        return rows
    start=time.monotonic()
    with ThreadPoolExecutor(max_workers=a.jobs) as pool:
        futures=[pool.submit(run_partition,i) for i in range(a.jobs)]
        while not all(f.done() for f in futures):
            done=sum(max(0,sum(1 for _ in path.open())-1) for path in batch.glob('results_*.csv'))
            print(f'{a.name}: {done}/{len(runs)}試行、経過{time.monotonic()-start:.1f}秒',flush=True)
            time.sleep(10)
        rows=[row for f in futures for row in f.result()]
    rows.sort(key=lambda r:r['id'])
    with (batch/'trials.csv').open('w',newline='') as stream:
        w=csv.DictWriter(stream,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    (batch/'completed').write_text(f'{len(rows)} trials; {time.monotonic()-start:.3f} seconds\n')
    for speed in a.speeds:
        r=[x for x in rows if float(x['speed_mps'])==speed]
        print(f"{a.model} {a.scenario}/{a.antenna} {speed:.6f} m/s: n={len(r)}, mean={sum(int(x['timeouts']) for x in r)/len(r):.6f}, zero_connected={sum(x['connected']=='1' and x['timeouts']=='0' for x in r)}/{len(r)}",flush=True)


if __name__=='__main__':main()
