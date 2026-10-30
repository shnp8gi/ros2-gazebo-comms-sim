"""生CSVキャンペーンの追加試行を、既存結果を上書きせずに実行する。"""
from concurrent.futures import ThreadPoolExecutor
import csv
import json
import os
from pathlib import Path
import subprocess
import time
import yaml
from run_probability_campaign import ROOT, container_path
from run_start_position_sweep import write_csv, category


def read_csv(path):
    if not Path(path).exists():return []
    with Path(path).open() as stream:return list(csv.DictReader(stream))


def typed(row):
    row=dict(row)
    for key in ['speed_mps','position_m','start_path_m','start_distance_m','mean_timeouts',
                'connected_fraction','zero_timeout_connection_fraction','first_half_mean','last_half_mean']:
        if key in row:row[key]=float(row[key])
    for key in ['trials','min_timeouts','max_timeouts']:
        if key in row:row[key]=int(row[key])
    return row


def run_batch(root, name, runs, jobs):
    directory=root/'batches'/name;directory.mkdir(parents=True,exist_ok=True)
    manifest_all=directory/'runs.json'
    serialized=json.dumps(runs,ensure_ascii=False,indent=2)
    if manifest_all.exists():assert manifest_all.read_text()==serialized,'追加試行の条件が変化しました'
    else:manifest_all.write_text(serialized)
    expected={r['id']:r for r in runs};assert len(expected)==len(runs)
    existing={}
    for path in sorted(directory.glob('results_*.csv')):
        assert path.read_bytes().endswith(b'\n'),f'CSVの末尾が不完全です: {path}'
        for r in read_csv(path):
            assert r['id'] in expected and r['id'] not in existing
            e=expected[r['id']]
            for key in ['model','scenario','antenna_mode']:assert r[key]==e[key]
            for key in ['seed','speed_mps','start_path_m','end_path_m']:assert float(r[key])==e[key]
            existing[r['id']]=r
    pending=[r for r in runs if r['id'] not in existing]
    generation=0
    while list(directory.glob(f'jobs_{generation}_*.yaml')):generation+=1
    def partition(number):
        part=pending[number::jobs]
        if not part:return []
        manifest=directory/f'jobs_{generation}_{number}.yaml';result=directory/f'results_{generation}_{number}.csv'
        manifest.write_text(yaml.safe_dump(dict(runs=part),sort_keys=False))
        with manifest.with_suffix('.log').open('w') as log:
            subprocess.run(['docker','run','--rm','--user',f'{os.getuid()}:{os.getgid()}',
                '-e','HOME=/tmp','-e','OMP_WAIT_POLICY=PASSIVE','-v',f'{ROOT}:/workspace','-w','/workspace',
                'ros2-gazebo-comms-sim:humble-harmonic','build/no_expected_rssi/comms_sim_pkg/initial_connection_sweep',
                container_path(manifest),container_path(result)],stdout=log,stderr=subprocess.STDOUT,check=True)
        return read_csv(result)
    with ThreadPoolExecutor(max_workers=jobs) as pool:
        futures=[pool.submit(partition,i) for i in range(jobs)]
        while not all(f.done() for f in futures):
            done=sum(max(0,sum(1 for _ in p.open())-1) for p in directory.glob('results_*.csv'))
            temp=root/'progress.tmp';temp.write_text(json.dumps(dict(status='running',phase=name,done=done,total=len(runs))))
            temp.replace(root/'progress.json');print(f'{name}: {done}/{len(runs)}',flush=True);time.sleep(5)
        result=list(existing.values())+[r for f in futures for r in f.result()]
    result.sort(key=lambda r:r['id']);assert len(result)==len(runs)
    write_csv(directory/'trials.csv',result)
    return result


def make_runs(root, name, cases, repetitions, seed_start):
    runs=[]
    for i,(profile,model,scenario,mode,speed,position) in enumerate(cases):
        for j in range(1 if model=='threshold' else repetitions):
            runs.append(dict(id=f'{name}_{i:04}_{j:04}',model=model,scenario=scenario,antenna_mode=mode,
                config=container_path(root/'configs'/profile/f'{model}.yaml'),speed_mps=speed,
                start_path_m=position,end_path_m=60.,seed=seed_start+len(runs)))
    return runs
