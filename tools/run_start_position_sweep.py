"""指定速度で開始位置を1m刻み、境界を0.1m刻みで計算する（既定5m/s）。"""
import argparse
from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor
import csv
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import statistics
import subprocess
import time
import yaml
from run_probability_campaign import ROOT,container_path,PATTERNS


def write_csv(path,rows):
    if not rows:return
    with path.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)


def summarize(rows,axis):
    grouped=defaultdict(list)
    for r in rows:
        value=round(float(r['start_path_m'] if axis=='path' else r['start_distance_m']),6)
        grouped[r['model'],r['scenario'],r['antenna_mode'],value].append(r)
    result=[]
    for (model,scenario,mode,value),rs in sorted(grouped.items()):
        values=[int(r['timeouts']) for r in rs];n=len(rs)
        result.append(dict(model=model,scenario=scenario,antenna_mode=mode,position_m=value,
            start_path_m=float(rs[0]['start_path_m']),start_distance_m=float(rs[0]['start_distance_m']),trials=n,
            mean_timeouts=statistics.mean(values),connected_fraction=sum(int(r['connected']) for r in rs)/n,
            zero_timeout_connection_fraction=sum(r['connected']=='1' and r['timeouts']=='0' for r in rs)/n,
            min_timeouts=min(values),max_timeouts=max(values),
            first_half_mean=statistics.mean(values[:max(1,n//2)]),last_half_mean=statistics.mean(values[n//2:])))
    return result


def category(row):
    if row['connected_fraction']==0:return 'unconnected'
    return 'near_zero' if row['mean_timeouts']<=.01 and row['zero_timeout_connection_fraction']>=.99 else 'nonzero_or_unreliable'


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path)
    p.add_argument('--axis',choices=['path','distance'],default='path')
    p.add_argument('--max-position',type=float);p.add_argument('--coarse-repetitions',type=int,default=20)
    p.add_argument('--fine-repetitions',type=int,default=600);p.add_argument('--jobs',type=int,default=4)
    p.add_argument('--speed',type=float,default=5)
    p.add_argument('--base-directory',type=Path,help='threshold.yaml/statistical.yamlを置いた共通設定ディレクトリ')
    p.add_argument('--patterns',nargs='+',choices=['parallel_fixed','parallel_tracking','cross_fixed','cross_tracking'])
    p.add_argument('--seed-start',type=int,default=70000000)
    p.add_argument('--skip-plots',action='store_true')
    p.add_argument('--resume',action='store_true',help='保存済み試行を検証して、未計算のseedから再開する')
    a=p.parse_args()
    if not 1<=a.jobs<=8 or min(a.coarse_repetitions,a.fine_repetitions)<=0:p.error('jobsは1〜8、反復数は正値にしてください')
    if not math.isfinite(a.speed) or a.speed<=0:p.error('速度は正の有限値にしてください')
    patterns=[tuple(s.split('_')) for s in a.patterns] if a.patterns else PATTERNS
    root=a.directory.resolve();root.mkdir(parents=True,exist_ok=a.resume)
    (root/'metadata').mkdir(exist_ok=a.resume);(root/'analysis').mkdir(exist_ok=a.resume);maximum=a.max_position or (60 if a.axis=='path' else 150)
    guard=(root/'metadata/execution.lock').open('a');fcntl.flock(guard,fcntl.LOCK_EX|fcntl.LOCK_NB)
    source=a.base_directory.resolve() if a.base_directory else root.parent/'timeout_boundary/metadata'
    for model in ['threshold','statistical']:
        destination=root/'metadata'/f'{model}.yaml'
        if destination.exists():
            assert destination.read_bytes()==(source/f'{model}.yaml').read_bytes(),'再開時に基準設定が変わっています'
        else:shutil.copy2(source/f'{model}.yaml',destination)
    c=yaml.safe_load((root/'metadata/threshold.yaml').read_text());height=c['drone_b']['altitude_m']-c['drone_a']['altitude_m']
    def path_value(scenario,value):
        if a.axis=='path':return value
        square=value**2-height**2-(c['lateral_separation_m']**2 if scenario=='parallel' else 0)
        if square<=0:raise ValueError('機体間距離が最接近距離以下です')
        return math.sqrt(square/(4 if scenario=='parallel' else 2))
    parameters=dict(speed_mps=a.speed,axis=a.axis,max_position_m=maximum,coarse_step_m=1,fine_step_m=.1,
        end_position_m=maximum,end_policy='fixed position after closest approach; independent of starting position',
        coarse_repetitions=a.coarse_repetitions,fine_repetitions=a.fine_repetitions,
        patterns=[list(v) for v in patterns],seed_start=a.seed_start,
        timeout_scope='before first connection; never connected counted until end; unconnected is not zero-timeout success')
    parameter_path=root/'metadata/parameters.json'
    if parameter_path.exists():
        saved=json.loads(parameter_path.read_text());saved.setdefault('patterns',[list(v) for v in PATTERNS]);saved.setdefault('seed_start',70000000)
        assert saved==parameters,'再開時に実験条件が変わっています'
    else:parameter_path.write_text(json.dumps(parameters,ensure_ascii=False,indent=2))
    def batch(name,cases,repetitions,seed_start):
        directory=root/'batches'/name;directory.mkdir(parents=True,exist_ok=a.resume)
        runs=[]
        for i,(model,scenario,mode,value) in enumerate(cases):
            n=1 if model=='threshold' else repetitions
            for repeat in range(n):
                runs.append(dict(id=f'{name}_{i:04}_{repeat:04}',model=model,scenario=scenario,antenna_mode=mode,
                    config=container_path(root/'metadata'/f'{model}.yaml'),speed_mps=a.speed,start_path_m=path_value(scenario,value),end_path_m=path_value(scenario,maximum),
                    seed=seed_start+len(runs)))
        cases_path=directory/'cases.json'
        if cases_path.exists():assert cases_path.read_text()==json.dumps(cases),'再開時に探索位置が変わっています'
        else:cases_path.write_text(json.dumps(cases))
        expected={r['id']:r for r in runs};existing={}
        for result in sorted(directory.glob('results_*.csv')):
            assert result.read_bytes().endswith(b'\n'),f'途中のCSV行を確認してください: {result}'
            with result.open() as stream:
                for row in csv.DictReader(stream):
                    assert None not in row and None not in row.values(),result
                    assert row['id'] in expected and row['id'] not in existing,(result,row['id'])
                    job=expected[row['id']]
                    for key in ['model','scenario','antenna_mode']:assert row[key]==job[key],(result,key)
                    for key in ['speed_mps','start_path_m','end_path_m','seed']:assert float(row[key])==job[key],(result,key)
                    existing[row['id']]=row
        pending=[r for r in runs if r['id'] not in existing]
        generation=0
        while list(directory.glob(f'jobs_resume{generation}_*.yaml')):generation+=1
        def partition(number):
            jobs=pending[number::a.jobs]
            if not jobs:return []
            prefix=f'resume{generation}_' if a.resume else ''
            manifest=directory/f'jobs_{prefix}{number}.yaml';result=directory/f'results_{prefix}{number}.csv'
            manifest.write_text(yaml.safe_dump(dict(runs=jobs),sort_keys=False))
            with (directory/f'run_{prefix}{number}.log').open('w') as log:
                subprocess.run(['docker','run','--rm','--user',f'{os.getuid()}:{os.getgid()}',
                    '-e','HOME=/tmp','-e','OMP_WAIT_POLICY=PASSIVE','-v',f'{ROOT}:/workspace','-w','/workspace',
                    'ros2-gazebo-comms-sim:humble-harmonic','build/no_expected_rssi/comms_sim_pkg/initial_connection_sweep',
                    container_path(manifest),container_path(result)],stdout=log,stderr=subprocess.STDOUT,check=True)
            with result.open() as f:return list(csv.DictReader(f))
        with ThreadPoolExecutor(max_workers=a.jobs) as pool:
            futures=[pool.submit(partition,n) for n in range(a.jobs)]
            while not all(f.done() for f in futures):
                done=sum(max(0,sum(1 for _ in f.open())-1) for f in directory.glob('results_*.csv'))
                (root/'progress.json').write_text(json.dumps(dict(status='running',phase=name,done=done,total=len(runs))))
                print(f'{name}: {done}/{len(runs)}',flush=True);time.sleep(10)
            rows=list(existing.values())+[r for f in futures for r in f.result()]
        assert sorted(r['id'] for r in rows)==sorted(r['id'] for r in runs)
        rows.sort(key=lambda r:r['id']);write_csv(directory/'trials.csv',rows)
        (directory/'completed').write_text(f'{len(rows)} trials\n')
        return rows
    cases=[]
    for model in ['threshold','statistical']:
        for scenario,mode in patterns:
            minimum=0 if a.axis=='path' else math.sqrt(height**2+(c['lateral_separation_m']**2 if scenario=='parallel' else 0))
            first=round(math.floor(minimum*10)/10+.1,1)
            values=sorted({first,*range(math.floor(minimum)+1,math.floor(maximum)+1)})
            cases.extend((model,scenario,mode,float(v)) for v in values)
    coarse=batch('coarse_1m',cases,a.coarse_repetitions,a.seed_start);coarse_summary=summarize(coarse,a.axis)
    write_csv(root/'analysis/coarse_summary.csv',coarse_summary)
    fine_cases=set();regions=[]
    for model in ['threshold','statistical']:
        for scenario,mode in patterns:
            rows=[r for r in coarse_summary if (r['model'],r['scenario'],r['antenna_mode'])==(model,scenario,mode)]
            for left,right in zip(rows,rows[1:]):
                if category(left)==category(right):continue
                regions.append(dict(model=model,scenario=scenario,antenna_mode=mode,lower_m=left['position_m'],upper_m=right['position_m'],
                    lower_class=category(left),upper_class=category(right)))
                for n in range(round(left['position_m']*10),round(right['position_m']*10)+1):
                    fine_cases.add((model,scenario,mode,round(n/10,1)))
            # 全域0回接続なら端点と旧開始位置も反復確認する。未接続の0回は対象外。
            if model=='statistical' and all(category(r)=='near_zero' for r in rows):
                for value in [rows[0]['position_m'],rows[len(rows)//2]['position_m'],rows[-1]['position_m']]:
                    fine_cases.add((model,scenario,mode,value))
    (root/'analysis/refinement_regions.json').write_text(json.dumps(regions,ensure_ascii=False,indent=2))
    fine=batch('fine_01m',sorted(fine_cases),a.fine_repetitions,a.seed_start+1000000) if fine_cases else []
    fine_summary=summarize(fine,a.axis);write_csv(root/'analysis/fine_summary.csv',fine_summary)
    selected={(r['model'],r['scenario'],r['antenna_mode'],r['position_m']):dict(**r,phase='coarse') for r in coarse_summary}
    for r in fine_summary:selected[r['model'],r['scenario'],r['antenna_mode'],r['position_m']]=dict(**r,phase='fine')
    summary=[selected[k] for k in sorted(selected)];write_csv(root/'analysis/position_summary.csv',summary)
    write_csv(root/'analysis/trials.csv',[dict(**r,phase='coarse') for r in coarse]+[dict(**r,phase='fine') for r in fine])
    transitions=[]
    for model in ['threshold','statistical']:
        for scenario,mode in patterns:
            rs=[r for r in summary if (r['model'],r['scenario'],r['antenna_mode'])==(model,scenario,mode)]
            for l,r in zip(rs,rs[1:]):
                if category(l)!=category(r):transitions.append(dict(model=model,scenario=scenario,antenna_mode=mode,
                    lower_m=l['position_m'],upper_m=r['position_m'],lower_class=category(l),upper_class=category(r)))
    write_csv(root/'analysis/boundary_intervals.csv',transitions)
    if not a.skip_plots:plot(root,summary,a.axis,a.speed)
    status=dict(status='complete',trials=len(coarse)+len(fine),conditions=len(summary),parameters=parameters)
    (root/'progress.json').write_text(json.dumps(status,ensure_ascii=False,indent=2))
    print('開始位置の比較完了:',status,flush=True)


def plot(root,rows,axis,speed=5):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams['font.family']='Noto Sans CJK JP'
    label='各機の最接近地点までの開始距離 [m]' if axis=='path' else '開始時の機体間距離 [m]'
    for detail in [False,True]:
        fig,axes=plt.subplots(2,2,figsize=(13,9),layout='constrained')
        for ax,(scenario,mode),title in zip(axes.flat,PATTERNS,['並行・固定','並行・追従','十字・固定','十字・追従']):
            subset=[r for r in rows if (r['scenario'],r['antenna_mode'])==(scenario,mode)]
            for model,name,color,marker in [('threshold','① 閾値','C0','o'),('statistical','② 実測PER＋仮定','C1','s')]:
                rs=[r for r in subset if r['model']==model]
                ax.plot([r['position_m'] for r in rs],[r['mean_timeouts'] if r['connected_fraction']>0 else float('nan') for r in rs],
                    color=color,marker=marker,markersize=3,linestyle='-' if model=='threshold' else '--',label=name)
                disconnected=[r for r in rs if r['connected_fraction']==0]
                if disconnected:ax.scatter([r['position_m'] for r in disconnected],[r['mean_timeouts'] for r in disconnected],marker='x',color='gray',s=24,label=name+' 未接続',zorder=4)
            if all(r['connected_fraction']==0 for r in subset):
                ax.text(.5,.5,'全確認位置でリンク未確立\n0回接続の境界は定義しない',transform=ax.transAxes,ha='center')
                ax.set_ylim(top=1.2)
            elif detail:
                # 全域0回の確認用3点ではなく、実際に区分が変わる位置を拡大する。
                boundary_positions=[]
                for model in ['threshold','statistical']:
                    rs=[r for r in subset if r['model']==model]
                    for left,right in zip(rs,rs[1:]):
                        if category(left)!=category(right):
                            boundary_positions.extend([left['position_m'],right['position_m']])
                if boundary_positions:
                    lower=max(0,min(boundary_positions)-.3);upper=max(boundary_positions)+.3
                    ax.set_xlim(lower,upper)
                    visible=[r['mean_timeouts'] for r in subset if lower<=r['position_m']<=upper]
                    ax.set_ylim(top=max(2,max(visible,default=0))+.3)
            ax.set(title=title,xlabel=label,ylabel='初回リンク確立までの平均Timeout回数');ax.set_ylim(bottom=-.2);ax.grid(alpha=.25);ax.legend(fontsize=8)
        fig.suptitle(f'各機{speed:g} m/s固定：開始位置とTimeout'+('（境界付近）' if detail else ''))
        fig.supxlabel('基本1 m刻み、境界は0.1 m刻み。①は各1回、②は探索20回・確認600回。\n線は計算点間の補助線。×は未接続（観測終了までの回数）。未接続の0回は成功に含めない。\n各位置の試行数・接続率はposition_summary.csvを参照。',fontsize=10)
        for ext in ['png','pdf','svg']:fig.savefig(root/'analysis'/f'position_timeout_{"detail" if detail else "overview"}.{ext}',dpi=180,bbox_inches='tight')
        plt.close(fig)


if __name__=='__main__':main()
