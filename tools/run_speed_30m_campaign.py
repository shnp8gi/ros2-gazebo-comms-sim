"""開始30 m・全5パターンの速度1〜5 m/sを0.1 m/s刻みで確認する。"""
import argparse
from collections import defaultdict
from datetime import datetime
import fcntl
import hashlib
import json
import statistics
from pathlib import Path
import subprocess
import sys
from raw_antenna_campaign_common import read_csv, category, run_batch, make_runs, write_csv
from refine_raw_antenna_speed import stats, split_profiles


def condition(r):
    return (r['profile'],r['model'],r['scenario'],r['antenna_mode'],
            float(r['speed_mps']),float(r['start_path_m']))


def stable(rows):
    rows=sorted(rows,key=lambda r:int(r['seed']));middle=len(rows)//2
    halves=[stats(part,'original','check')[0] for part in [rows[:middle],rows[middle:]]]
    return abs(halves[0]['mean_timeouts']-halves[1]['mean_timeouts'])<=.05 and category(halves[0])==category(halves[1])


def save(path,value):
    temp=path.with_suffix('.tmp');temp.write_text(json.dumps(value,ensure_ascii=False,indent=2));temp.replace(path)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path)
    p.add_argument('--jobs',type=int,default=6);p.add_argument('--repetitions',type=int,choices=[20,600],default=20)
    a=p.parse_args();root=a.directory.resolve();out=root/'speed_30m';out.mkdir(exist_ok=True)
    guard=(out/'execution.lock').open('a');fcntl.flock(guard,fcntl.LOCK_EX|fcntl.LOCK_NB)
    index=json.loads((root/'index.json').read_text())
    def phase(name,status='running',**extra):
        index.update(status=status,phase=name,updated_at=datetime.now().astimezone().isoformat(),**extra)
        save(root/'index.json',index)
    phase('speed_30m',speed_step_mps=.1,speed_baseline_start_path_m=30,
          speed_30m_started_at=datetime.now().astimezone().isoformat())
    try:
        for path,digest in json.loads((root/'metadata/input_sha256.json').read_text()).items():
            assert hashlib.sha256((root/path).read_bytes()).hexdigest()==digest,path
        groups={};all_existing_seeds=set()
        for c in index['components']:
            directory=Path(c['output']);profile=c['profile']
            preferred={(r['model'],r['scenario'],r['antenna_mode']):r['phase']
                       for r in read_csv(directory/'analysis/position_summary.csv') if float(r['position_m'])==30}
            for row in read_csv(directory/'analysis/trials.csv'):
                all_existing_seeds.add(int(row['seed']))
                if float(row['start_path_m'])!=30 or row['phase']!=preferred[row['model'],row['scenario'],row['antenna_mode']]:continue
                r=dict(row,profile=profile,source_component=str(directory.relative_to(root)))
                groups.setdefault(condition(r),[]).append(r)
        patterns=[('original','parallel','fixed'),('original','parallel','tracking'),
                  ('original','cross','fixed'),('original','cross','tracking'),('dual60','cross','fixed')]
        cases=sorted((profile,model,scenario,mode,n/10,30.) for profile,scenario,mode in patterns
                     for model in ['threshold','statistical'] for n in range(10,51))
        additions=[]
        def execute(name,cases,repetitions,seed,phase_name):
            runs=make_runs(root,name,cases,repetitions,seed)
            if not runs:return []
            assert not any(r['seed'] in all_existing_seeds for r in runs)
            rows=split_profiles(root,runs,run_batch(out,name,runs,a.jobs))
            rows=[dict(r,phase=phase_name,source_component='speed_30m') for r in rows]
            additions.extend(rows)
            return rows
        missing=[k for k in cases if len(groups.get(k,[]))<(1 if k[1]=='threshold' else a.repetitions)]
        def cost(k):
            known=min((x for x in groups if x[:4]==k[:4]),key=lambda x:abs(x[4]-k[4]))
            return statistics.mean(float(r['wall_seconds']) for r in groups[known])*known[4]/k[4]
        followup=0.
        if a.repetitions<600:
            for k in cases:
                if k[1]!='statistical' or len(groups.get(k,[]))>=600:continue
                known=min((x for x in groups if x[:4]==k[:4]),key=lambda x:abs(x[4]-k[4]))
                if category(stats(groups[known],k[0],'estimate')[0])=='near_zero':followup+=600*cost(k)
            followup+=2400*max(cost(k) for k in cases if k[1]=='statistical' and k[2:] == ('parallel','fixed',3.,30.))
        initial=sum((1 if k[1]=='threshold' else a.repetitions)*cost(k) for k in missing)/a.jobs
        save(out/'plan.json',dict(started_at=datetime.now().astimezone().isoformat(),jobs=a.jobs,
            estimated_initial_seconds=initial,estimated_followup_seconds=followup/a.jobs,
            estimated_postprocess_seconds=120,total_initial_trials=sum(1 if k[1]=='threshold' else a.repetitions for k in missing),
            base_repetitions=a.repetitions,patterns=5,speed_points_per_pattern=41))
        for r in execute('uniform_01mps',missing,a.repetitions,500000000,'speed30_coarse'):
            k=condition(r)
            if groups.get(k) and groups[k][0]['phase']!='speed30_coarse':groups[k]=[]
            groups.setdefault(k,[]).append(r)
        assert set(groups)==set(cases)
        summaries={k:stats(rs,k[0],rs[0]['phase'])[0] for k,rs in groups.items()}
        near={k for k,s in summaries.items() if k[1]=='statistical' and category(s)=='near_zero'}
        for profile,scenario,mode in patterns:
            keys=sorted(k for k in cases if k[:4]==(profile,'statistical',scenario,mode))
            for left,right in zip(keys,keys[1:]):
                if category(summaries[left])!=category(summaries[right]):near.update([left,right])
        needed=sorted(k for k in near if len(groups[k])<600)
        refined=defaultdict(list)
        for r in execute('zero_region_600',needed,600,510000000,'speed30_fine'):refined[condition(r)].append(r)
        groups.update(refined)
        for attempt in range(4):
            pending=sorted(k for k,rs in groups.items() if k[1]=='statistical' and len(rs)>=600 and not stable(rs))
            if not pending:break
            for r in execute(f'convergence_{attempt+1}',pending,600,520000000+attempt*1000000,'speed30_fine'):
                groups[condition(r)].append(r)
        summaries=[];audit=[]
        for k,rs in sorted(groups.items()):
            s=stats(rs,k[0],rs[0]['phase'])[0]
            times=[float(r['first_connected_s']) for r in rs if r['connected']=='1']
            s.update(connected_trials=len(times),first_connected_min_s=min(times) if times else None,
                     first_connected_max_s=max(times) if times else None,
                     first_connected_spread_ms=(max(times)-min(times))*1000 if times else None)
            summaries.append(s)
            if k[1]=='statistical' and len(rs)>=600:
                audit.append(dict(**s,stable=stable(rs),absolute_half_difference=abs(s['first_half_mean']-s['last_half_mean'])))
        seeds=[int(r['seed']) for r in additions];assert len(seeds)==len(set(seeds))
        write_csv(out/'trials.csv',additions);write_csv(out/'speed_summary.csv',summaries)
        write_csv(out/'convergence_audit.csv',audit)
        save(out/'progress.json',dict(status='complete',conditions=len(summaries),trials=len(additions),
            checked_conditions=len(audit),unstable_conditions=sum(not s['stable'] for s in audit)))
        (out/'completed').touch()
        phase('plots_and_summary')
        subprocess.run([sys.executable,str(Path(__file__).with_name('summarize_raw_antenna_campaign.py')),str(root)],check=True)
        phase('report')
        subprocess.run([sys.executable,str(Path(__file__).with_name('write_raw_antenna_report.py')),str(root),
                        str(root/'metadata/report_format_reference.docx')],check=True)
        phase('complete','complete',finished_at=datetime.now().astimezone().isoformat())
        print('開始30 m・速度0.1 m/s刻みの計算、図、レポート更新完了',flush=True)
    except Exception as exc:
        phase('speed_30m','failed',error=str(exc));raise


if __name__=='__main__':main()
