"""開始距離25 mの固定アンテナ速度境界と、確率モデルの反復安定性を確認する。"""
import argparse
from collections import defaultdict
import json
from pathlib import Path
import statistics
from raw_antenna_campaign_common import read_csv, typed, category, run_batch, make_runs, write_csv


def stats(rows, profile, phase):
    grouped=defaultdict(list)
    for r in rows:grouped[r['model'],r['scenario'],r['antenna_mode'],float(r['speed_mps']),float(r['start_path_m'])].append(r)
    result=[]
    for (model,scenario,mode,speed,pos),rs in sorted(grouped.items()):
        rs.sort(key=lambda r:int(r['seed']));v=[int(r['timeouts']) for r in rs];n=len(v)
        result.append(dict(profile=profile,model=model,scenario=scenario,antenna_mode=mode,speed_mps=speed,
            position_m=pos,start_path_m=pos,start_distance_m=float(rs[0]['start_distance_m']),trials=n,
            mean_timeouts=statistics.mean(v),connected_fraction=sum(int(r['connected']) for r in rs)/n,
            zero_timeout_connection_fraction=sum(r['connected']=='1' and r['timeouts']=='0' for r in rs)/n,
            min_timeouts=min(v),max_timeouts=max(v),first_half_mean=statistics.mean(v[:max(1,n//2)]),
            last_half_mean=statistics.mean(v[n//2:]),phase=phase))
    return result


def split_profiles(root, runs, rows):
    profiles={r['id']:Path(r['config']).parent.name for r in runs}
    return [dict(r,profile=profiles[r['id']]) for r in rows]


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path);p.add_argument('--jobs',type=int,default=6)
    a=p.parse_args();root=a.directory.resolve();out=root/'speed_refinement';out.mkdir(exist_ok=True)
    all_positions=[];source_trials=[]
    for c in json.loads((root/'index.json').read_text())['components']:
        assert c['status']=='complete'
        for r in read_csv(Path(c['output'])/'analysis/position_summary.csv'):
            all_positions.append(dict(typed(r),profile=c['profile'],speed_mps=float(c['speed_mps'])))
        for r in read_csv(Path(c['output'])/'analysis/trials.csv'):
            source_trials.append(dict(r,profile=c['profile']))
    cases=set()
    for profile,patterns in [('original',['parallel','cross']),('dual60',['cross'])]:
        for model in ['threshold','statistical']:
            for scenario in patterns:
                rs=sorted((r for r in all_positions if (r['profile'],r['model'],r['scenario'],r['antenna_mode'],r['position_m'])==
                           (profile,model,scenario,'fixed',25.)),key=lambda r:r['speed_mps'])
                assert len(rs)==5
                for l,r in zip(rs,rs[1:]):
                    if category(l)!=category(r):
                        for n in range(round(l['speed_mps']*10),round(r['speed_mps']*10)+1):
                            cases.add((profile,model,scenario,'fixed',round(n/10,1),25.))
                # 全域0回の場合も速度の両端を600試行で確認する。
                if model=='statistical' and all(category(r)=='near_zero' for r in rs):
                    for speed in [1.,5.]:cases.add((profile,model,scenario,'fixed',speed,25.))
    runs=make_runs(root,'speed_01mps',sorted(cases),600,300000000)
    rows=split_profiles(root,runs,run_batch(out,'speed_01mps',runs,a.jobs)) if runs else []
    summaries=[]
    for profile in ['original','dual60']:summaries+=stats([r for r in rows if r['profile']==profile],profile,'speed_fine')
    write_csv(out/'trials.csv',rows);write_csv(out/'speed_summary.csv',summaries)

    # 確率モデルの細分化条件に対し、600回単位で前後半平均の安定性を確認する。
    # 判定は信頼区間ではなく、平均差0.05回以下かつ0回付近の区分一致。
    conditions={}
    for s in all_positions+summaries:
        if s['model']!='statistical' or s['trials']<600:continue
        key=(s['profile'],s['model'],s['scenario'],s['antenna_mode'],s['speed_mps'],s['position_m'])
        conditions[key]=s
    groups={}
    for key,s in conditions.items():
        profile,model,scenario,mode,speed,pos=key
        pool=rows if s['phase']=='speed_fine' else source_trials
        groups[key]=sorted([r for r in pool if (r['profile'],r['model'],r['scenario'],r['antenna_mode'],
            float(r['speed_mps']),float(r['start_path_m']))==key and
            (s['phase']=='speed_fine' or r['phase']=='fine')],key=lambda r:int(r['seed']))
        assert len(groups[key])==s['trials']
    def stable(rs):
        halves=[rs[:len(rs)//2],rs[len(rs)//2:]]
        ss=[stats(h,'unused','unused')[0] for h in halves]
        return abs(ss[0]['mean_timeouts']-ss[1]['mean_timeouts'])<=.05 and category(ss[0])==category(ss[1])
    additions=[]
    for round_index in range(4):
        pending=sorted(k for k,rs in groups.items() if not stable(rs))
        if not pending:break
        name=f'convergence_{round_index+1}'
        more_runs=make_runs(root,name,pending,600,400000000+round_index*10000000)
        more=split_profiles(root,more_runs,run_batch(out,name,more_runs,a.jobs));additions+=more
        for r in more:
            k=(r['profile'],r['model'],r['scenario'],r['antenna_mode'],float(r['speed_mps']),float(r['start_path_m']))
            groups[k].append(r)
    audit=[];overrides=[]
    for k,rs in sorted(groups.items()):
        s=stats(rs,k[0],'convergence')[0];ok=stable(rs)
        audit.append(dict(**s,stable=ok,absolute_half_difference=abs(s['first_half_mean']-s['last_half_mean']),
                          source_phase=conditions[k]['phase']))
        if len(rs)>conditions[k]['trials']:overrides.append(dict(**s,source_phase=conditions[k]['phase']))
    write_csv(out/'convergence_trials.csv',additions);write_csv(out/'convergence_summary.csv',overrides)
    write_csv(out/'convergence_audit.csv',audit)
    (out/'progress.json').write_text(json.dumps(dict(status='complete',trials=len(rows)+len(additions),
        checked_conditions=len(audit),unstable_conditions=sum(not r['stable'] for r in audit)),indent=2))
    print('速度境界・反復安定性確認完了',len(rows)+len(additions),'追加試行',flush=True)


if __name__=='__main__':main()
