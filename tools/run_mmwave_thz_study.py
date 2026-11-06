#!/usr/bin/env python3
"""ミリ波/テラ波の初回リンク確立を、速度・開始距離・4パターンで比較する。"""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from copy import deepcopy
import csv
from datetime import datetime
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

PATTERNS=('parallel_fixed','parallel_tracking','cross_fixed','cross_tracking')
MODELS=('empirical','theoretical')


def read_csv(path):
    with path.open() as f:return list(csv.DictReader(f))


def write_csv(path,rows):
    temporary=path.with_suffix('.tmp')
    with temporary.open('w') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    temporary.replace(path)


def write_json(path,value):
    temporary=path.with_suffix('.tmp');temporary.write_text(json.dumps(value,ensure_ascii=False,indent=2));temporary.replace(path)


def main():
    repo=Path(__file__).resolve().parents[1]
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,default=repo/'sim_results/mmwave_thz_four_study_20261102')
    p.add_argument('--jobs',type=int,default=6);p.add_argument('--trials',type=int,default=30)
    p.add_argument('--seed',type=int,default=20261102);p.add_argument('--resume',action='store_true')
    p.add_argument('--reference',type=Path,default=Path('/home/rosgazebo/Downloads/初回リンク確立までのTimeout回数評価.docx'))
    a=p.parse_args()
    if not 1<=a.jobs<=8 or not 2<=a.trials<=99 or a.seed<0:p.error('jobs=1〜8、trials=2〜99、seed>=0を指定してください')
    root=a.output.resolve();root.mkdir(parents=True,exist_ok=a.resume)
    for folder in ('internal','inputs','configs','speed','distance','refinement'):(root/folder).mkdir(exist_ok=a.resume)
    binary=repo/'build/tjx_theory/initial_connection_sweep';binary_hash=hashlib.sha256(binary.read_bytes()).hexdigest()
    provenance_path=root/'internal/provenance.json'
    if provenance_path.exists():
        provenance=json.loads(provenance_path.read_text())
        assert provenance['binary_sha256']==binary_hash,'実行ファイルが前回と異なります'
        assert provenance['trials']==a.trials and provenance['seed']==a.seed
        for relative,digest in provenance['input_hashes'].items():assert hashlib.sha256((root/relative).read_bytes()).hexdigest()==digest
    else:
        d=repo/'src/comms_sim_pkg/config'
        theory=yaml.safe_load((d/'scenarios/drone_dual60_cross_fixed_theoretical.yaml').read_text())
        measured=yaml.safe_load((d/'scenarios/drone_statistical_measured_provisional.yaml').read_text())
        for direction in ('radio_ab','radio_ba'):
            theory[direction]=deepcopy(measured[direction])
            for key in ('tx_antenna_gain_dbi','rx_antenna_gain_dbi'):theory[direction].pop(key,None)
        theory['hrcp']['monitor_enabled']=False
        theory['raw_antenna_campaign']['profile']='mixed_mmwave_thz'
        for name in ('e_plane.csv','h_plane.csv','MCStable.csv','receiver_curve_measured_provisional.csv','tjx_theoretical_lut.yaml'):
            category='antennas' if name in ('e_plane.csv','h_plane.csv') else 'phy'
            shutil.copyfile(d/category/name,root/'inputs'/name)
        for key in ('e_plane','h_plane'):theory['antenna'][key]=str(root/'inputs'/Path(theory['antenna'][key]).name)
        theory['theoretical_phy']['table']=str(root/'inputs/tjx_theoretical_lut.yaml')
        for pattern in PATTERNS:
            scenario,mode=pattern.split('_')
            for model in MODELS:
                c=deepcopy(theory);c['scenario']=scenario;c['antenna_mode']=mode
                c['drone_a']['fixed_azimuth_offset_deg']=90 if scenario=='parallel' else -45
                c['drone_b']['fixed_azimuth_offset_deg']=90 if scenario=='parallel' else 45
                c['antenna_mount_revision']='parallel_sideways_45_elevation' if scenario=='parallel' else 'cross_facing_yaw_pitch_45deg'
                c['output_dir']=str(root/pattern/model)
                if model=='empirical':
                    c.pop('theoretical_phy');c.pop('model_assumptions',None)
                    c['empirical_phy']=deepcopy(measured['empirical_phy']);c['empirical_phy']['table']=str(root/'inputs/receiver_curve_measured_provisional.csv')
                    c['measurement_assumptions']=measured['measurement_assumptions'];c['mcs_table']=str(root/'inputs/MCStable.csv')
                (root/'configs'/f'{pattern}_{model}.yaml').write_text(yaml.safe_dump(c,allow_unicode=True,sort_keys=False))
        shutil.copyfile(a.reference,root/'internal/reference.docx')
        source_paths=list((repo/'src/comms_sim_pkg/src').glob('*.cpp'))+list((repo/'src/comms_sim_pkg/include/comms_sim_pkg').glob('*.hpp'))+[Path(__file__).resolve()]
        for source in source_paths:
            target=root/'internal/source_snapshot'/source.relative_to(repo);target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(source,target)
        provenance={'reference':str(a.reference),'reference_sha256':hashlib.sha256(a.reference.read_bytes()).hexdigest(),
            'binary_sha256':binary_hash,'trials':a.trials,'seed':a.seed,
            'git_branch':subprocess.check_output(['git','branch','--show-current'],cwd=repo,text=True).strip(),
            'execution':'native_shared_MAC; terminate_at_first_CONNECTED; no_Gazebo_process',
            'approved_scope':'4_patterns_speed_1_to_5_step_0.1; tracking_start_1_to_60_step_1; boundary_refinement_0.1',
            'approved_cross_fixed_angles':'A roll/pitch/yaw=0/-45/-45; B=0/45/45; overrides_reference_old_sideways',
            'reference_not_specified_noise':'prior_RF_defaults_retained: Q_NF0_P_NF10_T300_B2.1648GHz',
            'report_policy':'calibration_and_implementation_internal_only; antenna_gain_detail_in_report',
            'input_hashes':{str(path.relative_to(root)):hashlib.sha256(path.read_bytes()).hexdigest() for folder in ('inputs','configs') for path in (root/folder).iterdir()}}
        write_json(provenance_path,provenance)
    started=time.monotonic();all_results=[];aggregates=[]

    def evaluate(case):
        phase,ordinal,pattern,model,speed,start=case
        directory=root/phase/f'{pattern}_{model}_v{speed:.1f}_s{start:.1f}';directory.mkdir(exist_ok=True)
        result=directory/'results.csv';runs=[];scenario,mode=pattern.split('_')
        phase_number={'speed':0,'distance':1,'refinement':2}[phase]
        for index in range(a.trials):
            seed=a.seed+phase_number*100000000+ordinal*100+index
            runs.append({'id':f'{phase}_{ordinal:04d}_{model}_{index:02d}','model':model,'scenario':scenario,'antenna_mode':mode,
                'config':str(root/'configs'/f'{pattern}_{model}.yaml'),'speed_mps':speed,'start_path_m':start,'end_path_m':60.,'seed':seed,'mac_seed':seed^0x12345678})
        manifest=directory/'manifest.yaml'
        if manifest.exists():assert yaml.safe_load(manifest.read_text())=={'runs':runs}
        else:manifest.write_text(yaml.safe_dump({'runs':runs},sort_keys=False))
        if not result.exists():
            partial=directory/'results.partial.csv'
            if partial.exists():partial.rename(directory/f'incomplete_{time.time_ns()}.csv')
            with (directory/'execution.log').open('a') as log:
                subprocess.run([str(binary),str(manifest),str(partial)],cwd=repo,stdout=log,stderr=subprocess.STDOUT,check=True,
                               env=dict(os.environ,OMP_NUM_THREADS='1',OMP_WAIT_POLICY='PASSIVE'))
            rows=read_csv(partial);assert len(rows)==a.trials
            partial.replace(result)
        rows=read_csv(result);assert len(rows)==a.trials
        for row,job in zip(rows,runs):
            assert row['id']==job['id'] and int(row['seed'])==job['seed']
            assert row['model']==model and row['scenario']==scenario and row['antenna_mode']==mode
            assert math.isclose(float(row['speed_mps']),speed) and math.isclose(float(row['start_path_m']),start)
            assert math.isclose(float(row['observation_duration_s']),(start+60)/speed)
            assert int(row['connected'])==int(float(row['first_connected_s'])>=0)
        values=[int(r['timeouts']) for r in rows];connected=[r for r in rows if int(r['connected'])]
        successful_times=[float(r['first_connected_s']) for r in connected]
        zero=sum(int(r['connected']) and int(r['timeouts'])==0 for r in rows)
        summary={'phase':phase,'pattern':pattern,'model':model,'speed_mps':speed,'start_path_m':start,'trials':len(rows),
            'connected_trials':len(connected),'connected_fraction':len(connected)/len(rows),'zero_timeout_connected_trials':zero,
            'zero_timeout_connection_fraction':zero/len(rows),'mean_timeouts':statistics.mean(values),'variance_timeouts':statistics.pvariance(values),
            'sample_variance_timeouts':statistics.variance(values),'stddev_timeouts':statistics.pstdev(values),'min_timeouts':min(values),'max_timeouts':max(values),
            'mean_first_connected_s':statistics.mean(successful_times) if successful_times else '',
            'mean_timeouts_connected':statistics.mean(int(r['timeouts']) for r in connected) if connected else '',
            'category':'all_zero_connected' if zero==len(rows) else 'none_connected' if not connected else 'nonzero_or_mixed'}
        return rows,summary

    def phase(name,cases):
        print(f'工程 {name}: {len(cases)}条件 × {a.trials}試行',flush=True)
        completed=0;phase_results=[];phase_stats=[]
        with ThreadPoolExecutor(max_workers=a.jobs) as pool:
            futures=[pool.submit(evaluate,case) for case in cases]
            for future in as_completed(futures):
                rows,summary=future.result();phase_results.extend(rows);phase_stats.append(summary);completed+=1
                if completed%10==0 or completed==len(cases):
                    print(f'{name}: {completed}/{len(cases)}条件完了、経過{time.monotonic()-started:.0f}秒',flush=True)
                    write_json(root/'internal/progress.json',{'phase':name,'completed_conditions':completed,'total_conditions':len(cases),
                        'completed_trials_current_phase':completed*a.trials,'elapsed_s':time.monotonic()-started})
        phase_stats.sort(key=lambda r:(r['pattern'],r['model'],r['speed_mps'],r['start_path_m']))
        phase_results.sort(key=lambda r:r['id'])
        write_csv(root/f'{name}_trials.csv',phase_results);write_csv(root/f'{name}_summary.csv',phase_stats)
        all_results.extend(phase_results);aggregates.extend(phase_stats)
        return phase_stats

    speed_cases=[]
    for pi,pattern in enumerate(PATTERNS):
        for si in range(41):
            for model in MODELS:speed_cases.append(('speed',pi*41+si,pattern,model,round(1+si*.1,1),30.))
    phase('speed',speed_cases)
    distance_cases=[]
    for pi,pattern in enumerate(('parallel_tracking','cross_tracking')):
        for speed in range(1,6):
            for start in range(1,61):
                for model in MODELS:distance_cases.append(('distance',pi*300+(speed-1)*60+start-1,pattern,model,float(speed),float(start)))
    distance=phase('distance',distance_cases)
    # 分類が変わる粗探索区間だけを両モデルとも0.1m刻みで追加する。
    intervals=set()
    for pattern in ('parallel_tracking','cross_tracking'):
        for speed in range(1,6):
            for model in MODELS:
                group=sorted((r for r in distance if r['pattern']==pattern and r['speed_mps']==speed and r['model']==model),key=lambda r:r['start_path_m'])
                for left,right in zip(group,group[1:]):
                    if left['category']!=right['category']:intervals.add((pattern,float(speed),left['start_path_m'],right['start_path_m']))
    refinement=[];ordinal=0
    for pattern,speed,left,right in sorted(intervals):
        for j in range(1,10):
            for model in MODELS:refinement.append(('refinement',ordinal,pattern,model,speed,round(left+.1*j,1)))
            ordinal+=1
    if refinement:phase('refinement',refinement)
    write_json(root/'internal/refined_intervals.json',sorted(intervals))
    # 同じ条件・seedで両モデルのRF初期値が一致することを全行で照合。
    pairs={}
    for row in all_results:
        key=(row['scenario'],row['antenna_mode'],row['speed_mps'],row['start_path_m'],row['seed'])
        signature=tuple(row[k] for k in ('start_distance_m','initial_p_dbm','initial_q_dbm','observation_duration_s'))
        if key in pairs:assert pairs[key]==signature
        else:pairs[key]=signature
    assert len(pairs)*2==len(all_results)
    write_csv(root/'all_summary.csv',aggregates)
    write_json(root/'internal/verification.json',{'status':'complete','trials':len(all_results),'conditions':len(aggregates),
        'speed_conditions':len(speed_cases),'distance_coarse_conditions':len(distance_cases),'refinement_conditions':len(refinement),
        'trials_per_condition':a.trials,'matched_initial_RF_geometry_and_seed_all_pairs':True,
        'initial_connection_only':True,'censored_trials_counted_separately':True,'elapsed_s':time.monotonic()-started})
    write_json(root/'internal/progress.json',{'phase':'complete','trials':len(all_results),'elapsed_s':time.monotonic()-started})
    print(f'全工程完了: {root}',flush=True)


if __name__=='__main__':main()
