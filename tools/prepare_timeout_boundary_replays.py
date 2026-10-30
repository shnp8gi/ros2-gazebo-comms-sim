"""境界前後の代表試行を通常のGazebo経路で再現し、モニター再生を保存する。"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import csv
import fcntl
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import uuid
import yaml
from run_probability_campaign import ROOT,container_path,make_config


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path)
    p.add_argument('--jobs',type=int,default=2);a=p.parse_args();root=a.directory.resolve()
    guard=(root/'metadata/replay_execution.lock').open('a');fcntl.flock(guard,fcntl.LOCK_EX)
    targets=json.loads((root/'metadata/replay_targets.json').read_text())
    def run(target):
        model=target['model'];scenario=target['scenario'];mode=target['antenna_mode'];speed=target['speed_mps'];seed=target['seed']
        group=f'{model}_{scenario}_{mode}'
        if target.get('profile'):group=target['profile']+'_'+group
        directory=root/'replays'/group/f'{speed:.4f}mps_seed{seed}'
        input_path=directory.parent/(directory.name+'.yaml')
        directory.parent.mkdir(parents=True,exist_ok=True)
        if not (directory/'completed').exists():
            if directory.exists():raise ValueError('未完了の再生結果は上書きしません: '+str(directory))
            if target.get('config_path'):
                c=yaml.safe_load(Path(target['config_path']).read_text())
                c.update(scenario=scenario,antenna_mode=mode)
                for drone in ['drone_a','drone_b']:c[drone]['speed_kmh']=speed*3.6
            else:
                c=make_config(model,scenario,mode,speed)
            if 'start_path_m' in target:
                c['encounter_time_s']=target['start_path_m']/speed
                c['duration_s']=(target['start_path_m']+target.get('end_path_m',target['start_path_m']))/speed
                c['start_position_review']=dict(start_path_m=target['start_path_m'],source_trial=target['source_trial'])
            if model=='statistical':c['empirical_phy']['seed']=seed
            c['campaign_note']='Timeout境界の代表試行。比較は初回確立まで。通常Gazebo計算と再生用ログは全区間。'
            if target.get('replay_scope')=='initial_connection' and target['first_connected_s']>=0:
                source_duration=c['duration_s']
                c['duration_s']=min(source_duration,target['first_connected_s']+max(
                    c['simulation']['physics_step_s'],c['simulation']['log_interval_s']))
                c['simulation']['allow_pre_encounter_end']=True
                c['replay_window']=dict(scope='initial_connection',source_duration_s=source_duration,
                    source_end_path_m=target['end_path_m'],recorded_end_s=c['duration_s'])
                c['campaign_note']='初回確立までの代表再生。通信条件・軌道・seedは元試行と同じ。確立直後までを通常Gazeboで記録。'
            input_path.write_text(yaml.safe_dump(c,allow_unicode=True,sort_keys=False))
            command='source /opt/ros/humble/setup.bash && source install/no_expected_rssi/setup.bash && '+shlex.join([
                'ros2','launch','comms_sim_pkg','sim_launch.py','headless:=true','bridge:=false',
                'config_file:='+container_path(input_path),'output_dir:='+container_path(directory)])
            with input_path.with_suffix('.launch.log').open('w') as log:
                subprocess.run(['docker','run','--rm','--user',f'{os.getuid()}:{os.getgid()}',
                    '-e','HOME=/tmp','-e','DISPLAY=','-e','OMP_WAIT_POLICY=PASSIVE',
                    '-e','GZ_PARTITION=boundary-'+uuid.uuid4().hex,'-v',f'{ROOT}:/workspace','-w','/workspace',
                    'ros2-gazebo-comms-sim:humble-harmonic','bash','-c',command],stdout=log,stderr=subprocess.STDOUT,check=True)
        with (directory/'verification.log').open('w') as log:
            subprocess.run([sys.executable,str(ROOT/'tools/validate_drone_results.py'),str(directory)],stdout=log,stderr=subprocess.STDOUT,check=True)
            if model=='statistical':
                subprocess.run([sys.executable,str(ROOT/'tools/analyze_statistical_results.py'),str(directory)],stdout=log,stderr=subprocess.STDOUT,check=True)
        with (directory/'summary.csv').open() as f:summary=next(csv.DictReader(f))
        first=float(summary['first_connected_s'])
        with (directory/'timeout_events.csv').open() as f:
            timeouts=sum(first<0 or float(r['time_s'])<first for r in csv.DictReader(f))
        assert abs(first-target['first_connected_s'])<1e-9,(target,summary)
        assert timeouts==target['timeouts'],(target,timeouts)
        with (directory/'timeseries.csv').open() as stream:
            samples=list(csv.DictReader(stream))
        power_summary=dict(sampled_max_p_dbm=max(float(r['rssi_ba_dbm']) for r in samples),
                           sampled_max_q_dbm=max(float(r['rssi_ab_dbm']) for r in samples))
        if target.get('profile')=='dual60':
            difference=max(abs(float(r['rssi_ba_dbm'])-float(r['rssi_ab_dbm'])) for r in samples)
            assert difference<1e-8,('対称な無線条件でP/Q電力が不一致',difference)
            power_summary['max_pq_difference_db']=difference
        probability=yaml.safe_load((directory/'statistical_verification.yaml').read_text()) if model=='statistical' else {}
        replay=directory/'replay.sh'
        replay_options=['--stop-at-connection'] if target.get('replay_scope')=='initial_connection' and first>=0 else []
        replay.write_text('#!/usr/bin/env bash\nset -e\nexec '+shlex.join([sys.executable,str(ROOT/'tools/monitor_hrcp_signals.py'),
            str(directory/'live_signals.json'),'--replay','--autoplay','--window','full','--playback-speed','2',*replay_options])+'\n')
        replay.chmod(0o755)
        print(f"再生検証完了: {target['label']} Timeout={timeouts}",flush=True)
        saved_config=yaml.safe_load((directory/'config.yaml').read_text())
        result=dict(**target,output=str(directory),replay=str(replay),normal_gazebo_matches=True,
                    recorded_scope=saved_config.get('replay_window',{}).get('scope','full'),
                    power_summary=power_summary,checked_probability_frames=probability.get('checked_frames',0))
        (directory/'representative_verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
        return result
    with ThreadPoolExecutor(max_workers=a.jobs) as pool:results=list(pool.map(run,targets))
    index_file=root/'replays/index.json'
    if index_file.exists():
        previous=json.loads(index_file.read_text());extra_file=root/'replays/extra_index.json'
        extra=json.loads(extra_file.read_text()) if extra_file.exists() else []
        current={r['output'] for r in results}
        extras={r['output']:r for r in [*extra,*previous] if r['output'] not in current}
        extra_file.write_text(json.dumps(list(extras.values()),ensure_ascii=False,indent=2))
    temp=index_file.with_suffix('.tmp');temp.write_text(json.dumps(results,ensure_ascii=False,indent=2));temp.replace(index_file)
    (root/'analysis/replay_verification.json').write_text(json.dumps(
        dict(all_normal_gazebo_matches=all(r['normal_gazebo_matches'] for r in results),
             verified_cases=len(results),checked_probability_frames=sum(r['checked_probability_frames'] for r in results),
             results=results),ensure_ascii=False,indent=2))
    picker=root/'replays/select.py'
    picker.write_text('#!/usr/bin/env python3\nimport os,sys\nos.execv(sys.executable,'+
                      repr([sys.executable,str(ROOT/'tools/select_timeout_replays.py'),str(root)])+')\n')
    monitor=root/'monitor.sh'
    monitor.write_text('#!/usr/bin/env bash\nset -e\nexec '+shlex.join([sys.executable,str(picker)])+'\n');monitor.chmod(0o755)


if __name__=='__main__':main()
