"""追従の開始位置を、Q受信しきい値へ達する指定秒数前にそろえる。"""
import argparse
import csv
import json
import math
from pathlib import Path
import shlex
import sys
import yaml
from run_probability_campaign import ROOT,make_config,container_path


def configure(model,scenario,speed,lead):
    c=make_config(model,scenario,'tracking',speed)
    r=c['radio_ab']
    budget=(10*math.log10(r['power_mw'])-r['tx_insertion_loss_db']+
            r['tx_antenna_gain_dbi']+r['rx_antenna_gain_dbi']+r['rx_lna_gain_db']+
            r['rx_conversion_gain_db']-r['rx_insertion_loss_db'])
    loss_at_one_m=20*math.log10(4*math.pi*float(r['frequency_hz'])/299792458)
    distance=10**((budget-r['sensitivity_dbm']-loss_at_one_m)/(10*c['propagation']['exponent']))
    height=c['drone_b']['altitude_m']-c['drone_a']['altitude_m']
    lateral=c['lateral_separation_m'] if scenario=='parallel' else 0
    along_squared=distance**2-height**2-lateral**2
    if along_squared<=0:raise ValueError('最接近位置でもQがしきい値へ達しません')
    path_to_closest=math.sqrt(along_squared/(4 if scenario=='parallel' else 2))
    c['encounter_time_s']=lead+path_to_closest/speed
    c['duration_s']=2*c['encounter_time_s']
    start_path=path_to_closest+speed*lead
    start_distance=math.sqrt((4 if scenario=='parallel' else 2)*start_path**2+height**2+lateral**2)
    c['tracking_start']=dict(policy='q_threshold_minus_lead_time',q_threshold_lead_s=lead,
        q_threshold_dbm=r['sensitivity_dbm'],q_threshold_distance_m=distance,
        initial_distance_m=start_distance,distance_before_encounter_m=start_path,
        note='接近側でQがしきい値に達する5秒前から開始。P受信可能な開始を許容。②の成功確率しきい値ではなく計算RSSIを基準にする。')
    c['campaign_note']='Qの計算RSSIしきい値到達を開始5秒後に固定。待機時間・RF・MCS・MACは従来設定。'
    return c


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('directory',type=Path);p.add_argument('--lead-seconds',type=float,default=5)
    a=p.parse_args()
    if a.lead_seconds<=0:p.error('開始前の秒数は正値にしてください')
    root=a.directory.resolve();root.mkdir(parents=True,exist_ok=False)
    (root/'configs').mkdir();(root/'analysis').mkdir()
    runs=[]
    for model in ['threshold','statistical']:
        for scenario in ['parallel','cross']:
            for speed in range(1,11):
                name=f'{model}_{scenario}_tracking_{speed:02}mps'
                c=configure(model,scenario,speed,a.lead_seconds)
                c['output_dir']=container_path(root/'runs'/name)
                path=root/'configs'/f'{name}.yaml'
                path.write_text(yaml.safe_dump(c,allow_unicode=True,sort_keys=False))
                runs.append(dict(name=name,model=model,scenario=scenario,speed_mps=speed,
                    config=container_path(path),**c['tracking_start'],duration_s=c['duration_s'],encounter_time_s=c['encounter_time_s']))
    (root/'index.json').write_text(json.dumps(dict(status='configured',policy='q_threshold_minus_lead_time',
        q_threshold_lead_s=a.lead_seconds,runs=runs),ensure_ascii=False,indent=2))
    with (root/'analysis/start_positions.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(runs[0]));w.writeheader();w.writerows(runs)
    # 指定した新条件を通常のGazeboで実行し、記録済みなら再生する入口。
    launcher=root/'run.py'
    launcher.write_text('''#!/usr/bin/env python3
import argparse,json,os,shlex,subprocess,sys,uuid
from pathlib import Path
root=Path(__file__).resolve().parent
project=next(p for p in root.parents if (p/'src/comms_sim_pkg').exists())
p=argparse.ArgumentParser(description='Qしきい値の5秒手前から実行・再生')
p.add_argument('model',choices=['1','2']);p.add_argument('scenario',choices=['parallel','cross']);p.add_argument('speed',type=int,choices=range(1,11))
a=p.parse_args();model='threshold' if a.model=='1' else 'statistical'
r=next(r for r in json.loads((root/'index.json').read_text())['runs'] if r['model']==model and r['scenario']==a.scenario and r['speed_mps']==a.speed)
output=Path(r['output']) if r.get('output') else root/'runs'/r['name']
if not (output/'completed').exists():
 if output.exists():raise RuntimeError('未完了の記録が存在するため上書きしません: '+str(output))
 command='source /opt/ros/humble/setup.bash && source install/no_expected_rssi/setup.bash && '+shlex.join(['ros2','launch','comms_sim_pkg','sim_launch.py','headless:=true','bridge:=false','config_file:='+r['config'],'output_dir:=/workspace/'+str(output.relative_to(project))])
 subprocess.run(['docker','run','--rm','--user',f'{os.getuid()}:{os.getgid()}','-e','HOME=/tmp','-e','DISPLAY=','-e','OMP_WAIT_POLICY=PASSIVE','-e','GZ_PARTITION=q-lead-'+uuid.uuid4().hex,'-v',f'{project}:/workspace','-w','/workspace','ros2-gazebo-comms-sim:humble-harmonic','bash','-c',command],check=True)
 if not (output/'completed').exists():raise RuntimeError('計算完了マーカーがありません')
 subprocess.run([sys.executable,str(project/'tools/validate_drone_results.py'),str(output)],check=True)
subprocess.run([sys.executable,str(project/'tools/monitor_hrcp_signals.py'),str(output/'live_signals.json'),'--replay','--autoplay','--window','full'],check=True)
''')
    print('追従40条件の開始位置と実行・再生入口を保存:',root)


if __name__=='__main__':main()
