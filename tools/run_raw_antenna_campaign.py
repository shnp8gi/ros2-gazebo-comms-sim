"""CSV利得を補正・打切りなしで使い、速度×開始位置の実験を順次実行する。"""
import argparse
import copy
from datetime import datetime
import fcntl
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time
import yaml
from run_probability_campaign import ROOT, container_path


def save(path, value):
    temp=path.with_suffix(path.suffix+'.tmp')
    temp.write_text(json.dumps(value,ensure_ascii=False,indent=2));temp.replace(path)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path)
    p.add_argument('--jobs',type=int,default=6);p.add_argument('--resume',action='store_true')
    p.add_argument('--report-template',type=Path,required=True);a=p.parse_args()
    if not 1<=a.jobs<=8:p.error('jobsは1〜8です')
    root=a.directory.resolve();root.relative_to(ROOT)
    root.mkdir(parents=True,exist_ok=a.resume)
    for name in ['metadata','configs','inputs','analysis','speed_sweeps']:(root/name).mkdir(exist_ok=a.resume)
    guard=(root/'metadata/execution.lock').open('a');fcntl.flock(guard,fcntl.LOCK_EX|fcntl.LOCK_NB)
    index_path=root/'index.json'
    if index_path.exists():
        index=json.loads(index_path.read_text())
    else:
        source=root.parent/'start_position_5mps_path/metadata'
        bases={m:yaml.safe_load((source/f'{m}.yaml').read_text()) for m in ['threshold','statistical']}
        for name in ['e_plane.csv','h_plane.csv','MCStable.csv','receiver_curve_measured_provisional.csv',
                     'receiver_curve_measured_provisional.csv.provenance.json']:
            shutil.copy2(ROOT/'src/comms_sim_pkg/config'/name,root/'inputs'/name)
        for profile in ['original','dual60']:
            (root/'configs'/profile).mkdir()
            for model,base in bases.items():
                c=copy.deepcopy(base)
                for direction in ['radio_ab','radio_ba']:
                    for key in ['tx_antenna_gain_dbi','rx_antenna_gain_dbi']:c[direction].pop(key,None)
                c['antenna'].update(max_attenuation_db=None,
                    e_plane=container_path(root/'inputs/e_plane.csv'),h_plane=container_path(root/'inputs/h_plane.csv'))
                c['mcs_table']=container_path(root/'inputs/MCStable.csv')
                if model=='statistical':c['empirical_phy']['table']=container_path(root/'inputs/receiver_curve_measured_provisional.csv')
                if profile=='dual60':
                    radio=copy.deepcopy(c['radio_ab'])
                    radio.update(frequency_hz=60e9,power_mw=10**(-7/10),
                        tx_insertion_loss_db=base['radio_ab']['tx_insertion_loss_db'],rx_lna_gain_db=0.,
                        rx_conversion_gain_db=base['radio_ba']['rx_conversion_gain_db'],
                        rx_insertion_loss_db=base['radio_ba']['rx_insertion_loss_db'],
                        receiver_noise_figure_db=base['radio_ba']['receiver_noise_figure_db'])
                    c['radio_ab']=copy.deepcopy(radio);c['radio_ba']=copy.deepcopy(radio)
                c['campaign_note']='生CSV合成利得、正面利得補正なし、減衰打切りなし。初回リンク確立までのTimeout。'
                c['raw_antenna_campaign']=dict(profile=profile,boresight_override=False,attenuation_clipping=False)
                (root/'configs'/profile/f'{model}.yaml').write_text(yaml.safe_dump(c,allow_unicode=True,sort_keys=False))
        components=[]
        for speed in [5,4,3,2,1]:
            for profile in ['original','dual60']:
                components.append(dict(profile=profile,speed_mps=speed,status='pending',
                    output=str(root/'speed_sweeps'/profile/f'{speed:02}mps')))
        index=dict(status='ready',phase='prepare',started_at=datetime.now().astimezone().isoformat(),
            jobs=a.jobs,components=components,models=['threshold','statistical'],pattern_count=5,
            start_position_axis='path_to_closest',coarse_position_step_m=1,fine_position_step_m=.1,
            max_start_path_m=60,end_path_m=60,coarse_repetitions=20,fine_repetitions=600,
            boresight_override=False,attenuation_clipping=False,report_template=str(a.report_template.resolve()))
        checksums={str(f.relative_to(root)):hashlib.sha256(f.read_bytes()).hexdigest()
                   for folder in ['configs','inputs'] for f in (root/folder).rglob('*') if f.is_file()}
        save(root/'metadata/input_sha256.json',checksums)
        shutil.copy2(a.report_template,root/'metadata/report_format_reference.docx')
        save(index_path,index)
    for path,digest in json.loads((root/'metadata/input_sha256.json').read_text()).items():
        assert hashlib.sha256((root/path).read_bytes()).hexdigest()==digest,path
    def phase(name):
        index.update(status='running',phase=name,updated_at=datetime.now().astimezone().isoformat());save(index_path,index)
        print('工程:',name,flush=True)
    try:
        phase('position_sweeps')
        for component in index['components']:
            if component['status']=='complete':continue
            output=Path(component['output']);profile=component['profile'];speed=component['speed_mps']
            component.update(status='running',started_epoch=time.time());save(index_path,index)
            command=[sys.executable,str(ROOT/'tools/run_start_position_sweep.py'),str(output),
                '--speed',str(speed),'--base-directory',str(root/'configs'/profile),'--max-position','60',
                '--jobs',str(a.jobs),'--seed-start',str((100000000 if profile=='original' else 200000000)+speed*10000000),
                '--skip-plots']
            if profile=='dual60':command+=['--patterns','cross_fixed']
            if output.exists():command+=['--resume']
            with (root/'metadata'/f'{profile}_{speed:02}mps.log').open('a') as log:
                subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True)
            result=json.loads((output/'progress.json').read_text());assert result['status']=='complete'
            component.update(status='complete',trials=result['trials'],conditions=result['conditions'],finished_epoch=time.time())
            save(index_path,index);print('速度・無線条件の計算完了:',profile,speed,'m/s',result['trials'],'試行',flush=True)
        phase('fixed_speed_refinement')
        subprocess.run([sys.executable,str(ROOT/'tools/refine_raw_antenna_speed.py'),str(root),'--jobs',str(a.jobs)],check=True)
        phase('plots_and_summary')
        subprocess.run([sys.executable,str(ROOT/'tools/summarize_raw_antenna_campaign.py'),str(root)],check=True)
        phase('gazebo_replays')
        subprocess.run([sys.executable,str(ROOT/'tools/prepare_timeout_boundary_replays.py'),str(root),'--jobs','4'],check=True)
        phase('report')
        subprocess.run([sys.executable,str(ROOT/'tools/write_raw_antenna_report.py'),str(root),str(root/'metadata/report_format_reference.docx')],check=True)
        index.update(status='complete',phase='complete',finished_at=datetime.now().astimezone().isoformat());save(index_path,index)
        print('計算・図・再生検証・レポートの全工程完了:',root,flush=True)
    except Exception as exc:
        index.update(status='failed',error=str(exc));save(index_path,index);raise


if __name__=='__main__':main()
