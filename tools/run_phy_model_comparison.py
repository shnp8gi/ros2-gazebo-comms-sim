#!/usr/bin/env python3
"""飛行・RF・MAC条件をそろえて理論/実測PERモデルを各30試行する。"""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from copy import deepcopy
import csv
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import statistics
import subprocess
import time
import numpy as np
import yaml


def main():
    root=Path(__file__).resolve().parents[1]
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=root/f'sim_results/phy_comparison_{datetime.now():%Y%m%d_%H%M%S}')
    parser.add_argument('--trials',type=int,default=30)
    parser.add_argument('--jobs',type=int,default=6)
    parser.add_argument('--seed',type=int,default=20261102)
    args=parser.parse_args()
    if args.trials<2 or args.jobs<1 or args.seed<0 or args.seed+args.trials>=2**64:
        parser.error('試行数・並列数・seedが範囲外です')
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=False)
    internal=output/'internal';internal.mkdir();inputs=internal/'inputs';inputs.mkdir()
    directory=root/'src/comms_sim_pkg/config/scenarios'
    theory=yaml.safe_load((directory/'drone_dual60_cross_fixed_theoretical.yaml').read_text())
    old=yaml.safe_load((directory/'drone_dual60_cross_fixed_statistical.yaml').read_text())
    empirical=deepcopy(theory)
    empirical.pop('theoretical_phy');empirical.pop('model_assumptions',None)
    empirical['empirical_phy']=deepcopy(old['empirical_phy'])
    empirical['mcs_table']=old['mcs_table']
    empirical['measurement_assumptions']=old['measurement_assumptions']
    configurations={'empirical':empirical,'theoretical':theory}
    common_keys=('scenario','antenna_mode','drone_a','drone_b','lateral_separation_m','encounter_time_s',
                 'duration_s','radio_ab','radio_ba','propagation','antenna','protocol','simulation','hrcp','awgn','lab_phy')
    assert all(theory[k]==empirical[k] for k in common_keys)
    provenance={'git_branch':subprocess.check_output(['git','branch','--show-current'],cwd=root,text=True).strip(),
        'git_head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip(),
        'execution':'native_shared_geometry_RadioModel_HrcpPairnet; no_Gazebo_process',
        'seeds':list(range(args.seed,args.seed+args.trials)),
        'scope':'same_geometry_RF_MAC; receiver_probability_MCS_selection_and_PHY_timing_are_model_specific',
        'inputs':{}}
    for model,config in configurations.items():
        config['hrcp']['monitor_enabled']=False
        files=[(config['antenna'],'e_plane'),(config['antenna'],'h_plane')]
        files+=([(config,'mcs_table'),(config['empirical_phy'],'table')] if model=='empirical' else [(config['theoretical_phy'],'table')])
        for mapping,key in files:
            source=(directory/mapping[key]).resolve();target=inputs/source.name
            if target.exists():assert target.read_bytes()==source.read_bytes()
            else:shutil.copyfile(source,target)
            mapping[key]=str(target)
            provenance['inputs'][target.name]={'source':str(source),'sha256':hashlib.sha256(target.read_bytes()).hexdigest()}
        config['output_dir']=str(output/model)
        (internal/f'{model}.yaml').write_text(yaml.safe_dump(config,allow_unicode=True,sort_keys=False))
    common={k:theory[k] for k in common_keys}
    (internal/'common_conditions.yaml').write_text(yaml.safe_dump(common,allow_unicode=True,sort_keys=False))
    (internal/'comparison_design.json').write_text(json.dumps({'models':list(configurations),'trials_per_model':args.trials,
        'verified_equal_config_keys':list(common_keys),'same_seed_schedule':True,
        'differences':['receiver_probability_source','MCS_selection','PHY_frame_duration'],
        'report_policy':'calibration/implementation/seed/runtime details internal only; antenna gain detail in report'},ensure_ascii=False,indent=2))
    binary=root/'build/tjx_theory/tjx_theory_run'
    provenance['binary_sha256']=hashlib.sha256(binary.read_bytes()).hexdigest()
    for source in list((root/'src/comms_sim_pkg/src').glob('*.cpp'))+list((root/'src/comms_sim_pkg/include/comms_sim_pkg').glob('*.hpp'))+[Path(__file__).resolve()]:
        target=internal/'source_snapshot'/source.relative_to(root);target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(source,target)
    (internal/'provenance.json').write_text(json.dumps(provenance,ensure_ascii=False,indent=2))
    started=time.monotonic();rows=[]

    def run(model,index):
        target=output/model/f'trial_{index:02d}'
        target.parent.mkdir(exist_ok=True)
        with (internal/f'{model}_{index:02d}.log').open('w') as log:
            subprocess.run([str(binary),str(internal/f'{model}.yaml'),str(target),str(args.seed+index)],cwd=root,
                           stdout=log,stderr=subprocess.STDOUT,check=True,env=dict(os.environ,OMP_NUM_THREADS='1'))
        summary=yaml.safe_load((target/'summary.yaml').read_text())
        series=np.genfromtxt(target/'timeseries.csv',delimiter=',',names=True)
        assert len(series)==9001 and series['time_s'][-1]==90
        assert np.all(np.diff(series['total_data_MiB'])>=0)
        assert np.isclose(series['total_data_MiB'][-1],summary['total_data_MiB'],atol=1e-9,rtol=0)
        events=list(csv.DictReader((target/'events.csv').open()))
        for metric,event in [('connections','CONNECTED'),('timeouts','TIMEOUT'),('disconnections','DISCONNECTED')]:
            assert summary[metric]==sum(e['event']==event for e in events)
        connected=0.;since=None
        for e in events:
            if e['event']=='CONNECTED':assert since is None;since=float(e['time_s'])
            elif e['event']=='DISCONNECTED':assert since is not None;connected+=float(e['time_s'])-since;since=None
        if since is not None:connected+=90-since
        assert abs(connected-summary['connected_time_s'])<1e-7
        return dict(model_key=model,trial=index,**summary)

    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        futures=[pool.submit(run,model,index) for index in range(args.trials) for model in configurations]
        for future in as_completed(futures):
            row=future.result();rows.append(row)
            print(f"完了 {len(rows)}/{2*args.trials}: {row['model_key']} trial={row['trial']} {row['total_data_MiB']:.3f} MiB",flush=True)
            (internal/'progress.json').write_text(json.dumps({'completed':len(rows),'total':2*args.trials,'elapsed_s':time.monotonic()-started}))
    rows.sort(key=lambda r:(r['model_key'],r['trial']))
    with (output/'trials.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    metrics=('total_data_MiB','active_transfer_bins_s','transfer_bin_span_s','connected_time_s','mean_goodput_Mbps',
             'first_connected_s','first_transfer_bin_start_s','last_transfer_bin_end_s','timeouts','connections','disconnections')
    stats={}
    for model in configurations:
        stats[model]={}
        for metric in metrics:
            values=[r[metric] for r in rows if r['model_key']==model]
            stats[model][metric]={'mean':statistics.mean(values),'variance':statistics.pvariance(values),
                'sample_variance':statistics.variance(values),'stddev':statistics.pstdev(values),'min':min(values),'max':max(values)}
    # 全試行・両モデルでRF入力が同一であることを数値時系列でも照合する。
    reference=np.genfromtxt(output/'theoretical/trial_00/timeseries.csv',delimiter=',',names=True)
    rf_fields=('time_s','rssi_ab_dbm','rssi_ba_dbm','snr_ab_db','snr_ba_db','gain_a_db','gain_b_db','off_axis_a_deg','off_axis_b_deg','distance_m')
    for model in configurations:
        for index in range(args.trials):
            data=np.genfromtxt(output/model/f'trial_{index:02d}/timeseries.csv',delimiter=',',names=True)
            for key in rf_fields:assert np.array_equal(reference[key],data[key]),(model,index,key)
    (output/'aggregate.json').write_text(json.dumps({'trials_per_model':args.trials,'statistics':stats},ensure_ascii=False,indent=2))
    (internal/'verification.json').write_text(json.dumps({'status':'complete','trials':len(rows),'duration_each_s':90,
        'identical_RF_and_geometry_all_trials':True,'verified_series_columns':rf_fields,
        'events_match_all_counters':True,'events_match_connected_time':True,'monotone_delivery':True,
        'elapsed_s':time.monotonic()-started},ensure_ascii=False,indent=2))
    print(f'比較シミュレーション完了: {output}',flush=True)


if __name__=='__main__':main()
