#!/usr/bin/env python3
"""290 K・P側全体NF8 dBの非対称リンクを、4軌道の速度と追従開始距離で掃引する。"""
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

import numpy as np
import yaml

PATTERNS = ('parallel_fixed', 'parallel_tracking', 'cross_fixed', 'cross_tracking')
ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path, value):
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False))
    temp.replace(path)


def write_csv(path, rows):
    temp = path.with_suffix('.tmp')
    with temp.open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    temp.replace(path)


def read_csv(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def design(smoke=False):
    cases = []
    for pattern in PATTERNS:
        for index in ([0, 100] if smoke else range(101)):
            cases.append(dict(phase='speed', pattern=pattern, speed_mps=(100+index)/100, start_path_m=20.))
    for pattern in ('parallel_tracking', 'cross_tracking'):
        for index in ([0, 50, 300] if smoke else range(301)):
            cases.append(dict(phase='distance', pattern=pattern, speed_mps=1., start_path_m=index/10))
    for index, case in enumerate(cases):
        case['case_id'] = f'case_{index:04d}'
        case['ordinal'] = index
    return cases


def prepare(output, args, binary, profile):
    for folder in ('inputs', 'configs', 'profiles', 'cases', 'internal'):
        (output/folder).mkdir(exist_ok=True)
    config_dir = ROOT/'src/comms_sim_pkg/config'
    base = yaml.safe_load((config_dir/'scenarios/drone_dual60_cross_fixed_theoretical.yaml').read_text())
    rf = yaml.safe_load((config_dir/'drone_params.yaml').read_text())
    for key in ('radio_ab', 'radio_ba', 'propagation', 'antenna'):
        base[key] = deepcopy(rf[key])
    base['radio_ba']['rx_lna_gain_db'] = 15.
    base['radio_ba']['receiver_noise_figure_db'] = 8.
    for key in ('radio_ab', 'radio_ba'):
        base[key]['receiver_model'] = 'rf_chain'
        base[key]['noise_temperature_k'] = 290.
        base[key]['required_snr_db'] = None
    base['hrcp']['monitor_enabled'] = False
    base.pop('raw_antenna_campaign', None)
    base['model_assumptions']['reporting'] = 'report_all_assumptions_and_LUT_limits'
    for name in ('e_plane.csv', 'h_plane.csv', 'tjx_theoretical_lut.yaml'):
        category = 'antennas' if name in ('e_plane.csv', 'h_plane.csv') else 'phy'
        shutil.copyfile(config_dir/category/name, output/'inputs'/name)
    for plane in ('e_plane', 'h_plane'):
        base['antenna'][plane] = str(output/'inputs'/f'{plane}.csv')
    base['theoretical_phy']['table'] = str(output/'inputs/tjx_theoretical_lut.yaml')
    base['encounter_time_s'] = 20.
    base['duration_s'] = 80.
    for pattern in PATTERNS:
        c = deepcopy(base)
        scenario, mode = pattern.split('_')
        c['scenario'], c['antenna_mode'] = scenario, mode
        for key in ('drone_a', 'drone_b'):
            c[key]['speed_kmh'] = 3.6
        c['drone_a']['fixed_azimuth_offset_deg'] = 90. if scenario == 'parallel' else -45.
        c['drone_b']['fixed_azimuth_offset_deg'] = 90. if scenario == 'parallel' else 45.
        c['antenna_mount_revision'] = 'parallel_facing_at_encounter' if scenario == 'parallel' else 'cross_facing_yaw_pitch_45deg'
        c['output_dir'] = str(output/'cases')
        path = output/'configs'/f'{pattern}.yaml'
        path.write_text(yaml.safe_dump(c, allow_unicode=True, sort_keys=False))
        # 位置診断は全開始位置を含む30m手前〜60m通過後、1m/s。
        c['encounter_time_s'], c['duration_s'] = 30., 90.
        diagnostic = output/'configs'/f'{pattern}_profile.yaml'
        diagnostic.write_text(yaml.safe_dump(c, allow_unicode=True, sort_keys=False))
        subprocess.run([str(profile), str(diagnostic), str(output/'profiles'/f'{pattern}.csv')], check=True)
    sources = list((ROOT/'src/comms_sim_pkg/src').glob('*.cpp'))
    sources += list((ROOT/'src/comms_sim_pkg/include/comms_sim_pkg').glob('*.hpp'))
    sources += [Path(__file__).resolve(), ROOT/'tools/asymmetric_link_profile.cpp', ROOT/'tools/tjx_theory/CMakeLists.txt']
    for source in sources:
        target = output/'internal/source_snapshot'/source.relative_to(ROOT)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)
    write_json(output/'internal/provenance.json', {
        'created_at': datetime.now().astimezone().isoformat(), 'binary_sha256': digest(binary),
        'profile_binary_sha256': digest(profile), 'trials': args.trials, 'seed': args.seed, 'smoke': args.smoke,
        'execution': 'native_shared_RadioModel_LinkProtocol_HrcpPairnet; no_Gazebo_process',
        'scope': 'initial_connection_only; stop_at_first_CONNECTED_or_60m_after_encounter',
        'rf_reference': 'drone_params.yaml; retain_23_24dBi_boresight_and_30dB_pattern_clipping',
        'temperature_k': 290, 'P_LNA_gain_db': 15, 'P_total_NF_db': 8, 'Q_total_NF_db': 0,
        'variance_scope': 'independent_PHY_and_MAC_seeds; fixed_LUT; excludes_LUT_estimation_uncertainty',
        'input_hashes': {str(p.relative_to(output)): digest(p) for folder in ('inputs','configs') for p in (output/folder).iterdir()},
        'git_head': subprocess.check_output(['git','rev-parse','HEAD'], cwd=ROOT, text=True).strip(),
    })


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--trials', type=int, default=30)
    parser.add_argument('--jobs', type=int, default=6)
    parser.add_argument('--seed', type=int, default=20261104)
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--smoke', action='store_true')
    args = parser.parse_args()
    if not 2 <= args.trials <= 999 or not 1 <= args.jobs <= 8 or args.seed < 0:
        parser.error('試行数2〜999、並列数1〜8、seedは0以上を指定してください')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=args.resume)
    binary = ROOT/'build/tjx_theory/initial_connection_sweep'
    profile = ROOT/'build/tjx_theory/asymmetric_link_profile'
    metadata = output/'internal/provenance.json'
    if metadata.exists():
        saved = json.loads(metadata.read_text())
        assert saved['binary_sha256'] == digest(binary)
        assert (saved['trials'], saved['seed'], saved['smoke']) == (args.trials, args.seed, args.smoke)
        for relative, expected in saved['input_hashes'].items():
            assert digest(output/relative) == expected, relative
    else:
        prepare(output, args, binary, profile)
    cases = design(args.smoke)
    write_csv(output/'conditions.csv', cases)
    print(f'{len(cases)}条件 × {args.trials}試行を開始: {output}', flush=True)
    started = time.monotonic()

    def evaluate(case):
        directory = output/'cases'/case['case_id']
        directory.mkdir(exist_ok=True)
        scenario, mode = case['pattern'].split('_')
        jobs = [dict(id=f"{case['case_id']}_{i:03d}", model='theoretical', scenario=scenario, antenna_mode=mode,
                     config=str(output/'configs'/f"{case['pattern']}.yaml"), speed_mps=case['speed_mps'],
                     start_path_m=case['start_path_m'], end_path_m=60., seed=args.seed+case['ordinal']*1000+i,
                     mac_seed=(args.seed+case['ordinal']*1000+i)^0x12345678) for i in range(args.trials)]
        manifest = directory/'manifest.yaml'
        if manifest.exists():
            assert yaml.safe_load(manifest.read_text()) == {'runs': jobs}
        else:
            manifest.write_text(yaml.safe_dump({'runs': jobs}, sort_keys=False))
        result = directory/'trials.csv'
        if not result.exists():
            partial = directory/'trials.partial.csv'
            if partial.exists():
                partial.rename(directory/f'interrupted_{time.time_ns()}.csv')
            with (directory/'execution.log').open('a') as log:
                subprocess.run([str(binary),str(manifest),str(partial)], cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                               check=True, env=dict(os.environ, OMP_NUM_THREADS='1', OMP_WAIT_POLICY='PASSIVE'))
            assert len(read_csv(partial)) == args.trials
            partial.replace(result)
        rows = read_csv(result)
        assert len(rows) == args.trials
        for row, job in zip(rows, jobs):
            for key in ('id','scenario','antenna_mode'):
                assert row[key] == job[key], (case['case_id'],key)
            for key in ('speed_mps','start_path_m','seed'):
                assert float(row[key]) == job[key]
            assert math.isclose(float(row['observation_duration_s']), (case['start_path_m']+60)/case['speed_mps'])
            assert int(row['connected']) == int(float(row['first_connected_s']) >= 0)
            assert math.isclose(float(row['initial_q_snr_db'])-float(row['initial_p_snr_db']), 8.984781380449016, abs_tol=1e-8)
            for side in ('p','q'):
                n = int(row[f'{side}_frames'])
                assert 0 <= int(row[f'{side}_failed_frames']) <= n
                if n:
                    assert 0 <= float(row[f'{side}_mean_per']) <= 1
            row.update(case_id=case['case_id'],phase=case['phase'],pattern=case['pattern'])
        connected = [r for r in rows if int(r['connected'])]
        counts = [int(r['timeouts']) for r in rows]
        summary = {k:v for k,v in case.items() if k != 'ordinal'}
        summary.update(trials=len(rows), connected_trials=len(connected), connected_fraction=len(connected)/len(rows),
                       zero_timeout_connected_trials=sum(int(r['timeouts'])==0 for r in connected),
                       mean_timeouts=statistics.mean(counts), stddev_timeouts=statistics.pstdev(counts),
                       min_timeouts=min(counts), max_timeouts=max(counts),
                       mean_first_connected_s=statistics.mean(float(r['first_connected_s']) for r in connected) if connected else '',
                       initial_p_snr_db=float(rows[0]['initial_p_snr_db']), initial_q_snr_db=float(rows[0]['initial_q_snr_db']),
                       initial_distance_m=float(rows[0]['start_distance_m']))
        summary['zero_timeout_connected_fraction'] = summary['zero_timeout_connected_trials']/len(rows)
        for side in ('p','q'):
            count = sum(int(r[f'{side}_frames']) for r in rows)
            failed = sum(int(r[f'{side}_failed_frames']) for r in rows)
            summary[f'{side}_frames'] = count
            summary[f'{side}_failed_fraction'] = failed/count if count else ''
            summary[f'{side}_mean_per'] = sum(float(r[f'{side}_mean_per'])*int(r[f'{side}_frames']) for r in rows if int(r[f'{side}_frames']))/count if count else ''
        summary['lut_outside_frames'] = sum(int(r['lut_outside_frames']) for r in rows)
        return rows, summary

    rows, summaries = [], []
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        futures = [pool.submit(evaluate, case) for case in cases]
        for future in as_completed(futures):
            trial_rows, summary = future.result()
            rows.extend(trial_rows)
            summaries.append(summary)
            if len(summaries)%10 == 0 or len(summaries)==len(cases):
                elapsed = time.monotonic()-started
                print(f'{len(summaries)}/{len(cases)}条件完了、経過{elapsed:.1f}秒', flush=True)
                write_json(output/'internal/progress.json',dict(completed_conditions=len(summaries),total_conditions=len(cases),elapsed_s=elapsed))
                write_csv(output/'partial_summary.csv', sorted(summaries,key=lambda r:r['case_id']))
    summaries.sort(key=lambda r:r['case_id'])
    rows.sort(key=lambda r:r['id'])
    write_csv(output/'all_trials.csv',rows)
    write_csv(output/'all_summary.csv',summaries)
    for phase in ('speed','distance'):
        write_csv(output/f'{phase}_summary.csv',[r for r in summaries if r['phase']==phase])
    # RF診断でも約9dB差と十字固定の正対姿勢を照合。
    for pattern in PATTERNS:
        data = np.genfromtxt(output/'profiles'/f'{pattern}.csv', delimiter=',', names=True)
        assert len(data)==9001 and np.allclose(data['q_snr_db']-data['p_snr_db'],8.984781380449016,atol=1e-8,rtol=0)
        assert np.all((data['p_control_per']>=0)&(data['p_control_per']<=1))
        if pattern=='cross_fixed':
            closest = np.argmin(abs(data['remaining_path_m']-10/math.sqrt(2)))
            assert data['off_axis_a_deg'][closest] < .01 and data['off_axis_b_deg'][closest] < .01
        if pattern.endswith('tracking'):
            assert np.max(data['off_axis_a_deg']) < 1e-5 and np.max(data['off_axis_b_deg']) < 1e-5
    assert len(rows)==len(cases)*args.trials and len({r['id'] for r in rows})==len(rows)
    write_json(output/'internal/verification.json',dict(status='complete',conditions=len(cases),trials=len(rows),
        trials_per_condition=args.trials,speed_conditions=sum(c['phase']=='speed' for c in cases),
        distance_conditions=sum(c['phase']=='distance' for c in cases),initial_connection_only=True,
        all_case_counts_seeds_durations_checked=True,SNR_gap_all_profiles_checked=True,
        crossing_fixed_facing_checked=True,tracking_pointing_checked=True,elapsed_s=time.monotonic()-started))
    print(f'完了: {output}',flush=True)


if __name__ == '__main__':
    main()
