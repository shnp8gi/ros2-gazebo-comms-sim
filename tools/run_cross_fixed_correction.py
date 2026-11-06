"""十字固定の向き訂正について、元資料から独立した再計算結果を保存する。"""
from pathlib import Path
import hashlib
import json
from raw_antenna_campaign_common import make_runs, run_batch, write_csv
from refine_raw_antenna_speed import stats, split_profiles

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'sim_results/model12_speed_campaign_20261029/raw_csv_speed_position/cross_fixed_forward'


def main():
    cases = [(p, m, 'cross', 'fixed', n / 10, 30.)
             for p in ['original', 'dual60']
             for m in ['threshold', 'statistical'] for n in range(10, 51)]
    runs = make_runs(OUT, 'forward45_speed30', cases, 20, 700000000)
    rows = split_profiles(OUT, runs, run_batch(OUT, 'forward45_speed30', runs, 6))
    summaries = []
    for profile in ['original', 'dual60']:
        summaries += stats([r for r in rows if r['profile'] == profile], profile, 'forward45')
    write_csv(OUT / 'trials.csv', rows)
    write_csv(OUT / 'speed_summary.csv', summaries)
    audit = dict(status='complete', conditions=len(summaries), trials=len(rows),
                 connected_trials=sum(int(r['connected']) for r in rows),
                 timeout_total=sum(int(r['timeouts']) for r in rows),
                 revision='forward_up_down_45deg', start_path_m=30,
                 speed_min_mps=1, speed_max_mps=5, step_mps=.1,
                 repetitions={'threshold': 1, 'statistical': 20})
    audit['sha256'] = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                       for p in [*OUT.glob('configs/*/*.yaml'),
                                 ROOT / 'build/no_expected_rssi/comms_sim_pkg/initial_connection_sweep']}
    (OUT / 'verification.json').write_text(json.dumps(audit, ensure_ascii=False, indent=2))
    (OUT / 'progress.json').write_text(json.dumps(audit, ensure_ascii=False, indent=2))
    print(json.dumps(audit, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
