#!/usr/bin/env python3
"""
記録済みの全シードに対して全手法を再生し、既存の集計に載せる。

record_runs.py が作った sim_results/<name>/seed_*/ を入力に、各手法を
replay_sim.py で再計算して runs.csv / agg.csv / paired.csv を出す。集計と
対比較は tools/lib/eval_metrics.py をそのまま使うので、実行時評価を解析した
場合と同じ計算になる。

再生は互いに独立なので並列に回せる。実行時評価と違い、**並列数を上げても
結果は変わらない** (単一プロセスで同期実行し、チャネルは記録済みのため)。

  python3 tools/replay_sweep.py sim_results/urban_rec \\
      --state-dir /workspace/sim_results/urban_learn/rem_state \\
      --out sim_results/urban_ablation_replay --jobs 8
"""
import argparse
import concurrent.futures as cf
import glob
import os
import subprocess
import sys

import pandas as pd

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TOOLS_DIR)

from lib import eval_metrics  # noqa: E402


def tuning_specs(state_dir, key, values, common=None, tag_prefix=''):
    """1つの設定値を振ったアームを作る (チューニング用)。

    記録は手法に依存しないので、同じ記録を使い回して設定だけを変えられる。
    実行時評価では設定ごとに Gazebo を回し直す必要があった。

    common は全アームに共通で渡す設定。2因子の掃引 (例: 再計画周期 × kappa) は
    片方を common に固定して2回走らせ、シードで対にして突き合わせる。
    """
    warm = ['--state-dir', state_dir]
    c = []
    for kv in (common or []):
        c += ['--kkf-set', kv]
    # 通信側の設定 (再確立コスト等) は SchedulerConfig ではなく replay_sim 側の
    # 引数なので、キーで振り分ける
    replay_flags = {'t_est_ms': '--t-est-ms'}
    out = {'assoc_hold': ['--arm', 'assoc_hold']}
    for v in values:
        tag = f'{tag_prefix}{key}_{v}'.replace('-', 'm').replace('.', 'p')
        if key in replay_flags:
            extra = [replay_flags[key], str(v)]
        else:
            extra = ['--kkf-set', f'{key}={v}']
        out[tag] = ['--arm', 'kkf'] + warm + c + extra
    return out


def arm_specs(state_dir, common=None):
    """アブレーションのアーム定義。値は replay_sim.py への追加引数。

    各アームが「何を抜いたときに何を失うか」を1つずつ切り分ける:
      kkf_cold        学習した地図の価値 (規格忠実な観測のまま)
      kkf_cold_probe  「地図を持つ」価値と「事前に学習する」価値の分離
      kkf_nolcb       LCB (不確実性を使った慎重な選択) の価値
      kkf_novar       遮蔽リスク地図 σ_ν² の価値
      oracle          全ペアの現在真値を雑音なしで使う情報上界。
                      「予測が完璧なら何点取れるか」= 地図を改良する余地の上限。
                      これが assoc_hold と並ぶなら、その構成では地図に価値がない
    """
    warm = ['--state-dir', state_dir]
    # common: 全 kkf アームに共通で渡す設定 (チューニング後の動作点で
    # アブレーションを取り直すときに使う)
    c = []
    for kv in (common or []):
        c += ['--kkf-set', kv]
    return {
        'assoc_hold': ['--arm', 'assoc_hold'],
        'kkf_conv': ['--arm', 'kkf'] + warm + c,
        'kkf_cold': ['--arm', 'kkf'] + c,
        'kkf_cold_probe': ['--arm', 'kkf', '--observe-all-pairs'] + c,
        'kkf_nolcb': ['--arm', 'kkf'] + warm + c + ['--kkf-set', 'kappa=0.0'],
        'kkf_novar': ['--arm', 'kkf'] + warm + c + ['--kkf-set', 'varmap_enabled=false'],
        'oracle': ['--arm', 'kkf', '--observe-all-pairs'] + c
                  + ['--kkf-set', 'control_plane=oracle'],
    }


def run_one(rec_dir, arm, extra, out_root, timeout):
    seed = os.path.basename(rec_dir.rstrip('/'))
    out = os.path.join(out_root, 'per_run', f'{arm}__{seed}')
    cfg = os.path.join(rec_dir, 'effective_sim_params.yaml')
    cmd = [sys.executable, os.path.join(TOOLS_DIR, 'replay_sim.py'), rec_dir,
           '--out', out, '--config', cfg] + extra
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return (arm, seed, None, 'timeout')
    if r.returncode != 0:
        return (arm, seed, None, (r.stderr or r.stdout or '')[-300:])
    csv = os.path.join(out, 'replay_summary.csv')
    if not os.path.exists(csv):
        return (arm, seed, None, 'no summary')
    return (arm, seed, csv, None)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('rec_root', help='record_runs.py の出力 (seed_* を含む)')
    ap.add_argument('--out', required=True)
    ap.add_argument('--state-dir', required=True, help='学習済み REM')
    ap.add_argument('--jobs', type=int, default=8)
    ap.add_argument('--timeout', type=int, default=3600)
    ap.add_argument('--baseline', default='kkf_conv')
    ap.add_argument('--arms', nargs='*', default=None, help='既定は全アーム')
    ap.add_argument('--tune', default=None, metavar='KEY',
                    help='SchedulerConfig の属性を振る (例 idle_lcb_db)')
    ap.add_argument('--tune-values', nargs='*', default=[],
                    help='--tune で振る値')
    ap.add_argument('--kkf-common', nargs='*', default=[], metavar='KEY=VAL',
                    help='全 kkf アームに共通で渡す設定')
    ap.add_argument('--tag-prefix', default='',
                    help='--tune のアーム名の接頭辞 (2因子掃引の突き合わせ用)')
    a = ap.parse_args()

    recs = sorted(d for d in glob.glob(os.path.join(a.rec_root, 'seed_*'))
                  if os.path.isdir(os.path.join(d, 'pairs')))
    if not recs:
        sys.exit(f"記録が見つかりません: {a.rec_root}/seed_*/pairs")
    if a.tune:
        specs = tuning_specs(a.state_dir, a.tune, a.tune_values,
                             common=a.kkf_common, tag_prefix=a.tag_prefix)
    else:
        specs = arm_specs(a.state_dir, a.kkf_common)
    if a.arms:
        specs = {k: v for k, v in specs.items() if k in a.arms}
    print(f"[replay_sweep] {len(recs)} シード x {len(specs)} 手法 "
          f"= {len(recs) * len(specs)} 再生 (並列 {a.jobs})")

    jobs = [(r, arm, extra) for r in recs for arm, extra in specs.items()]
    results, failures = [], []
    with cf.ThreadPoolExecutor(max_workers=a.jobs) as ex:
        futs = [ex.submit(run_one, r, arm, extra, a.out, a.timeout)
                for r, arm, extra in jobs]
        for i, f in enumerate(cf.as_completed(futs), 1):
            arm, seed, csv, err = f.result()
            if err:
                failures.append((arm, seed, err))
                print(f"  [{i}/{len(futs)}] NG {arm} {seed}: {err}", flush=True)
            else:
                results.append((arm, seed, csv))
                if i % 10 == 0 or i == len(futs):
                    print(f"  [{i}/{len(futs)}] 完了", flush=True)

    if failures:
        print(f"\n失敗 {len(failures)} 件:")
        for arm, seed, err in failures[:8]:
            print(f"  {arm} {seed}: {err}")

    # 走行 × 手法の表に畳む (eval_metrics が期待する runs.csv 形式)
    seeds = sorted({s for _, s, _ in results})
    run_no = {s: i + 1 for i, s in enumerate(seeds)}
    rows = []
    for arm, seed, csv in results:
        d = pd.read_csv(csv)
        tot = d[d.vehicle_name.str.endswith('_total')]
        rows.append({
            'method': arm, 'load': 'contended', 'run': run_no[seed], 'seed': seed,
            'total_data_MB': float(tot.total_data_MB.sum()),
            'connected_time_s': float(tot.connected_time_s.sum()),
            'grant_time_s': float(tot.grant_time_s.sum()),
            'connectable_time_s': float(tot.connectable_time_s.sum()),
            'min_car_data_MB': float(tot.total_data_MB.min()) if len(tot) else 0.0,
            'zero_car_pct': float((tot.total_data_MB <= 0).mean() * 100.0) if len(tot) else 0.0,
            'n_cars': int(len(tot)),
        })
    runs = pd.DataFrame(rows).sort_values(['method', 'run'])
    out_dir = os.path.join(a.out, 'analysis')
    os.makedirs(out_dir, exist_ok=True)
    runs.to_csv(os.path.join(out_dir, 'runs.csv'), index=False)

    metrics = ['total_data_MB', 'connected_time_s', 'grant_time_s',
               'min_car_data_MB', 'zero_car_pct']
    agg = eval_metrics.aggregate(runs, ['method', 'load'], metrics)
    agg.to_csv(os.path.join(out_dir, 'agg.csv'), index=False)
    if a.baseline in set(runs.method):
        pr = eval_metrics.paired(runs, a.baseline, 'method', ['load'], metrics)
        pr.to_csv(os.path.join(out_dir, 'paired.csv'), index=False)

    piv = agg.set_index('method')[['total_data_MB_mean', 'total_data_MB_ci95']]
    print(f"\n=== total_data_MB (N={runs.run.nunique()}) ===")
    print(piv.round(1).to_string())
    print(f"\n[replay_sweep] wrote {out_dir}/runs.csv, agg.csv, paired.csv")
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
