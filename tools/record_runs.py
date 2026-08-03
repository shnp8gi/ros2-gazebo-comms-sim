#!/usr/bin/env python3
"""
事後再計算 (tools/replay_sim.py) の入力を、シードごとに記録する。

記録するのは軌跡と全ペア RSSI だけで、どちらも**手法に依存しない**:
  姿勢  … 運動のみに依存 (VehicleMotionController は通信を参照しない)
  RSSI  … 幾何のみに依存 (記録モードでは grant を見ずに評価する)

したがって記録は「シードごとに1回」で足り、全手法がその1回を使い回せる。
実行時評価では「手法 × シード」だけ Gazebo を回す必要があったのに対し、
Gazebo 実行が手法の数だけ減る。

記録は判断に依存しないので、並列数を上げても結果は変わらない (KKF アームだけ
が負荷で不利になっていた交絡は、記録には存在しない)。

  python3 tools/record_runs.py --scenario config/scenarios/road_urban_2lane.yaml \\
      --name urban_rec --runs 20 --load contended --concurrency 8
"""
import argparse
import datetime
import os
import subprocess
import sys

import yaml

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(TOOLS_DIR)
IN_CONTAINER = os.path.exists('/.dockerenv')

# 競合条件 (RSU が需要に対して希少になる設定)。中負荷では手法間に差が出ない
# ことが分かっているので、既定は競合条件にする
LOAD_PRESETS = {
    'contended': {
        'traffic.lanes[0].speed_mps': 7.0,
        'traffic.lanes[1].speed_mps': 7.0,
        'traffic.lanes[0].headway_s.mean': 2.0,
        'traffic.lanes[1].headway_s.mean': 2.0,
        'traffic.target_ratio': 0.8,
        'traffic.window_s': [0.0, 35.0],
    },
    'moderate': {},
}


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--scenario', required=True)
    ap.add_argument('--name', required=True, help='記録の格納先 sim_results/<name>')
    ap.add_argument('--runs', type=int, default=20)
    ap.add_argument('--base-seed', type=int, default=12345)
    ap.add_argument('--seed-stride', type=int, default=1000)
    ap.add_argument('--load', default='contended', choices=sorted(LOAD_PRESETS))
    ap.add_argument('--concurrency', type=int, default=8)
    ap.add_argument('--period-s', type=float, default=0.005,
                    help='記録周期 [s] (通信更新レートに合わせる)')
    ap.add_argument('--timeout', type=int, default=5400)
    ap.add_argument('--start-run', type=int, default=1,
                    help='途中から再開するとき')
    a = ap.parse_args()

    root = os.path.join('sim_results', a.name)
    os.makedirs(os.path.join(REPO_ROOT, root), exist_ok=True)
    started = datetime.datetime.now()

    seed_tag = "{seed}"
    rec = f"/workspace/{root}/seed_{seed_tag}"
    # 記録には制御プレーンを使わない。何を記録するかは手法に依存しないので、
    # 最も軽い構成 (プラグイン内で完結し外部ノードを立てない) を選ぶ
    case_cfg = {
        'link_controller_node.ros__parameters.scheduling_policy': 'assoc_hold',
        'link_controller_node.ros__parameters.control_plane': 'none',
        'comms_simulator_node.ros__parameters.measurement_report.enabled': False,
        'comms_simulator_node.ros__parameters.record_pairs_dir': f"{rec}/pairs",
    }
    scen = dict(LOAD_PRESETS[a.load])
    scen['simulation_overrides.record_poses_path'] = f"{rec}/poses.csv"
    scen['simulation_overrides.record_poses_period_s'] = a.period_s

    sweep = {'sweep': {
        'name': a.name,
        'scenario': a.scenario,
        'variables': [
            {'name': 'method', 'cases': {'record': {'config': case_cfg}}},
            {'name': 'load', 'cases': {a.load: {'scenario': scen} if scen else {}}},
        ],
        'analysis': {'group_by': ['method', 'load'], 'arm_var': 'method',
                     'baseline': 'record', 'metrics': ['total_data_MB']},
        'execution': {
            'num_runs': a.runs,
            'base_seed': a.base_seed,
            'task_timeout_sec': a.timeout,
            'real_time_factor': 2.0,
            'max_concurrency': max(1, a.concurrency),
            'output_name': f"{a.name}/sweep",
        },
    }}
    path = os.path.join('scratch', f"record_{a.name}.yaml")
    with open(os.path.join(REPO_ROOT, path), 'w', encoding='utf-8') as f:
        yaml.dump(sweep, f, sort_keys=False, allow_unicode=True)

    print(f"[record] {a.runs} シード x {a.load} を並列 {a.concurrency} で記録", flush=True)
    rc = subprocess.call([sys.executable, os.path.join(TOOLS_DIR, 'sim.py'),
                          'run', '--sweep-config', path], cwd=REPO_ROOT)
    if rc != 0:
        print(f"[record] 失敗 (exit {rc})", file=sys.stderr)
        return rc

    print(f"\n[record] 完了: {root}/seed_* "
          f"({datetime.datetime.now() - started})")
    print("[record] 次: python3 tools/replay_sweep.py で全手法を再生")
    return 0


if __name__ == '__main__':
    sys.exit(main())
