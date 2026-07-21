#!/usr/bin/env python3
"""
cases / entity_groups 記法と manifest 生成の単体テスト。コンテナ内で実行:
  docker compose exec -T sim python3 /workspace/tools/tests/sweep_manifest_test.py
"""
import os
import sys
import tempfile

import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import lib.sweep_config as sweep_config
from lib import manifest as eval_manifest
from lib.scenario_loader import generate_sim_params, load_scenario
from lib.task_formatting import build_logging_strings

PASS = FAIL = 0


def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  ok    {name}")
    else:
        FAIL += 1
        print(f"  FAIL  {name}  {detail}")


SWEEP_YAML = """
sweep:
  name: test
  scenario: config/scenarios/road_multicar.yaml
  variables:
    - name: method
      cases:
        lut:
          config:
            link_controller_node.ros__parameters.scheduling_policy: feedforward_optimal
            comms_simulator_node.ros__parameters.measurement_report.enabled: false
        kkf_full:
          config:
            link_controller_node.ros__parameters.scheduling_policy: external_schedule
            link_controller_node.ros__parameters.control_plane: kkf_mpc
          entities:
            truck_co_0: { enabled: true }
        baseline_empty: {}
    - name: density
      entity_groups:
        "0": []
        "2": [truck_co_0, truck_opp_0]
        "4": [truck_co_0, truck_opp_0, truck_co_1, truck_opp_1]
  execution:
    num_runs: 3
    base_seed: 777
"""


def test_cases_and_groups():
    print("[1] cases / entity_groups の展開")
    with tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False) as f:
        f.write(SWEEP_YAML)
        path = f.name
    try:
        sweep_config.load_sweep_config(path)
        gvars = sweep_config.GENERIC_VARIABLES
        check("変数2個", len(gvars) == 2, f"got {len(gvars)}")

        method = gvars[0]
        check("methodのstate数=3", len(method['states']) == 3)
        lut = method['states'][0]
        check("lutラベル", lut[0].get('state_label') == 'lut')
        check("lutのconfig_path上書き2件",
              sum(1 for o in lut if o.get('config_path')) == 2)
        kkf = method['states'][1]
        ent = [o for o in kkf if o.get('entity_name')]
        check("kkf_fullのentities上書き",
              len(ent) == 1 and ent[0]['field'] == 'enabled' and ent[0]['value'] is True)
        empty = method['states'][2]
        check("空ケースはno-op+ラベル",
              len(empty) == 1 and empty[0]['state_label'] == 'baseline_empty'
              and empty[0].get('config_path') is None)

        density = gvars[1]
        check("densityのstate数=3", len(density['states']) == 3)
        d0 = density['states'][0]
        check("d0: 制御対象4体すべてenabled=false",
              len(d0) == 4 and all(o['value'] is False for o in d0))
        d2 = density['states'][1]
        on = {o['entity_name'] for o in d2 if o['value'] is True}
        off = {o['entity_name'] for o in d2 if o['value'] is False}
        check("d2: 2体on/2体off",
              on == {'truck_co_0', 'truck_opp_0'}
              and off == {'truck_co_1', 'truck_opp_1'})

        # ファイル名・ログ文字列にラベルが使われるか
        task_vars = {'method': method['states'][1], 'density': density['states'][1]}
        suffix, params = build_logging_strings(task_vars)
        check("ログ文字列にラベル", suffix == 'method_kkf_full_density_2', suffix)

        # 実シナリオへ適用して enabled 制御が効くか (d2: トラック2台のみ)
        scenario = load_scenario('config/scenarios/road_multicar.yaml')
        overrides = []
        for so in task_vars.values():
            overrides.extend(so)
        cfg = generate_sim_params(scenario, overrides)
        # 制御対象 (co_0/opp_0/co_1/opp_1) のうち d2 グループ外は無効化され、
        # entity_groups に載らないエンティティ (co_2以降) は触らない
        blockers = set(cfg.get('blocker_entities', {}).keys())
        check("d2適用: グループ内on/グループ外off/対象外は不変",
              {'truck_co_0', 'truck_opp_0'} <= blockers
              and not ({'truck_co_1', 'truck_opp_1'} & blockers)
              and 'truck_co_2' in blockers, str(blockers))
        lc = cfg['link_controller_node']['ros__parameters']
        check("kkf_fullのconfig上書き反映",
              lc['scheduling_policy'] == 'external_schedule'
              and lc['control_plane'] == 'kkf_mpc')
    finally:
        os.unlink(path)


def test_manifest():
    print("[2] manifest の生成・finalize")
    with tempfile.TemporaryDirectory() as d:
        sweep_path = os.path.join(d, 'sweep.yaml')
        with open(sweep_path, 'w') as f:
            f.write(SWEEP_YAML)
        sweep_config.load_sweep_config(sweep_path)
        eval_dir = os.path.join(d, 'my_eval')
        m = eval_manifest.init_manifest(
            eval_dir, name='my_eval', sweep_config_path=sweep_path,
            scenario_path='config/scenarios/road_multicar.yaml',
            base_config_path='src/comms_sim_pkg/config/sim_params.yaml',
            num_runs=3, base_seed=777, seed_stride=1000,
            derive_run_seeds=True,
            generic_variables=sweep_config.GENERIC_VARIABLES)

        check("manifest.yaml存在",
              os.path.exists(os.path.join(eval_dir, 'manifest.yaml')))
        check("config/スナップショット3点",
              all(os.path.exists(os.path.join(eval_dir, 'config', n))
                  for n in ['sweep.yaml', 'scenario.yaml', 'sim_params_base.yaml']))
        check("raw/作成", os.path.isdir(os.path.join(eval_dir, 'raw')))
        check("シード表3run",
              len(m['seed_table']) == 3 and m['seed_table'][1]['channel'] == 1777
              and m['seed_table'][1]['measurement_report'] == 1787
              and m['seed_table'][1]['kkf'] == 1797)
        check("git commit記録", bool(m['git'].get('commit')))
        check("dirty時はworkspace.patch",
              (not m['git']['dirty'])
              or os.path.exists(os.path.join(eval_dir, 'config', 'workspace.patch')))
        check("変数記述にラベル",
              m['variables'][0]['labels'] == ['lut', 'kkf_full', 'baseline_empty'])

        # finalize: 進捗ログのDONE行からrun成否を拾う (リトライは最終行が勝つ)
        plog = os.path.join(d, 'progress.log')
        with open(plog, 'w') as f:
            f.write("START xxx TOTAL=2\n"
                    "DONE task=1 params=[method=lut] status=FAIL ts=t worker=0\n"
                    "DONE task=1 params=[method=lut] status=OK ts=t worker=0\n"
                    "DONE task=2 params=[method=kkf_full] status=OK ts=t worker=1\n"
                    "DONE task=2 params=[-] status=SWEEP_COMPLETE ts=t\n")
        m2 = eval_manifest.finalize_manifest(eval_dir, progress_log_path=plog)
        check("run成否2件・全OK",
              m2['task_summary'] == {'total': 2, 'failed': 0}
              and m2['runs'][0]['attempts'] == 2)
        check("status=completed", m2['status'] == 'completed')
        check("進捗ログ取込",
              os.path.exists(os.path.join(eval_dir, 'sweep_progress.log')))


if __name__ == '__main__':
    os.chdir(os.path.join(os.path.dirname(__file__), '..', '..'))
    test_cases_and_groups()
    test_manifest()
    print(f"\n{PASS} passed, {FAIL} failed")
    sys.exit(1 if FAIL else 0)
