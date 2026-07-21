#!/usr/bin/env python3
"""
run毎シード導出 (CRN) の単体テスト: inject_run_seeds の注入規則を検証。
コンテナ内で実行:
  docker compose exec -T sim bash -c \
    "cd /workspace && PYTHONDONTWRITEBYTECODE=1 python3 tools/tests/run_seed_test.py"
"""
import copy
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from lib import sweep_config


def base_config():
    return {
        'comms_simulator_node': {'ros__parameters': {
            'channel': {'seed': 42, 'shadowing': {'enabled': True}},
            'measurement_report': {'enabled': True, 'seed': 123},
        }},
        'link_controller_node': {'ros__parameters': {
            'scheduling_policy': 'external_schedule',
        }},
    }


def main():
    failures = []
    sweep_config.DERIVE_RUN_SEEDS = True
    sweep_config.BASE_SEED = 12345

    # 1) run_idx のみに依存: 同一 run なら別方式 (別config) でも同一シード (CRN)
    cfg_a, cfg_b = base_config(), base_config()
    cfg_b['link_controller_node']['ros__parameters']['scheduling_policy'] = 'feedforward_optimal'
    s_a = sweep_config.inject_run_seeds(cfg_a, 1)
    s_b = sweep_config.inject_run_seeds(cfg_b, 1)
    if s_a != s_b:
        failures.append(f"同一runで方式間シード不一致: {s_a} != {s_b}")

    # 2) run 間はシードが異なり、yaml の固定シードは上書きされる
    cfg1, cfg2 = base_config(), base_config()
    s1 = sweep_config.inject_run_seeds(cfg1, 1)
    s2 = sweep_config.inject_run_seeds(cfg2, 2)
    ch1 = cfg1['comms_simulator_node']['ros__parameters']['channel']['seed']
    ch2 = cfg2['comms_simulator_node']['ros__parameters']['channel']['seed']
    rep1 = cfg1['comms_simulator_node']['ros__parameters']['measurement_report']['seed']
    kkf1 = cfg1['link_controller_node']['ros__parameters']['kkf_seed']
    print(f"  run1: channel={ch1} report={rep1} kkf={kkf1} / run2: channel={ch2}")
    if s1 == s2 or ch1 == ch2:
        failures.append("run間でシードが変わらない")
    if ch1 == 42 or rep1 == 123:
        failures.append("yamlの固定シードが上書きされていない")
    if len({ch1, ch1 + 1, ch1 + 2, rep1, kkf1}) != 5:
        failures.append("派生シード (+1,+2,+10,+20) が衝突している")
    if abs(s2 - s1) < 30:
        failures.append(f"runシード間隔 {abs(s2-s1)} が派生オフセットと衝突しうる")

    # 3) channel セクションが無い config には channel.seed を新設しない
    cfg = base_config()
    del cfg['comms_simulator_node']['ros__parameters']['channel']
    sweep_config.inject_run_seeds(cfg, 1)
    if 'channel' in cfg['comms_simulator_node']['ros__parameters']:
        failures.append("channel セクションが無いのに新設された (物理モデル構成が変わる)")

    # 4) derive_run_seeds: false で無効化 (yaml のシード維持)
    sweep_config.DERIVE_RUN_SEEDS = False
    cfg = base_config()
    ret = sweep_config.inject_run_seeds(cfg, 2)
    if ret is not None or cfg['comms_simulator_node']['ros__parameters']['channel']['seed'] != 42:
        failures.append("derive_run_seeds=False でも注入された")
    sweep_config.DERIVE_RUN_SEEDS = True

    if failures:
        print("FAIL:")
        for f in failures:
            print(f"  - {f}")
        sys.exit(1)
    print("PASS: run_seed_test 全項目合格")


if __name__ == '__main__':
    main()
