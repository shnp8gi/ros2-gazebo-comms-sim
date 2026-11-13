#!/usr/bin/env python3
"""物理ベースの飛行を1回実行し、機体とアンテナの位置・姿勢を物理の刻みで記録する。

Docker 内または ROS 環境内で、ビルド後に実行する。出力ディレクトリに
world.sdf、flight.yaml（使った条件）、trace.bin / trace.json（記録）、gz.log を保存する。
条件は --set で上書きできる（例: --set scenario.speed_mps=3 vibration.enabled=true）。
"""
import argparse
from datetime import datetime
import os
from pathlib import Path
import subprocess
import time
import yaml

from flight_world import world

ROOT = Path(__file__).resolve().parents[1]
DEFAULT = ROOT / 'src/comms_sim_pkg/config/flight_default.yaml'
PLUGINS = ROOT / 'install/drone/comms_sim_pkg/lib/comms_sim_pkg/plugins'


def override(config, assignment):
    key, value = assignment.split('=', 1)
    node = config
    *parents, leaf = key.split('.')
    for part in parents:
        node = node[part]
    if leaf not in node:
        raise KeyError(f'設定にない項目です: {key}')
    node[leaf] = yaml.safe_load(value)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=str(DEFAULT))
    parser.add_argument('--set', nargs='*', default=[], metavar='KEY=VALUE')
    parser.add_argument('--output', help='新規の出力ディレクトリ')
    parser.add_argument('--gui', action='store_true', help='画面を表示して実時間で実行する')
    parser.add_argument('--plugins', default=str(PLUGINS))
    args = parser.parse_args()

    config = yaml.safe_load(Path(args.config).read_text())
    for assignment in args.set:
        override(config, assignment)
    output = Path(args.output or ROOT / f'sim_results/flight_{datetime.now():%Y%m%d_%H%M%S}').resolve()
    output.mkdir(parents=True, exist_ok=False)
    (output / 'flight.yaml').write_text(yaml.safe_dump(config, allow_unicode=True, sort_keys=False))
    (output / 'world.sdf').write_text(world(config, output, args.gui))
    env = dict(os.environ)
    env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = args.plugins + os.pathsep + env.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    env.setdefault('GZ_PARTITION', f'flight_{os.getpid()}')
    command = ['gz', 'sim', '-r', str(output / 'world.sdf')] + ([] if args.gui else ['-s'])
    print(f'飛行を実行します → {output}', flush=True)
    start = time.monotonic()
    with (output / 'gz.log').open('w') as log:
        subprocess.run(command, env=env, stdout=log, stderr=subprocess.STDOUT, check=True,
                       timeout=None if args.gui else 3600)
    if not (output / 'trace.json').exists():
        raise RuntimeError(f'記録が完了していません。{output / "gz.log"} を確認してください')
    print(f'完了: {time.monotonic() - start:.1f} s, {(output / "trace.json").read_text().strip()}', flush=True)


if __name__ == '__main__':
    main()
