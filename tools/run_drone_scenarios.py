#!/usr/bin/env python3
"""Docker内またはROS環境内で4条件を順次実行する。"""
import argparse
import csv
from datetime import datetime
import json
from pathlib import Path
import subprocess
import time
import uuid
import yaml
from validate_drone_results import validate
from analyze_drone_results import analyze


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default='src/comms_sim_pkg/config/drone_params.yaml')
    parser.add_argument('--output', default=None)
    parser.add_argument('--speed-a-kmh', type=float)
    parser.add_argument('--speed-b-kmh', type=float)
    args = parser.parse_args()
    root = Path(args.output or f'sim_results/drone_four_{datetime.now():%Y%m%d_%H%M%S}_{uuid.uuid4().hex[:6]}').resolve()
    root.mkdir(parents=True, exist_ok=False)
    config = Path(args.config).resolve()
    duration = yaml.safe_load(config.read_text())['duration_s']
    manifest = {'config': str(config), 'runs': []}
    summaries = []
    for scenario in ('parallel','cross'):
        for mode in ('fixed','tracking'):
            name = f'{scenario}_{mode}'
            output = root/name
            cmd = ['ros2','launch','comms_sim_pkg','sim_launch.py','headless:=true',
                   f'config_file:={config}',f'scenario:={scenario}',f'antenna_mode:={mode}',f'output_dir:={output}']
            for suffix, value in (('a',args.speed_a_kmh),('b',args.speed_b_kmh)):
                if value is not None: cmd.append(f'speed_{suffix}_kmh:={value}')
            print(f'実行: {name} → {output}',flush=True)
            start = time.monotonic()
            with (root/f'{name}.log').open('w') as log:
                subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=max(120,duration*30))
            elapsed = time.monotonic()-start
            row = validate(output)
            row.update(analyze(output))
            row['wall_time_s'] = round(elapsed,3)
            summaries.append(row)
            manifest['runs'].append({'name':name,'command':cmd,'wall_time_s':elapsed,'validated':True})
            (root/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    with (root/'comparison.csv').open('w') as stream:
        writer=csv.DictWriter(stream, fieldnames=summaries[0].keys())
        writer.writeheader();writer.writerows(summaries)
    print(f'4条件すべて完了: {root}',flush=True)


if __name__ == '__main__': main()
