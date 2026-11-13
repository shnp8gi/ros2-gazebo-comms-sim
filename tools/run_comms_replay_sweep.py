#!/usr/bin/env python3
"""通信だけを計算して、最接近距離 × 取り付けオフセット × 構成を掃引する（Gazebo は使わない）。

Docker 内または ROS 環境内で、ビルド済みの comms_sweep を呼ぶ。結果は 1 条件 1 行の CSV。
範囲は「開始:終了:刻み」をカンマでつなげて指定できる（例: 1:10:0.5,10:40:2）。
"""
import argparse
from datetime import datetime
from pathlib import Path
import subprocess
import yaml

ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = ROOT / 'src/comms_sim_pkg/config'
DEFAULT_CONFIGS = [f'drone_simple_{name}.yaml' for name in ('c1', 'c2', 'c3_2', 'c3_4')]


def values(text):
    """'1:3:0.5,5,10:20:5' → [1, 1.5, 2, 2.5, 3, 5, 10, 15, 20]"""
    result = []
    for part in text.split(','):
        if ':' not in part:
            result.append(float(part))
            continue
        start, stop, step = (float(v) for v in part.split(':'))
        count = int(round((stop - start) / step))
        result += [round(start + i * step, 9) for i in range(count + 1)]
    return sorted(set(result))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--configs', nargs='+', default=DEFAULT_CONFIGS, help='config/ からの相対パスか絶対パス')
    parser.add_argument('--d-min', default='5', help='最接近距離 [m]')
    parser.add_argument('--offset-el', default='0', help='取り付けの仰角オフセット [deg]')
    parser.add_argument('--offset-az', default='0', help='取り付けの方位オフセット [deg]')
    parser.add_argument('--offset-target', choices=('both', 'a', 'b'), default='both', help='オフセットを与える機体')
    parser.add_argument('--speed', type=float, help='各機の速度 [m/s]。省略時は設定ファイルの値')
    parser.add_argument('--output', help='新規の出力ディレクトリ')
    parser.add_argument('--exe', default=str(ROOT / 'install/drone/comms_sim_pkg/lib/comms_sim_pkg/comms_sweep'))
    args = parser.parse_args()

    output = Path(args.output or ROOT / f'sim_results/comms_sweep_{datetime.now():%Y%m%d_%H%M%S}').resolve()
    output.mkdir(parents=True, exist_ok=False)
    runs = []
    for name in args.configs:
        config = Path(name) if Path(name).is_absolute() else CONFIG_DIR / name
        for d in values(args.d_min):
            for el in values(args.offset_el):
                for az in values(args.offset_az):
                    run = {'id': f'{config.stem}_d{d:g}_el{el:g}_az{az:g}', 'config': str(config), 'd_min_m': d}
                    if args.speed:
                        run['speed_mps'] = args.speed
                    for drone in ('a', 'b'):
                        if args.offset_target in ('both', drone):
                            run[f'offset_{drone}_el_deg'] = el
                            run[f'offset_{drone}_az_deg'] = az
                    runs.append(run)
    manifest = output / 'manifest.yaml'
    manifest.write_text(yaml.safe_dump({'command': vars(args), 'runs': runs}, allow_unicode=True, sort_keys=False))
    print(f'{len(runs)} 条件を計算します → {output}', flush=True)
    subprocess.run([args.exe, str(manifest), str(output / 'results.csv')], check=True)
    print(f'完了: {output / "results.csv"}', flush=True)


if __name__ == '__main__':
    main()
