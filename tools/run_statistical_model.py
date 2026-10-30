#!/usr/bin/env python3
"""②モデルを保存済みDocker環境で実行する。CSVとYAMLの変更後に再ビルドは不要。"""
import argparse
import os
from pathlib import Path
import shlex
import subprocess
import sys
import time
import uuid
import yaml


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=root/'src/comms_sim_pkg/config/drone_statistical_parallel_1mps.yaml')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--seed', type=int, help='受信成否の乱数seedを上書き')
    parser.add_argument('--replay', action='store_true')
    args = parser.parse_args()
    config = args.config.resolve()
    if not config.is_relative_to(root):
        parser.error('設定と参照CSVはこのリポジトリ内に置いてください')
    output = (args.output or root/'sim_results'/f'statistical_{time.strftime("%Y%m%d_%H%M%S")}_{uuid.uuid4().hex[:6]}').resolve()
    if not output.is_relative_to(root):
        parser.error('出力先はこのリポジトリ内に指定してください')
    if output.exists() and any(output.iterdir()):
        parser.error('出力先には既存結果があります')
    c = yaml.safe_load(config.read_text())
    if not c.get('empirical_phy', {}).get('enabled'):
        parser.error('empirical_phy.enabled=trueの設定を指定してください')
    setup = root/'install/no_expected_rssi/setup.bash'
    if not setup.exists():
        parser.error('ビルドが必要です。specification_review/statistical_model_usage_ja.txtを参照してください')
    # 元YAMLを変更しない。seed上書き用設定だけ一時ファイルへ保存する。
    temporary = None
    if args.seed is not None:
        if not 0 <= args.seed < 2**64:
            parser.error('seedは0〜2^64-1で指定してください')
        c['empirical_phy']['seed'] = args.seed
        temporary = config.parent/f'.statistical_run_{uuid.uuid4().hex}.yaml'
        temporary.write_text(yaml.safe_dump(c, allow_unicode=True, sort_keys=False))
        config = temporary
    container = lambda p: '/workspace/' + str(p.relative_to(root))
    command = 'source /opt/ros/humble/setup.bash && source install/no_expected_rssi/setup.bash && ros2 launch comms_sim_pkg sim_launch.py headless:=true bridge:=false ' + shlex.join([
        'config_file:='+container(config), 'output_dir:='+container(output)])
    start = time.monotonic()
    try:
        subprocess.run(['docker', 'run', '--rm', '--user', f'{os.getuid()}:{os.getgid()}', '-e', 'HOME=/tmp',
                        '-e', 'DISPLAY=', '-e', 'GZ_PARTITION=statistical-'+uuid.uuid4().hex,
                        '-v', f'{root}:/workspace', '-w', '/workspace',
                        'ros2-gazebo-comms-sim:humble-harmonic', 'bash', '-c', command], check=True)
    finally:
        if temporary:
            temporary.unlink()
    if not (output/'completed').exists():
        raise RuntimeError('シミュレーションの完了記録がありません')
    (output/'runtime_seconds.txt').write_text(f'{time.monotonic()-start:.3f}\n')
    subprocess.run([sys.executable, str(root/'tools/validate_drone_results.py'), str(output)], check=True)
    subprocess.run([sys.executable, str(root/'tools/analyze_statistical_results.py'), str(output)], check=True)
    replay_command = shlex.join([sys.executable, str(root/'tools/monitor_hrcp_signals.py'),
        str(output/'live_signals.json'), '--replay', '--autoplay', '--window', 'full', '--details', '--playback-speed', '2'])
    (output/'replay.sh').write_text('#!/usr/bin/env bash\nset -e\nexec '+replay_command+'\n')
    (output/'replay.sh').chmod(0o755)
    if args.replay:
        with (output/'viewer.log').open('a') as log:
            subprocess.Popen(['bash', str(output/'replay.sh')], stdout=log, stderr=log, start_new_session=True)
    print(f'結果: {output}')


if __name__ == '__main__':
    main()
