#!/usr/bin/env python3
"""RSSIモニターを開いてからGazebo headlessを起動する（sudo不要）。"""
import argparse
import json
import os
from datetime import datetime
from pathlib import Path
import shlex
import subprocess
import sys
import uuid
import yaml


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default='src/comms_sim_pkg/config/drone_hrcp_monitor.yaml')
    parser.add_argument('--scenario', choices=('parallel','cross'))
    parser.add_argument('--antenna-mode', choices=('fixed','tracking'))
    parser.add_argument('--no-viewer', action='store_true', help='描画負荷の比較用。通信とモニター記録は同じ条件で実行')
    args = parser.parse_args()
    config = (root/args.config).resolve()
    if not config.is_file():
        parser.error(f'設定ファイルがありません: {config}')
    try:
        relative = config.relative_to(root)
    except ValueError:
        parser.error('Dockerで参照できるリポジトリ内の設定を指定してください')
    output = Path('sim_results')/f'hrcp_monitor_{datetime.now():%Y%m%d_%H%M%S}_{uuid.uuid4().hex[:6]}'
    settings=yaml.safe_load(config.read_text())
    viewer = None if args.no_viewer else subprocess.Popen([sys.executable,str(root/'tools/monitor_hrcp_signals.py'),
                               str(root/output/'live_signals.json'),'--window',
                               'full' if settings['duration_s']>1 else 'setup'],cwd=root)
    launch = ['ros2','launch','comms_sim_pkg','sim_launch.py','headless:=true','bridge:=false',
              f'config_file:=/workspace/{relative}',f'output_dir:=/workspace/{output}']
    if args.scenario:
        launch.append(f'scenario:={args.scenario}')
    if args.antenna_mode:
        launch.append(f'antenna_mode:={args.antenna_mode}')
    shell = 'source /opt/ros/humble/setup.bash && source install/drone/setup.bash && '+shlex.join(launch)
    command = ['docker','compose','-f','docker-compose.yml','run','--rm','--no-deps',
               '--user',f'{os.getuid()}:{os.getgid()}',
               '-e','HOME=/tmp','-e','DISPLAY=','-e','OMP_WAIT_POLICY=PASSIVE','sim','bash','-c',shell]
    print(f'モニター実行開始。出力先: {root/output}',flush=True)
    try:
        subprocess.run(['sg','docker','-c',shlex.join(command)],cwd=root,check=True)
        if not (root/output/'completed').is_file():
            raise RuntimeError('Gazeboの完了マーカーがありません')
        from validate_drone_results import validate
        from analyze_drone_results import analyze
        result=validate(root/output)
        counts=analyze(root/output,plot=False)
        analysis_key='hrcp' if settings.get('hrcp',{}).get('enabled',False) else 'analysis'
        (root/output/'verification.json').write_text(json.dumps({'validated':True,'preview_mode':settings.get('preview_mode',False),'summary':result,analysis_key:counts},ensure_ascii=False,indent=2)+'\n')
        print('実行完了。' + ('モニターは最終結果を表示しています。閉じると終了します。' if viewer else '描画なしで結果を保存しました。'),flush=True)
        if viewer:
            viewer.wait()
    except (subprocess.CalledProcessError,RuntimeError,KeyboardInterrupt):
        if viewer:
            viewer.terminate()
        raise


if __name__ == '__main__':
    main()
