"""
manifest.py
-----------
評価ディレクトリの manifest.yaml の生成・更新 (L0: stdlib + yaml のみ)。

設計原則「評価ディレクトリが唯一の契約」の中核。ツール間の受け渡しは
すべて manifest.yaml と評価ディレクトリの明文化されたレイアウトを介する:

  <eval_dir>/
    manifest.yaml      本ファイルが生成。メタデータ・シード表・run成否・スキーマ
    config/            再現に必要な入力の完全スナップショット
      sweep.yaml, scenario.yaml, sim_params_base.yaml, workspace.patch
    raw/               シミュレータ出力 (detailed_logs/ events/ controls/
                       summaries/ rssi_profiles/)。data.parquet 統合後は削除可
    data.parquet       tidy 長形式 (L1: eval_data.py が生成)
    analysis/          集計 CSV (L1/L2)
    figures/           図 (L2)
    report.html        自己完結レポート (L3)

再現性の保証水準 (manifest にも記載):
  - ビット一致は保証しない (タイミングジッタ・RTFスケジューリングのため)
  - 保証するのは「同一シード → 統計的に同一」(CRN対応が成立する水準)
"""
import os
import shutil
import socket
import subprocess
import sys
import datetime

import yaml

SCHEMA_VERSION = 1
MANIFEST_NAME = "manifest.yaml"

REPRODUCIBILITY_NOTE = (
    "ビット一致再現は保証しない (プロセスタイミング・RTFスケジューリング非決定性)。"
    "保証水準: 同一シード表からの再実行は統計的に同一分布 (CRN対応比較が成立)。"
)


def _run(cmd, cwd=None, timeout=30):
    """コマンドを実行し stdout を返す。失敗時は None (manifest 生成は止めない)。"""
    try:
        out = subprocess.run(cmd, cwd=cwd, timeout=timeout,
                             stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        if out.returncode == 0:
            return out.stdout.decode('utf-8', errors='replace').strip()
    except Exception:
        pass
    return None


def collect_environment():
    """再現に有用な実行環境情報を収集する (取得失敗は None のまま記録)。"""
    env = {
        'hostname': socket.gethostname(),
        'in_container': os.path.exists('/.dockerenv'),
        'python': sys.version.split()[0],
        'ros_distro': os.environ.get('ROS_DISTRO'),
    }
    gz = _run(['ign', 'gazebo', '--version']) or _run(['gz', 'sim', '--version'])
    env['gazebo'] = gz.splitlines()[0] if gz else None
    return env


def git_snapshot(repo_root, patch_out_path=None):
    """git のコミット・ブランチ・dirty 状態を取得し、未コミット差分をパッチ保存する。

    パッチ (workspace.patch) と commit ハッシュがあれば、当時のコードは
    `git checkout <commit> && git apply workspace.patch` で完全復元できる。
    """
    # コンテナ内では /workspace がホストユーザー所有のため、safe.directory を
    # 明示しないと git が dubious ownership で拒否する
    git = ['git', '-c', f'safe.directory={repo_root}']
    info = {
        'commit': _run(git + ['rev-parse', 'HEAD'], cwd=repo_root),
        'branch': _run(git + ['rev-parse', '--abbrev-ref', 'HEAD'], cwd=repo_root),
    }
    status = _run(git + ['status', '--porcelain'], cwd=repo_root)
    dirty_files = [l for l in (status or '').splitlines() if l.strip()]
    info['dirty'] = bool(dirty_files)
    info['dirty_files'] = dirty_files
    if patch_out_path and info['dirty']:
        # 追跡ファイルの差分 + 未追跡ファイルは一覧のみ (バイナリ肥大を避ける)
        patch = _run(git + ['diff', 'HEAD'], cwd=repo_root, timeout=60)
        if patch:
            with open(patch_out_path, 'w', encoding='utf-8') as f:
                f.write(patch + '\n')
            info['patch_file'] = os.path.basename(patch_out_path)
    return info


def build_seed_table(num_runs, base_seed, stride, derive=True):
    """run毎の解決済みシード表 (sweep_config.inject_run_seeds と同一の導出式)。"""
    if not derive:
        return []
    rows = []
    for run_idx in range(1, int(num_runs) + 1):
        seed0 = int(base_seed) + int(stride) * (run_idx - 1)
        rows.append({'run': run_idx, 'channel': seed0,
                     'measurement_report': seed0 + 10, 'kkf': seed0 + 20})
    return rows


def describe_variables(generic_variables):
    """sweep変数の名前とラベル一覧 (条件マトリクスの記述)。"""
    out = []
    for var in generic_variables or []:
        labels = []
        for state in var.get('states', []):
            if not state:
                continue
            lab = state[0].get('state_label')
            if lab is None:
                lab = state[0].get('raw_value')
            labels.append(lab)
        out.append({'name': var.get('name'), 'labels': labels})
    return out


def init_manifest(eval_dir, *, name, sweep_config_path, scenario_path,
                  base_config_path, num_runs, base_seed, seed_stride,
                  derive_run_seeds, generic_variables, command_line=None,
                  extra=None):
    """スイープ開始時に評価ディレクトリを初期化し manifest.yaml を書く。

    config/ に sweep・シナリオ・ベース設定のスナップショットと
    workspace.patch (未コミット差分) を保存する。
    """
    cfg_dir = os.path.join(eval_dir, 'config')
    os.makedirs(cfg_dir, exist_ok=True)
    os.makedirs(os.path.join(eval_dir, 'raw'), exist_ok=True)

    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
    snapshots = {}
    for src, dst in [(sweep_config_path, 'sweep.yaml'),
                     (scenario_path, 'scenario.yaml'),
                     (base_config_path, 'sim_params_base.yaml')]:
        if src and os.path.exists(src):
            shutil.copy2(src, os.path.join(cfg_dir, dst))
            snapshots[dst] = os.path.relpath(src, repo_root) \
                if os.path.isabs(src) else src

    manifest = {
        'schema_version': SCHEMA_VERSION,
        'name': name,
        'status': 'running',
        'created_at': datetime.datetime.now().astimezone().isoformat(),
        'command_line': command_line if command_line is not None else ' '.join(sys.argv),
        'reproducibility': REPRODUCIBILITY_NOTE,
        'reproduce_command': f"python3 tools/sim.py reproduce sim_results/{name}",
        'environment': collect_environment(),
        'git': git_snapshot(repo_root, os.path.join(cfg_dir, 'workspace.patch')),
        'config_snapshots': snapshots,
        'execution': {
            'num_runs': int(num_runs),
            'base_seed': int(base_seed),
            'seed_stride': int(seed_stride),
            'derive_run_seeds': bool(derive_run_seeds),
        },
        'seed_table': build_seed_table(num_runs, base_seed, seed_stride,
                                       derive_run_seeds),
        'variables': describe_variables(generic_variables),
        'runs': [],           # finalize 時に progress ログから充填
        'layout': {
            'raw': 'raw/ (detailed_logs, events, controls, summaries, rssi_profiles)',
            'data': 'data.parquet (tidy長形式。生成後 raw/ は削除可)',
            'analysis': 'analysis/ (runs.csv, agg.csv, paired.csv)',
            'figures': 'figures/',
            'report': 'report.html',
        },
    }
    if extra:
        manifest.update(extra)
    write_manifest(eval_dir, manifest)
    return manifest


def parse_progress_log(progress_log_path):
    """sweep_sim の進捗ログから task 単位の成否を抽出する。

    DONE task=<n> params=[...] status=<OK|FAIL|...> の行を最終状態として採用
    (リトライで同一 task の行が複数あるときは最後の行が勝つ)。
    """
    import re
    results = {}
    if not progress_log_path or not os.path.exists(progress_log_path):
        return []
    with open(progress_log_path, 'r', encoding='utf-8') as f:
        for line in f:
            m = re.search(r"DONE task=(\d+) params=\[(.*?)\] status=(\w+)", line)
            if m and m.group(3) != 'SWEEP_COMPLETE':
                task_no = int(m.group(1))
                prev = results.get(task_no)
                attempts = (prev['attempts'] + 1) if prev else 1
                results[task_no] = {'task': task_no, 'params': m.group(2),
                                    'status': m.group(3), 'attempts': attempts}
    return [results[k] for k in sorted(results)]


def finalize_manifest(eval_dir, progress_log_path=None, status=None):
    """スイープ終了時に run 成否と終了時刻を manifest へ反映する。

    進捗ログ自体も評価ディレクトリへコピーして自己完結させる。
    """
    manifest = read_manifest(eval_dir)
    if manifest is None:
        return None
    runs = parse_progress_log(progress_log_path)
    manifest['runs'] = runs
    n_fail = sum(1 for r in runs if r['status'] != 'OK')
    if status is None:
        status = 'completed' if runs and n_fail == 0 else \
                 ('completed_with_failures' if runs else 'unknown')
    manifest['status'] = status
    manifest['finished_at'] = datetime.datetime.now().astimezone().isoformat()
    manifest['task_summary'] = {'total': len(runs), 'failed': n_fail}
    if progress_log_path and os.path.exists(progress_log_path):
        try:
            shutil.copy2(progress_log_path,
                         os.path.join(eval_dir, 'sweep_progress.log'))
        except OSError:
            pass
    write_manifest(eval_dir, manifest)
    return manifest


def read_manifest(eval_dir):
    path = os.path.join(eval_dir, MANIFEST_NAME)
    if not os.path.exists(path):
        return None
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def write_manifest(eval_dir, manifest):
    path = os.path.join(eval_dir, MANIFEST_NAME)
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        yaml.safe_dump(manifest, f, allow_unicode=True, sort_keys=False,
                       default_flow_style=False)
    os.replace(tmp, path)
