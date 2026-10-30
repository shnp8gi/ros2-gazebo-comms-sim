"""開始位置比較の集計を検査し、日本語の結果説明と代表再生の入力を保存する。"""
import argparse
from collections import defaultdict
import csv
import json
from pathlib import Path
import statistics

from run_probability_campaign import PATTERNS
from run_start_position_sweep import category


NAMES = dict(zip(PATTERNS, ['並行・固定', '並行・追従', '十字・固定', '十字・追従']))
CLASSES = {'near_zero': 'ほぼ0回で接続', 'unconnected': '未接続',
           'nonzero_or_unreliable': 'Timeoutあり、または接続率が不足'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    root = args.directory.resolve()
    status = json.loads((root / 'progress.json').read_text())
    assert status['status'] == 'complete', '全試行の完了後に集計してください'
    params = status['parameters']
    with (root / 'analysis/trials.csv').open() as f:
        trials = list(csv.DictReader(f))
    with (root / 'analysis/position_summary.csv').open() as f:
        summaries = list(csv.DictReader(f))
    with (root / 'analysis/boundary_intervals.csv').open() as f:
        boundaries = list(csv.DictReader(f))
    assert len(trials) == status['trials']
    assert len({r['id'] for r in trials}) == len(trials)
    groups = defaultdict(list)
    for r in trials:
        position = float(r['start_path_m'] if params['axis'] == 'path' else r['start_distance_m'])
        groups[r['model'], r['scenario'], r['antenna_mode'], round(position, 6), r['phase']].append(r)
        assert float(r['speed_mps']) == 5
        assert abs(float(r['observation_duration_s']) - (float(r['start_path_m']) + float(r['end_path_m'])) / 5) < 1e-9
    for r in summaries:
        rs = groups[r['model'], r['scenario'], r['antenna_mode'], round(float(r['position_m']), 6), r['phase']]
        assert len(rs) == int(r['trials'])
        assert abs(statistics.mean(int(t['timeouts']) for t in rs) - float(r['mean_timeouts'])) < 1e-12
        assert abs(sum(t['connected'] == '1' for t in rs) / len(rs) - float(r['connected_fraction'])) < 1e-12
        for key in ['position_m', 'mean_timeouts', 'connected_fraction', 'zero_timeout_connection_fraction',
                    'first_half_mean', 'last_half_mean']:
            r[key] = float(r[key])
    text = [
        '各機5 m/s：開始位置を変えた初回リンク確立までのTimeout比較', '',
        '開始距離は各機の航路上で最接近地点までの距離。機体間直線距離ではない。',
        f'基本1 m刻み（最小点0.1 m）、区分の変わる区間を0.1 m刻みで追加。最大{params["max_position_m"]:g} m。',
        f'終了位置は最接近を通過して{params["end_position_m"]:g} m先に固定。開始位置を変えても終了位置は変えない。',
        '①は−68.5 dBmしきい値。②は既存の実測PER曲線と仮定を使い、独立seedで判定する。',
        f'①は各条件1回。②は粗探索{params["coarse_repetitions"]}回、境界と確認点{params["fine_repetitions"]}回。',
        '細分化した位置では追加試行の結果を使用し、粗探索と混ぜない。',
        'Timeoutは初回CONNECTEDより前のA/B両端末イベントを合算。未接続は観測終了まで数える。',
        '未接続のTimeout 0回を、0回で接続できた結果に含めない。',
        'ほぼ0回の区分：平均Timeoutが0.01回以下、かつ0回での接続率が99%以上。',
        '有限回の確認なので、②で今後も必ず0回になるという保証ではない。', '', '結果']
    for model, name in [('threshold', '①'), ('statistical', '②')]:
        for pattern in PATTERNS:
            rs = [r for r in summaries if (r['model'], r['scenario'], r['antenna_mode']) == (model, *pattern)]
            bs = [b for b in boundaries if (b['model'], b['scenario'], b['antenna_mode']) == (model, *pattern)]
            text.append(f'{name} {NAMES[pattern]}')
            if bs:
                for b in bs:
                    text.append(f"  {float(b['lower_m']):.1f}〜{float(b['upper_m']):.1f} m：{CLASSES[b['lower_class']]} → {CLASSES[b['upper_class']]}")
            elif all(category(r) == 'near_zero' for r in rs):
                text.append('  確認した全位置でほぼ0回で接続。今回の範囲に境界なし。')
            elif all(category(r) == 'unconnected' for r in rs):
                text.append('  確認した全位置で未接続。0回接続の境界は定義しない。')
            else:
                text.append('  確認した範囲に区分の変化なし。詳細はposition_summary.csv。')
            fine = [r for r in rs if r['phase'] == 'fine']
            if fine:
                delta = max(abs(r['first_half_mean'] - r['last_half_mean']) for r in fine)
                text.append(f'  確認試行の前半・後半での平均Timeout差（最大）：{delta:.6g}回。')
    text += ['', '解釈と保存先',
        '①②はMAC・再送処理も異なるため、差を受信モデルだけに帰属させない。',
        '追従のQしきい値5秒前開始とは別実験。今回は開始位置を独立に変えている。',
        '既存のRFパラメータ・500 msの接続待機時間・制御MCS 3は維持している。',
        'P側は未校正の代理曲線を含む。ここで得た開始距離は実機の通信距離保証ではない。',
        'analysis/position_timeout_overview.png：全範囲の比較',
        'analysis/position_timeout_detail.png：境界周辺の比較',
        'analysis/position_summary.csv：各位置の平均、接続率、0回接続率、試行数',
        'analysis/trials.csv：全試行の生結果',
        'analysis/boundary_intervals.csv：0.1 m刻みの区分境界',
        'monitor.sh：代表試行の通常Gazebo再生（再生ログ作成完了後に使用可能）']
    (root / 'RESULTS_ja.txt').write_text('\n'.join(text) + '\n')
    (root / 'README_ja.txt').write_text(
        '各機5 m/s・開始位置の比較\n\n結果と条件はRESULTS_ja.txt。\n'
        'analysis/にグラフとCSV、batches/に試行入力・出力、metadata/に共通設定を保存。\n'
        '全試行は通常実行と同じRadioModel/LinkProtocolで初回接続まで計算。\n'
        'replays/は選択した代表seedの通常Gazeboによる全区間ログ。平均値の再生ではない。\n'
        '\n再現コマンド（リポジトリ直下で実行）\n'
        'python3 tools/run_start_position_sweep.py 保存先 --axis path --max-position 60 --jobs 4\n'
        '保存先は同じ実験親フォルダ内の新規フォルダ。途中再開は同じ条件に --resume を追加。\n'
        '再開時は保存済み試行ID・開始位置・seedを照合し、未計算の試行のみ実行する。\n'
        'python3 tools/summarize_start_position_sweep.py 保存先\n'
        'python3 tools/prepare_timeout_boundary_replays.py 保存先 --jobs 4\n'
        'bash 保存先/monitor.sh\n'
        '\n計算器は初回CONNECTEDイベントを記録した瞬間で終了し、接続後のDATA計算を省く。\n'
        '既存の通信本体・受信判定・再送間隔・乱数方式は変更していない。\n')
    targets = []
    # 境界の両側と、境界が存在しないパターンの代表を選ぶ。
    selected = []
    for pattern in PATTERNS:
        rs = [r for r in summaries if (r['model'], r['scenario'], r['antenna_mode']) == ('statistical', *pattern)]
        bs = [b for b in boundaries if (b['model'], b['scenario'], b['antenna_mode']) == ('statistical', *pattern)]
        positions = {float(b[k]) for b in bs for k in ['lower_m', 'upper_m']}
        selected.extend([r for r in rs if r['position_m'] in positions] if positions else [rs[len(rs) // 2]])
    fixed = [r for r in summaries if (r['model'], r['scenario'], r['antenna_mode']) == ('threshold', 'parallel', 'fixed')]
    for cls in ['near_zero', 'unconnected']:
        candidates = [r for r in fixed if category(r) == cls]
        if candidates:
            selected.append(candidates[-1] if cls == 'near_zero' else candidates[0])
    for r in selected:
        rs = groups[r['model'], r['scenario'], r['antenna_mode'], round(r['position_m'], 6), r['phase']]
        trial = min(rs, key=lambda t: (abs(int(t['timeouts']) - r['mean_timeouts']), int(t['seed'])))
        name = '①' if r['model'] == 'threshold' else '②'
        targets.append(dict(model=r['model'], scenario=r['scenario'], antenna_mode=r['antenna_mode'], speed_mps=5,
            seed=int(trial['seed']), timeouts=int(trial['timeouts']), first_connected_s=float(trial['first_connected_s']),
            source_trial=trial['id'], start_path_m=float(trial['start_path_m']), end_path_m=float(trial['end_path_m']),
            label=f"{name} {NAMES[r['scenario'], r['antenna_mode']]} 開始距離{float(trial['start_path_m']):.1f} m"))
    target_path = root / 'metadata/replay_targets.json'
    if target_path.exists():
        assert json.loads(target_path.read_text()) == targets, '既存の再生対象と異なるため上書きしません'
    else:
        target_path.write_text(json.dumps(targets, ensure_ascii=False, indent=2))
    verification = dict(trials_checked=len(trials), summary_conditions_checked=len(summaries),
                        speed_mps=5, axis=params['axis'], replay_targets=len(targets), summary_matches_trials=True)
    (root / 'analysis/verification.json').write_text(json.dumps(verification, ensure_ascii=False, indent=2))
    print(json.dumps(verification, ensure_ascii=False))


if __name__ == '__main__':
    main()
