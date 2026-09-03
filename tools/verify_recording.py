#!/usr/bin/env python3
"""記録走行に、設定したエンティティが全部そろっているかを検査する。

なぜ要るか
----------
車両は ros_gz_sim の create (/world/<name>/create) でスポーンされるが、この
要求は既定 5 秒でタイムアウトする。並列実行で Gazebo が CPU を奪われると
これを超えることがあり、しかも create はタイムアウト後も終了コード 0 で
抜けるため、車両が欠けたまま「正常終了」する。静的エンティティ (基地局・
路上駐車) は同じ理由でワールドSDF直書きに移されたが (sim_launch.py の
コメント参照)、車両は create のままなので、この検査で拾う必要がある。

実測 (2026-09-01): rec_d70_n50 の 50 走行のうち 7 走行で欠落、最悪 21/49。
欠落は並列起動の第1バッチに集中する。

  python3 tools/verify_recording.py sim_results/rec_d70_n50
  python3 tools/verify_recording.py sim_results/*_pilot --max-missing-pct 0
"""
import argparse
import glob
import os
import sys

import numpy as np
import pandas as pd
import yaml


def motion_start(poses):
    """交通が走り出したシム時刻。全車が /sim/all_ready で同時に走り出す。"""
    d = poses.dropna(subset=['t_s', 'model', 'x'])
    v = d[d['model'].str.startswith(('tx', 'tf'), na=False)].sort_values('t_s')
    if v.empty:
        return None
    mv = (v.groupby('model')['x'].diff().abs() > 1e-4).fillna(False)
    return float(v.loc[mv, 't_s'].min()) if mv.any() else None


def expected_names(params):
    names = set()
    for v in params.get('vehicles', []) or []:
        if v.get('name'):
            names.add(v['name'])
    for key, cfg in (params.get('blocker_entities', {}) or {}).items():
        names.add((cfg or {}).get('name', key))
    for key, cfg in (params.get('spawn_entities', {}) or {}).items():
        names.add((cfg or {}).get('name', key))
    return names


def check_run(seed_dir, duration=None):
    """欠落と、評価窓が確保できているかを検査する。

    走り出し (/sim/all_ready) の発火はシム時刻ではばらつき、記録の終端も
    交通の詰まり具合で変わる。--duration を指定した評価では
    「走り出し + duration ≤ 記録終端」が成り立っている必要がある。
    """
    cfg = os.path.join(seed_dir, 'effective_sim_params.yaml')
    poses = os.path.join(seed_dir, 'poses.csv')
    if not (os.path.exists(cfg) and os.path.exists(poses)):
        return None
    params = yaml.safe_load(open(cfg, encoding='utf-8'))
    want = expected_names(params)
    po = pd.read_csv(poses, usecols=['t_s', 'model', 'x'])
    got = set(po.model.dropna().unique())
    win = None
    if duration is not None:
        t0 = motion_start(po)
        win = None if t0 is None else float(po.t_s.max()) - t0
    return want, want - got, win


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('rec_roots', nargs='+', help='記録ディレクトリ (seed_* を含む)')
    ap.add_argument('--max-missing-pct', type=float, default=0.0,
                    help='許容する欠落率 [%%]。これを超える走行があれば異常終了')
    ap.add_argument('--duration', type=float, default=None,
                    help='評価窓の長さ [s]。走り出しからこの長さを確保できない'
                         '走行を異常として報告する')
    ap.add_argument('--quiet', action='store_true', help='問題のある走行だけ出す')
    a = ap.parse_args()

    bad_total = 0
    for root in a.rec_roots:
        seeds = sorted(glob.glob(os.path.join(root, 'seed_*')))
        rows, skipped = [], 0
        for sd in seeds:
            r = check_run(sd, a.duration)
            if r is None:
                skipped += 1
                continue
            want, missing, win = r
            rows.append((os.path.basename(sd), len(want), sorted(missing), win))
        if not rows:
            print(f"{root}: 検査できる走行がありません")
            continue
        def bad_miss(r):
            return 100.0 * len(r[2]) / max(r[1], 1) > a.max_missing_pct

        def bad_win(r):
            return a.duration is not None and (r[3] is None or r[3] < a.duration)

        bad = [r for r in rows if bad_miss(r) or bad_win(r)]
        bad_total += len(bad)
        worst = max((100.0 * len(r[2]) / max(r[1], 1) for r in rows), default=0.0)
        mark = '✅' if not bad else '❌'
        msg = (f"{mark} {root}: {len(rows)} 走行 / 欠落のある走行 "
               f"{sum(1 for r in rows if r[2])} / 最悪の欠落率 {worst:.1f}%")
        if a.duration is not None:
            short = [r for r in rows if bad_win(r)]
            wins = [r[3] for r in rows if r[3] is not None]
            msg += (f" / 窓 {a.duration:.0f}s 未確保 {len(short)} 走行"
                    f"（最短 {min(wins):.1f}s）" if wins else '')
        print(msg + (f" / 検査不能 {skipped}" if skipped else ""))
        for name, n, missing, win in (bad if a.quiet else rows):
            notes = []
            if missing:
                notes.append(f"{len(missing)}/{n} 欠落 "
                             f"({100.0*len(missing)/n:.1f}%) 例 {missing[:4]}")
            if bad_win((name, n, missing, win)):
                notes.append(f"窓 {win:.1f}s < {a.duration:.0f}s"
                             if win is not None else '走り出し検出不可')
            if notes:
                print(f"    {name}: " + ' / '.join(notes))
    return 1 if bad_total else 0


if __name__ == '__main__':
    sys.exit(main())
