#!/usr/bin/env python3
"""
同一条件で別々に回した評価を束ねて、走行数を増やした集計をやり直す。

sweep_sim の resume はタスクを (y, アンテナ角) だけで識別するため、
method × load のような一般変数の掃引では走行数を増やす追加実行ができない。
そこで別スイープとして回し、ここで runs.csv 同士を突き合わせる。

走行番号は入力順に連番へ振り直す (先頭が run 1..n、次が run n+1..)。
各スイープの base_seed が重なっていないことが前提で、重なっていれば
同じ交通実現を二重に数えることになるため停止する。

  python3 tools/merge_evals.py sim_results/eval_a sim_results/eval_b \\
      --out sim_results/eval_merged --baseline assoc_hold

集計と対比較は tools/lib/eval_metrics.py の実装をそのまま使うので、
単一スイープを解析した場合と同じ計算になる。
"""
import argparse
import os
import sys

import pandas as pd
import yaml

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TOOLS_DIR)

from lib import eval_metrics  # noqa: E402


def base_seed_of(eval_dir):
    """そのスイープが使った base_seed (manifest から)。取れなければ None。"""
    for name in ('manifest.yaml', 'config/sweep.yaml'):
        p = os.path.join(eval_dir, name)
        if not os.path.exists(p):
            continue
        try:
            with open(p, 'r', encoding='utf-8') as f:
                d = yaml.safe_load(f) or {}
        except Exception:
            continue
        for path in (('sweep', 'execution', 'base_seed'),
                     ('execution', 'base_seed'), ('base_seed',)):
            cur = d
            for k in path:
                cur = cur.get(k) if isinstance(cur, dict) else None
                if cur is None:
                    break
            if isinstance(cur, int):
                return cur
    return None


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('eval_dirs', nargs='+', help='統合する評価ディレクトリ (2個以上)')
    ap.add_argument('--out', required=True, help='出力先ディレクトリ')
    ap.add_argument('--baseline', default=None, help='対比較の基準 arm')
    ap.add_argument('--arm-var', default='method')
    ap.add_argument('--group-vars', nargs='*', default=['method', 'load'])
    ap.add_argument('--seed-stride', type=int, default=1000,
                    help='走行ごとのシード刻み (重なり判定に使う)')
    a = ap.parse_args()

    if len(a.eval_dirs) < 2:
        sys.exit("統合には評価ディレクトリが2つ以上要ります")

    frames, seeds_seen, offset = [], {}, 0
    for d in a.eval_dirs:
        p = os.path.join(d, 'analysis', 'runs.csv')
        if not os.path.exists(p):
            sys.exit(f"runs.csv がありません: {p} (先に sim.py analyze を実行)")
        df = pd.read_csv(p)
        n_runs = int(df['run'].max())

        # シードの重なりを検出する。重なったまま束ねると、同じ交通実現を
        # 独立標本として二重に数え、対比較の自由度を偽って有意に見せる
        bs = base_seed_of(d)
        if bs is not None:
            for i in range(1, n_runs + 1):
                s = bs + a.seed_stride * (i - 1)
                if s in seeds_seen:
                    sys.exit(f"シード {s} が {seeds_seen[s]} と {d} で重複。"
                             "同じ交通実現を二重に数えるため統合できません")
                seeds_seen[s] = d
        else:
            print(f"警告: {d} の base_seed を読めず、重複を検査できません")

        df = df.copy()
        df['run'] = df['run'] + offset
        df['source'] = os.path.basename(d.rstrip('/'))
        frames.append(df)
        offset += n_runs
        print(f"  {d}: {n_runs} 走行 -> run {offset - n_runs + 1}..{offset}")

    runs = pd.concat(frames, ignore_index=True)
    group_vars = [v for v in a.group_vars if v in runs.columns]
    n_per = runs.groupby(group_vars)['run'].nunique()
    if n_per.nunique() != 1:
        print(f"警告: 条件ごとの走行数が揃っていません\n{n_per.to_string()}")
    print(f"統合後: 条件あたり {int(n_per.iloc[0])} 走行")

    os.makedirs(a.out, exist_ok=True)
    out_dir = os.path.join(a.out, 'analysis')
    os.makedirs(out_dir, exist_ok=True)
    runs.to_csv(os.path.join(out_dir, 'runs.csv'), index=False)

    agg = eval_metrics.aggregate(runs, group_vars)
    agg.to_csv(os.path.join(out_dir, 'agg.csv'), index=False)
    if a.baseline:
        rest = [v for v in group_vars if v != a.arm_var]
        pr = eval_metrics.paired(runs, a.baseline, a.arm_var, rest)
        pr.to_csv(os.path.join(out_dir, 'paired.csv'), index=False)

    piv = agg.pivot(index=group_vars[0], columns=group_vars[1],
                    values='total_data_MB_mean') if len(group_vars) >= 2 else None
    if piv is not None:
        print(f"\n=== total_data_MB (mean) ===\n{piv.round(1).to_string()}")
    print(f"\n[merge] wrote {out_dir}/runs.csv, agg.csv"
          + (", paired.csv" if a.baseline else ""))
    return 0


if __name__ == '__main__':
    sys.exit(main())
