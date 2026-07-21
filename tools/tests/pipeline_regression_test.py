#!/usr/bin/env python3
"""
新パイプライン (eval_data + eval_metrics) と旧 analyze_multicar_eval.py の
数値一致を検証する回帰テスト。コンテナ内で実行:
  docker compose exec -T sim python3 /workspace/tools/tests/pipeline_regression_test.py

前提: sim_results/road_multicar_eval に
  - 旧スクリプトが生成した analysis_runs.csv (基準値)
  - 新パイプラインが生成した *.parquet
の両方が存在すること。
"""
import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from lib import eval_data, eval_metrics

EVAL_DIR = 'sim_results/road_multicar_eval'
GROUP_VARS = ['method', 'density']

# 旧→新の指標名対応 (総量系は同名)
COMPARE = ['total_data_MB', 'connected_time_s', 'min_car_data_MB',
           'nlos_grant_pct', 'ho_count']


def main():
    os.chdir(os.path.join(os.path.dirname(__file__), '..', '..'))
    old = pd.read_csv(os.path.join(EVAL_DIR, 'analysis_runs.csv'))
    tables = eval_data.load_tables(EVAL_DIR)
    new = eval_metrics.runs_table(tables, GROUP_VARS)
    # 旧runs表のdensityはint、新はparquet由来のstr → 揃える
    new['density'] = new['density'].astype(int)

    # analysis_runs.csv は論文版 = kkf_full を road_multicar_kkftune
    # (トラッカー調整版の別実行) で差し替えた集計。base dir のデータとは
    # kkf_full 行だけ一致しないため比較から除外する。
    # (完全一致の初回検証は 2026-07-17 に差し替え前の集計で確認済み: 300/300)
    old = old[old['method'] != 'kkf_full']
    new = new[new['method'] != 'kkf_full']

    keys = GROUP_VARS + ['run']
    merged = old.merge(new, on=keys, suffixes=('_old', '_new'))
    print(f"突き合わせ: 旧{len(old)}行 × 新{len(new)}行 → 共通{len(merged)}行")
    if len(merged) != len(old) or len(merged) != len(new):
        print("FAIL: 行数が一致しません")
        sys.exit(1)

    n_fail = 0
    for met in COMPARE:
        a = merged[f'{met}_old'].astype(float)
        b = merged[f'{met}_new'].astype(float)
        both_nan = a.isna() & b.isna()
        close = np.isclose(a, b, rtol=1e-6, atol=1e-6) | both_nan
        n_bad = int((~close).sum())
        status = 'ok  ' if n_bad == 0 else 'FAIL'
        print(f"  {status} {met}: {len(merged) - n_bad}/{len(merged)} 一致", end='')
        if n_bad:
            n_fail += 1
            worst = (a - b).abs().nlargest(3)
            print(f"  最大差 {worst.iloc[0]:.6g} (例: "
                  f"{merged.loc[worst.index[0], keys].to_dict()})")
        else:
            print()

    # jain_index は新指標なので値域チェックのみ
    ji = new['jain_index'].dropna()
    ok_ji = bool(((ji > 0) & (ji <= 1.0 + 1e-9)).all())
    print(f"  {'ok  ' if ok_ji else 'FAIL'} jain_index: 値域(0,1] "
          f"min={ji.min():.3f} max={ji.max():.3f}")
    if not ok_ji:
        n_fail += 1

    print('\nPASS: 新旧パイプライン一致' if n_fail == 0
          else f'\nFAIL: {n_fail}指標で不一致')
    sys.exit(1 if n_fail else 0)


if __name__ == '__main__':
    main()
