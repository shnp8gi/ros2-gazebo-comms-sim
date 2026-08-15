#!/usr/bin/env python3
"""
選択余地の掃引を一括評価し、「天井 vs 選択余地率」の表を出す。

各条件について
  1. bs 索引の規約が単一かを検査する (割れていたらその条件を捨てる)
  2. 選択余地率を実測する (2基以上が同時に接続可能な時刻の割合)
  3. 再生で assoc_hold と oracle を回し、天井 = oracle - assoc_hold を出す

天井は「予測を完璧にしたら反応型からどれだけ伸びるか」で、予測器を改良する
余地の上限にあたる (L3 の割当層は固定なので、割当を変えれば超えうる)。
選択余地率が 0 なら車は常に選択肢が 0 か 1 で、どんな予測器も反応型と同じ
選択しかできない = 天井は原理的に 0 に近づく。この関係を描くのが目的。

  python3 tools/run_choice_margin_sweep.py \
      --recs sim_results/rec_cm_d40 sim_results/rec_cm_d50 ...
"""
import argparse
import os
import subprocess
import sys

import pandas as pd

TOOLS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TOOLS)


def sh(cmd):
    r = subprocess.run(cmd, capture_output=True, text=True, cwd=REPO)
    return r.returncode, (r.stdout or '') + (r.stderr or '')


def index_ok(rec):
    """索引検査の結果を (使ってよいか, 注記, 出力) で返す。

    終了コード 1 = 規約が割れている (この記録は使えない)
    終了コード 2 = 判定不能 (割れている証拠は無い。注記を付けて先に進む)
    この2つを同一視すると、接続機会が乏しい条件を「壊れている」と誤って
    捨ててしまう (実測: 誤検出で 5 条件中 4 条件を落としかけた)
    """
    rc, out = sh([sys.executable, os.path.join(TOOLS, 'check_bs_index_stability.py'), rec])
    if rc == 0:
        return True, '', out
    if rc == 2:
        return True, '判定不能 (割れている証拠なし)', out
    return False, '規約が割れている', out


def measure_margin(rec):
    rc, out = sh([sys.executable, os.path.join(REPO, 'scratch/measure_choice_margin.py'), rec])
    for line in out.splitlines():
        if '選択余地率 (実測)' in line:
            return float(line.split('=')[-1].strip()), out
    return float('nan'), out


def ceiling(rec, out_dir, state_dir, jobs, gate_db):
    cmd = [sys.executable, os.path.join(TOOLS, 'replay_sweep.py'), rec,
           '--state-dir', state_dir, '--out', out_dir, '--jobs', str(jobs),
           '--baseline', 'assoc_hold', '--arms', 'assoc_hold', 'oracle',
           'kkf_cold_probe']
    if gate_db is not None:
        cmd += ['--kkf-common', 'idle_lcb_db=%s' % gate_db]
    rc, out = sh(cmd)
    agg = os.path.join(out_dir, 'analysis', 'agg.csv')
    if not os.path.exists(agg):
        return None, out
    return pd.read_csv(agg).set_index('method'), out


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--recs', nargs='+', required=True)
    ap.add_argument('--state-dir',
                    default='/workspace/sim_results/learn_fixblk/rem_state')
    ap.add_argument('--jobs', type=int, default=8)
    ap.add_argument('--gate-db', type=float, default=None,
                    help='kkf_idle_lcb_db の上書き。既定は記録の設定のまま '
                         '(生成器が接続閾値に追従させているので通常は不要)')
    ap.add_argument('--out', default='sim_results/choice_margin_summary.csv')
    a = ap.parse_args()

    rows = []
    for rec in a.recs:
        name = os.path.basename(rec.rstrip('/'))
        print('\n' + '=' * 60 + '\n' + name + '\n' + '=' * 60, flush=True)

        ok, note, out = index_ok(rec)
        lines = [x for x in out.strip().splitlines() if x.strip()]
        print(lines[-1] if lines else '(索引検査 出力なし)')
        if not ok:
            print('>>> 索引規約が割れているのでこの条件は捨てる')
            rows.append({'条件': name, '索引OK': False})
            continue
        if note:
            print('注記: ' + note)

        margin, _ = measure_margin(rec)
        print('選択余地率 (実測) = %.4f' % margin)

        agg, cout = ceiling(rec, rec + '_ceiling', a.state_dir, a.jobs, a.gate_db)
        if agg is None:
            print('>>> 再生に失敗\n' + cout[-500:])
            rows.append({'条件': name, '索引OK': True, '選択余地率': margin})
            continue

        base = float(agg.loc['assoc_hold', 'total_data_MB_mean'])
        orc = float(agg.loc['oracle', 'total_data_MB_mean'])
        probe = float(agg.loc['kkf_cold_probe', 'total_data_MB_mean'])
        rows.append({
            '条件': name, '索引OK': True, '索引注記': note,
            '選択余地率': margin,
            'assoc_hold_MB': base, 'oracle_MB': orc, 'kkf_probe_MB': probe,
            '天井_pct': 100.0 * (orc - base) / base,
            'KKF捕捉率_pct': (100.0 * (probe - base) / (orc - base)
                              if orc > base else float('nan')),
            'n': int(agg.loc['assoc_hold', 'n']),
        })
        print('天井 = %+.1f%%  (assoc_hold %.0f -> oracle %.0f MB, N=%d)'
              % (rows[-1]['天井_pct'], base, orc, rows[-1]['n']))

    t = pd.DataFrame(rows)
    if '選択余地率' in t:
        t = t.sort_values('選択余地率')
    t.to_csv(os.path.join(REPO, a.out), index=False)
    print('\n\n' + '=' * 60 + '\n=== 天井 vs 選択余地率 ===\n' + '=' * 60)
    print(t.round(3).to_string(index=False))
    print('\n[sweep] wrote ' + a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
