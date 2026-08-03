#!/usr/bin/env python3
"""
「通信は運動に影響しない」という前提が、まだ成り立っているかを検査する。

なぜ要るか
----------
事後再計算 (tools/replay_sim.py) は、軌跡を一度記録して全手法で使い回す。
これは **物理が手法に依存しない** ことに全面的に依存している。現在の実装では
運動制御が VehicleMotionController::CalculateCommand(運動状態, dt) しか
参照しないため成立するが、将来これが破れると再生の結果は静かに誤りになる。

破れる典型例:
  - 隊列走行 (前車との通信で車間制御)
  - 接続維持のための速度調整
  - 遠隔運転 (制御指令が通信路を通る)
  - 協調合流・交差点制御

いずれもリアルタイム車両制御を入れた瞬間に発生する。**そのとき事後再計算は
使えない** (詳細と代替案は docs/replay_architecture.md)。

この検査は、異なる手法で同一シードの記録を取り、軌跡が完全に一致することを
確かめる。前提が破れていれば軌跡がずれるので、静かな誤りではなく失敗として
表面化する。

  python3 tools/check_motion_independence.py poses_armA.csv poses_armB.csv
"""
import argparse
import sys

import numpy as np
import pandas as pd


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('pose_csvs', nargs='+',
                    help='異なる手法・同一シードで記録した poses.csv (2つ以上)')
    ap.add_argument('--tol-m', type=float, default=1e-6,
                    help='位置の許容差 [m]')
    ap.add_argument('--tol-rad', type=float, default=1e-6,
                    help='姿勢の許容差 [rad]')
    a = ap.parse_args()

    if len(a.pose_csvs) < 2:
        sys.exit("2つ以上の記録が要ります (異なる手法・同一シード)")

    ref = pd.read_csv(a.pose_csvs[0])
    ref['t_s'] = ref['t_s'].round(6)
    ref = ref.set_index(['t_s', 'model']).sort_index()
    failures = []

    for path in a.pose_csvs[1:]:
        d = pd.read_csv(path)
        d['t_s'] = d['t_s'].round(6)
        d = d.set_index(['t_s', 'model']).sort_index()

        common = ref.index.intersection(d.index)
        only_ref = len(ref.index.difference(d.index))
        only_new = len(d.index.difference(ref.index))
        if not len(common):
            failures.append(f"{path}: 共通の (時刻, モデル) が無い")
            continue

        dp = np.abs(ref.loc[common, ['x', 'y', 'z']].to_numpy()
                    - d.loc[common, ['x', 'y', 'z']].to_numpy()).max()
        dr = np.abs(ref.loc[common, ['roll', 'pitch', 'yaw']].to_numpy()
                    - d.loc[common, ['roll', 'pitch', 'yaw']].to_numpy()).max()
        print(f"{path}: 照合 {len(common):,} / 片側のみ {only_ref + only_new:,} / "
              f"位置差 {dp:.3e} m / 姿勢差 {dr:.3e} rad")
        if dp > a.tol_m or dr > a.tol_rad:
            failures.append(f"{path}: 軌跡が一致しない (位置 {dp:.3e} m, 姿勢 {dr:.3e} rad)")
        # 片側のみは走行終了時刻の差で出るので、比率が大きいときだけ問題にする
        if (only_ref + only_new) > 0.02 * len(common):
            failures.append(f"{path}: 記録範囲の食い違いが大きい "
                            f"({only_ref + only_new:,} / {len(common):,})")

    if failures:
        print("\nNG: 通信が運動に影響しているか、記録条件が揃っていません")
        for f in failures:
            print(f"  - {f}")
        print("\n前提が破れている場合、事後再計算 (replay_sim.py) の結果は誤りです。"
              "\nリアルタイム制御を入れたのであれば docs/replay_architecture.md の"
              "\n代替案 (ステップ駆動の協調シミュレーション) を検討してください。")
        return 1
    print("\nOK: 軌跡は手法に依存していない (事後再計算の前提は成立)")
    return 0


if __name__ == '__main__':
    sys.exit(main())
