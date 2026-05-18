#!/usr/bin/env python3
"""
sweep_progress.py
-----------------
スイープシミュレーションの進捗を確認するスクリプト。
sweep_sim.py が出力するプログレスログ (sweep_progress.log) と
結果CSVファイルをもとに「何タスク中何タスク完了したか」を表示する。

使い方:
    python3 sweep_progress.py
    python3 sweep_progress.py --watch       # 5秒ごとに自動更新
    python3 sweep_progress.py --log-file sweep_out.log  # ログファイルを指定
"""

import argparse
import glob
import math
import os
import re
import time
import datetime

# sweep_sim.py から動的に設定を読み込む
try:
    import sweep_sim
    Y_POSITIONS = sweep_sim.Y_POSITIONS
    ANGLES_DEG  = sweep_sim.ANGLES_DEG
    TOTAL_TASKS = len(Y_POSITIONS) * len(ANGLES_DEG)
except Exception:
    Y_POSITIONS = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
    ANGLES_DEG  = list(range(0, 91))
    TOTAL_TASKS = len(Y_POSITIONS) * len(ANGLES_DEG)


PROGRESS_LOG = "sweep_progress.log"
SIM_RESULTS_DIR = "sim_results"


def parse_progress_log(log_path: str):
    """sweep_sim.py が書き出す progress log を読んで進捗情報を返す"""
    if not os.path.exists(log_path):
        return None

    completed = 0
    current_task = None
    start_time = None
    last_success_time = None
    total_tasks = None

    with open(log_path, encoding="utf-8") as f:
        for line in f:
            line = line.rstrip()
            # 開始行: START 2026-05-18T17:00:00 TOTAL=910
            m = re.match(r"START (\S+) TOTAL=(\d+)", line)
            if m:
                start_time = datetime.datetime.fromisoformat(m.group(1))
                total_tasks = int(m.group(2))
                continue
            # 完了行: DONE task=5 y=1 angle=4 status=OK ts=2026-05-18T17:01:23
            m = re.match(r"DONE task=(\d+) y=[\d\.]+ angle=[-\d\.]+ status=(\w+) ts=(\S+)", line)
            if m:
                completed = int(m.group(1))
                last_success_time = datetime.datetime.fromisoformat(m.group(3))
                continue
            # 実行中行: RUNNING task=6 y=1 angle=5 ts=2026-05-18T17:01:25
            m = re.match(r"RUNNING task=(\d+) y=([\d\.]+) angle=([-\d\.]+) ts=(\S+)", line)
            if m:
                current_task = {
                    "task_no": int(m.group(1)),
                    "y": float(m.group(2)),
                    "angle": float(m.group(3)),
                    "started_at": datetime.datetime.fromisoformat(m.group(4)),
                }

    return {
        "completed": completed,
        "current_task": current_task,
        "start_time": start_time,
        "last_success_time": last_success_time,
        "total_tasks": total_tasks,
    }



def count_csv_rows():
    """sim_results/ 内の最新 CSV から完了した run_id 数を数える"""
    csvs = sorted(glob.glob(os.path.join(SIM_RESULTS_DIR, "sweep_summary_*.csv")))
    if not csvs:
        return 0, None
    latest = csvs[-1]
    run_ids = set()
    with open(latest, encoding="utf-8") as f:
        for i, line in enumerate(f):
            if i == 0:
                continue  # header
            cols = line.strip().split(",")
            if cols and cols[0]:
                run_ids.add(cols[0])
    return len(run_ids), latest


def render(log_path: str):
    """進捗を1画面分出力する"""
    os.system("clear")
    now = datetime.datetime.now()
    print("=" * 60)
    print(f"  Sweep Sim Progress Monitor  [{now.strftime('%Y-%m-%d %H:%M:%S')}]")
    print("=" * 60)

    info = parse_progress_log(log_path)

    if info is None:
        # progress log がない場合は CSV から推定
        completed_runs, latest_csv = count_csv_rows()
        print(f"\n  [!] {PROGRESS_LOG} が見つかりません。")
        print(f"      CSV から推定: {completed_runs} run_id 完了 / 総タスク {TOTAL_TASKS}")
        if latest_csv:
            print(f"      最新CSV: {os.path.basename(latest_csv)}")
        print("\n  sweep_sim.py を最新版に更新すると詳細な進捗が表示されます。")
        print("=" * 60)
        return

    completed = info["completed"]
    total = info.get("total_tasks") or TOTAL_TASKS
    remaining = total - completed
    pct = completed / total * 100 if total > 0 else 0

    # プログレスバー (50文字幅)
    bar_width = 50
    filled = int(bar_width * completed / total) if total > 0 else 0
    bar = "█" * filled + "░" * (bar_width - filled)

    print(f"\n  進捗: {completed:4d} / {total}  ({pct:.1f}%)")
    print(f"  [{bar}]")
    print(f"  残り: {remaining} タスク")

    # 経過時間 & 推定残り時間
    if info["start_time"]:
        elapsed = now - info["start_time"]
        elapsed_s = elapsed.total_seconds()
        print(f"\n  開始時刻 : {info['start_time'].strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"  経過時間 : {str(elapsed).split('.')[0]}")
        if completed > 0:
            spt = elapsed_s / completed           # 1タスクあたり秒数
            eta_s = spt * remaining
            eta_dt = now + datetime.timedelta(seconds=eta_s)
            print(f"  1タスク平均: {spt:.1f} 秒")
            print(f"  推定残り時間: {str(datetime.timedelta(seconds=int(eta_s)))}")
            print(f"  推定完了時刻: {eta_dt.strftime('%Y-%m-%d %H:%M:%S')}")

    # 現在実行中のタスク
    if info["current_task"]:
        ct = info["current_task"]
        running_for = now - ct["started_at"]
        print(f"\n  ▶ 実行中タスク #{ct['task_no']}")
        print(f"    Y={ct['y']} m,  角度オフセット={ct['angle']}°")
        print(f"    実行時間: {str(running_for).split('.')[0]}")
    elif completed == total:
        print("\n  ✅ 全タスク完了!")

    print("\n" + "=" * 60)



def main():
    parser = argparse.ArgumentParser(description="Sweep sim progress monitor")
    parser.add_argument("--watch", action="store_true",
                        help="5秒ごとに自動更新")
    parser.add_argument("--interval", type=float, default=5.0,
                        help="--watch 時の更新間隔 (秒, デフォルト 5)")
    parser.add_argument("--log-file", default=PROGRESS_LOG,
                        help=f"進捗ログファイルのパス (デフォルト: {PROGRESS_LOG})")
    args = parser.parse_args()

    if args.watch:
        while True:
            render(args.log_file)
            print(f"  (Ctrl+C で終了。{args.interval}秒ごとに更新)")
            time.sleep(args.interval)
    else:
        render(args.log_file)


if __name__ == "__main__":
    main()
