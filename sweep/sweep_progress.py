#!/usr/bin/env python3
"""
sweep_progress.py
-----------------
スイープシミュレーションの進捗を確認するスクリプト。
sweep_sim.py が出力するプログレスログ (sweep_progress.log) と
結果CSVファイルをもとに「並列化の状況」「完了時間(ETA)」「CPU/メモリ状況」を表示する。

使い方:
    python3 sweep_progress.py
    python3 sweep_progress.py --watch       # 5秒ごとに自動更新
    python3 sweep_progress.py --log-file sweep_out.log  # ログファイルを指定
"""

import argparse
import glob
import math
import os
import sys
import re
import time
import datetime
import subprocess

# 同一ディレクトリにある sweep_sim.py から動的に設定を読み込めるようにパスを追加
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

try:
    import sweep_sim
    Y_POSITIONS = sweep_sim.Y_POSITIONS
    ANGLES_DEG  = sweep_sim.ANGLES_DEG
    TOTAL_TASKS = len(Y_POSITIONS) * len(ANGLES_DEG) * sweep_sim.NUM_RUNS
except Exception:
    Y_POSITIONS = [1.0]
    ANGLES_DEG  = [round(0.2 * i, 2) for i in range(76)]
    TOTAL_TASKS = len(Y_POSITIONS) * len(ANGLES_DEG) * 10

PROGRESS_LOG_DEFAULT = "sweep/log/sweep_progress.log"
SIM_RESULTS_DIR = "sim_results"

# システムリソース検出ライブラリのロード試行
try:
    import psutil
    HAS_PSUTIL = True
except ImportError:
    HAS_PSUTIL = False

def find_latest_progress_log():
    """sweep/log/ 配下から最も新しいタイムスタンプフォルダ内の sweep_progress.log を探す"""
    pattern = os.path.join("sweep", "log", "*", "sweep_progress.log")
    logs = glob.glob(pattern)
    if not logs:
        # フォールバックとして sweep/log/sweep_progress.log や sweep_progress.log も探す
        fallback_patterns = [
            os.path.join("sweep", "log", "sweep_progress.log"),
            "sweep_progress.log"
        ]
        for p in fallback_patterns:
            if os.path.exists(p):
                return p
        return PROGRESS_LOG_DEFAULT
    
    # タイムスタンプ付きディレクトリをソートして最新のものを取得
    logs.sort()
    return logs[-1]

PROGRESS_LOG = find_latest_progress_log()

def parse_progress_log(log_path: str):
    """sweep_sim.py が書き出す progress log を読んで進捗情報を返す"""
    if not os.path.exists(log_path):
        return None

    completed_task_ids = set()
    running_tasks = {}
    task_start_times = {}
    task_durations = []
    
    start_time = None
    last_success_time = None
    total_tasks = None
    concurrency = None

    with open(log_path, encoding="utf-8") as f:
        for line in f:
            line = line.rstrip()
            # 開始行: START 2026-05-18T17:00:00 TOTAL=910 CONCURRENCY=8
            m = re.match(r"START (\S+) TOTAL=(\d+)(?:\s+CONCURRENCY=(\d+))?", line)
            if m:
                start_time = datetime.datetime.fromisoformat(m.group(1))
                total_tasks = int(m.group(2))
                if m.group(3):
                    concurrency = int(m.group(3))
                continue
                
            # 実行中行: RUNNING task=6 y=1 angle=5 ts=2026-05-18T17:01:25
            m = re.match(r"RUNNING task=(\d+) y=([\d\.]+) angle=([-\d\.]+) ts=(\S+)", line)
            if m:
                t_id = int(m.group(1))
                y_val = float(m.group(2))
                ang_val = float(m.group(3))
                start_ts = datetime.datetime.fromisoformat(m.group(4))
                
                task_start_times[t_id] = start_ts
                running_tasks[t_id] = {
                    "task_no": t_id,
                    "y": y_val,
                    "angle": ang_val,
                    "started_at": start_ts
                }
                continue

            # 完了行: DONE task=5 y=1 angle=4 status=OK ts=2026-05-18T17:01:23
            m = re.match(r"DONE task=(\d+) y=([\d\.-]+) angle=([-\d\.-]+) status=(\w+) ts=(\S+)", line)
            if m:
                t_id = int(m.group(1))
                status = m.group(4)
                
                if status == "SWEEP_COMPLETE":
                    continue
                    
                done_ts = datetime.datetime.fromisoformat(m.group(5))
                completed_task_ids.add(t_id)
                running_tasks.pop(t_id, None)
                last_success_time = done_ts
                
                if t_id in task_start_times:
                    duration = (done_ts - task_start_times[t_id]).total_seconds()
                    task_durations.append(duration)
                continue

    return {
        "completed": len(completed_task_ids),
        "completed_ids": completed_task_ids,
        "running_tasks": running_tasks,
        "start_time": start_time,
        "last_success_time": last_success_time,
        "total_tasks": total_tasks,
        "concurrency": concurrency,
        "task_durations": task_durations
    }

def count_csv_rows():
    """sim_results/ 内の最新 CSV から完了した run_id 数を数える"""
    csvs = sorted(glob.glob(os.path.join(SIM_RESULTS_DIR, "sweep_*", "sweep_summary_run*.csv")))
    if not csvs:
        csvs = sorted(glob.glob(os.path.join(SIM_RESULTS_DIR, "sweep_summary_*.csv")))
    if not csvs:
        return 0, None
    latest = csvs[-1]
    run_ids = set()
    try:
        with open(latest, encoding="utf-8") as f:
            for i, line in enumerate(f):
                if i == 0:
                    continue  # header
                cols = line.strip().split(",")
                if cols and cols[0]:
                    run_ids.add(cols[0])
    except Exception:
        pass
    return len(run_ids), latest

def clear_screen():
    os.system("cls" if os.name == "nt" else "clear")

def make_progress_bar(percent, width=40):
    filled = int(width * percent / 100)
    filled = max(0, min(width, filled))
    return "█" * filled + "░" * (width - filled)

def get_cpu_usage(interval=None):
    """CPU使用率の取得"""
    if HAS_PSUTIL:
        try:
            return psutil.cpu_percent(interval=interval)
        except Exception:
            pass
    
    # Linux /proc/stat のフォールバック
    if os.path.exists("/proc/stat"):
        try:
            def read_cpu_ticks():
                with open("/proc/stat", "r") as f:
                    line = f.readline()
                parts = line.split()
                if len(parts) >= 5:
                    ticks = [float(x) for x in parts[1:]]
                    total = sum(ticks)
                    idle = ticks[3] + (ticks[4] if len(ticks) > 4 else 0)
                    return total, idle
                return 0, 0
            
            t1, i1 = read_cpu_ticks()
            time.sleep(interval or 0.1)
            t2, i2 = read_cpu_ticks()
            
            dt = t2 - t1
            di = i2 - i1
            if dt > 0:
                return (1.0 - di / dt) * 100.0
        except Exception:
            pass
            
    return 0.0

def get_memory_usage():
    """メモリ使用状況の取得 (total_bytes, used_bytes, available_bytes, percent)"""
    if HAS_PSUTIL:
        try:
            mem = psutil.virtual_memory()
            return mem.total, mem.used, mem.available, mem.percent
        except Exception:
            pass
            
    # Linux /proc/meminfo のフォールバック
    if os.path.exists("/proc/meminfo"):
        try:
            mem_info = {}
            with open("/proc/meminfo", "r") as f:
                for line in f:
                    parts = line.split()
                    if len(parts) >= 2:
                        key = parts[0].rstrip(":")
                        val = int(parts[1])
                        mem_info[key] = val
            total = mem_info.get("MemTotal", 0) * 1024
            available = mem_info.get("MemAvailable", 0) * 1024
            if not available:
                free = mem_info.get("MemFree", 0) * 1024
                buffers = mem_info.get("Buffers", 0) * 1024
                cached = mem_info.get("Cached", 0) * 1024
                available = free + buffers + cached
            used = total - available
            pct = (used / total) * 100 if total > 0 else 0
            return total, used, available, pct
        except Exception:
            pass
            
    return 0, 0, 0, 0.0

def get_gpu_usage():
    """GPUの状況取得 (nvidia-smi を使用)"""
    import shutil
    if not shutil.which("nvidia-smi"):
        return None
    try:
        res = subprocess.run(
            ["nvidia-smi", "--query-gpu=name,utilization.gpu,utilization.memory,memory.used,memory.total", "--format=csv,noheader,nounits"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=1
        )
        if res.returncode == 0:
            gpus = []
            for line in res.stdout.strip().split("\n"):
                if not line.strip():
                    continue
                parts = [p.strip() for p in line.split(",")]
                if len(parts) >= 5:
                    gpus.append({
                        "name": parts[0],
                        "gpu_util": float(parts[1]),
                        "mem_util": float(parts[2]),
                        "mem_used": float(parts[3]),
                        "mem_total": float(parts[4])
                    })
            return gpus
    except Exception:
        pass
    return None

def render(log_path: str):
    """進捗を1画面分出力する"""
    clear_screen()
    now = datetime.datetime.now()
    print("=" * 75)
    print(f"       🚀 Sweep Sim Progress Monitor  [{now.strftime('%Y-%m-%d %H:%M:%S')}]")
    print("=" * 75)

    info = parse_progress_log(log_path)

    if info is None:
        # progress log がない場合は CSV から推定
        completed_runs, latest_csv = count_csv_rows()
        print(f"\n  [!] {log_path} が見つかりません。")
        print(f"      CSV から推定: {completed_runs} run_id 完了 / 総タスク {TOTAL_TASKS}")
        if latest_csv:
            print(f"      最新CSV: {os.path.basename(latest_csv)}")
        print("\n  sweep_sim.py を最新版に更新すると詳細な進捗が表示されます。")
        print("=" * 75)
        return

    completed = info["completed"]
    total = info.get("total_tasks") or TOTAL_TASKS
    remaining = total - completed
    pct = completed / total * 100 if total > 0 else 0

    # 1. 進捗バー (40文字幅)
    bar = make_progress_bar(pct, width=40)
    print(f"\n  📊 進捗: {completed:4d} / {total} タスク  ({pct:.1f}%)")
    print(f"  [{bar}]")
    print(f"  残り: {remaining} タスク")

    # 2. システムリソース状況
    print("\n  💻 システムリソース状況:")
    
    # CPU
    cpu_pct = get_cpu_usage(interval=0.1)
    cpu_bar = make_progress_bar(cpu_pct, width=20)
    print(f"    CPU 使用率:  [{cpu_bar}] {cpu_pct:5.1f}%")
    
    # Memory
    mem_total, mem_used, mem_avail, mem_pct = get_memory_usage()
    if mem_total > 0:
        mem_bar = make_progress_bar(mem_pct, width=20)
        total_gb = mem_total / (1024**3)
        used_gb = mem_used / (1024**3)
        print(f"    メモリ使用量: [{mem_bar}] {used_gb:5.1f} / {total_gb:.1f} GiB ({mem_pct:.1f}%)")
    else:
        print("    メモリ使用量: 取得できませんでした")

    # GPU
    gpus = get_gpu_usage()
    if gpus:
        print("\n    GPU 状況:")
        for idx, gpu in enumerate(gpus):
            gpu_bar = make_progress_bar(gpu['gpu_util'], width=15)
            gpu_mem_pct = (gpu['mem_used'] / gpu['mem_total'] * 100) if gpu['mem_total'] > 0 else 0
            gpu_mem_bar = make_progress_bar(gpu_mem_pct, width=15)
            print(f"      GPU #{idx} [{gpu['name']}]:")
            print(f"        負荷: [{gpu_bar}] {gpu['gpu_util']:5.1f}%")
            print(f"        メモリ: [{gpu_mem_bar}] {gpu['mem_used']:.0f} / {gpu['mem_total']:.0f} MiB ({gpu_mem_pct:.1f}%)")

    # 3. 経過時間 & 推定残り時間
    print("\n  ⏱️ 時間計測 & 進捗速度:")
    if info["start_time"]:
        elapsed = now - info["start_time"]
        elapsed_s = elapsed.total_seconds()
        print(f"    開始時刻 : {info['start_time'].strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"    経過時間 : {str(elapsed).split('.')[0]}")
        
        if completed > 0:
            # システム全体の処理スピード (1タスクあたりの経過秒数)
            spt = elapsed_s / completed
            eta_s = spt * remaining
            eta_dt = now + datetime.timedelta(seconds=eta_s)
            
            # タスク完了速度の表現
            tasks_per_min = 60.0 / spt if spt > 0 else 0.0
            print(f"    処理速度 : 1タスクあたり平均 {spt:.1f} 秒 (全体スループット: {tasks_per_min:.1f} タスク/分)")
            
            # 各タスクのシミュレーション実行時間の平均値
            if info["task_durations"]:
                avg_dur = sum(info["task_durations"]) / len(info["task_durations"])
                print(f"    シミュレーション平均実行時間: {avg_dur:.1f} 秒 (1インスタンス単体)")
            
            print(f"    推定残り時間: {str(datetime.timedelta(seconds=int(eta_s)))}")
            print(f"    推定完了時刻: {eta_dt.strftime('%Y-%m-%d %H:%M:%S')}")
        else:
            print("    処理速度 : 算出中 (最初のタスク完了待ち)...")
    else:
        print("    開始時刻 : ログに記録されていません")

    # 4. 現在実行中のタスク
    running_tasks = info["running_tasks"]
    concurrency_limit = info.get("concurrency")
    limit_str = f" / 最大並列数: {concurrency_limit}" if concurrency_limit else ""
    
    print(f"\n  🔄 実行中の並列タスク ({len(running_tasks)} 個のアクティブワーカー{limit_str}):")
    if running_tasks:
        for t_id in sorted(running_tasks.keys()):
            ct = running_tasks[t_id]
            running_for = now - ct["started_at"]
            run_sec = int(running_for.total_seconds())
            print(f"    ▶ タスク #{ct['task_no']:3d} | Y={ct['y']:4.1f}m, 角度={ct['angle']:5.1f}° | 実行時間: {run_sec:3d}秒")
    elif completed == total:
        print("    ✅ 全タスク完了しました!")
    else:
        print("    💤 現在実行中のタスクはありません（待機中または終了中）")

    print("\n" + "=" * 75)

def main():
    parser = argparse.ArgumentParser(description="Sweep sim progress monitor")
    parser.add_argument("--watch", action="store_true",
                        help="自動更新モード")
    parser.add_argument("--interval", type=float, default=5.0,
                        help="--watch 時の更新間隔 (秒, デフォルト 5)")
    parser.add_argument("--log-file", default=None,
                        help=f"進捗ログファイルのパス (デフォルト: 最新のログファイル)")
    args = parser.parse_args()

    log_file = args.log_file
    if not log_file:
        log_file = find_latest_progress_log()

    if args.watch:
        try:
            while True:
                render(log_file)
                print(f"  (Ctrl+C で終了。{args.interval}秒ごとに更新)")
                time.sleep(args.interval)
        except KeyboardInterrupt:
            print("\n進捗監視を終了します。")
    else:
        render(log_file)

if __name__ == "__main__":
    main()
