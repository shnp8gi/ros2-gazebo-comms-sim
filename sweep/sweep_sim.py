#!/usr/bin/env python3
import subprocess
import signal
import os
import math
import time
import shutil
import re
import datetime
import concurrent.futures
import threading
import queue
import argparse

PROGRESS_LOG = "sweep/log/sweep_progress.log"
# 1タスクあたりの最大待機時間 [秒]
TASK_TIMEOUT_SEC = 120
# スイープ時の加速倍率 (ヘッドレス時のみ有効。1.0=リアルタイム)
SWEEP_REAL_TIME_FACTOR = 5.0
# パラメータスイープを繰り返す回数
NUM_RUNS = 10

def get_optimal_concurrency() -> int:
    """CPUコア数と利用可能なメモリ容量から最適な並列度を決定する"""
    # 1. CPU制限の計算 (物理コア of 75%)
    try:
        cpu_count = os.cpu_count()
    except Exception:
        cpu_count = 4
    if cpu_count is None:
        cpu_count = 4
    max_by_cpu = max(1, int(cpu_count * 0.75))

    # 2. メモリ制限の取得
    mem_limit = None

    # (a) cgroup v2
    cgroup2_path = "/sys/fs/cgroup/memory.max"
    if os.path.exists(cgroup2_path):
        try:
            with open(cgroup2_path, "r") as f:
                val = f.read().strip()
                if val != "max":
                    mem_limit = int(val)
        except Exception:
            pass

    # (b) cgroup v1 (フォールバック)
    if mem_limit is None:
        cgroup1_path = "/sys/fs/cgroup/memory/memory.limit_in_bytes"
        if os.path.exists(cgroup1_path):
            try:
                with open(cgroup1_path, "r") as f:
                    val = f.read().strip()
                    limit = int(val)
                    if limit < 9223372036854771712:
                        mem_limit = limit
            except Exception:
                pass

    # (c) /proc/meminfo (フォールバック)
    if mem_limit is None:
        meminfo_path = "/proc/meminfo"
        if os.path.exists(meminfo_path):
            try:
                with open(meminfo_path, "r") as f:
                    for line in f:
                        if line.startswith("MemTotal:"):
                            parts = line.split()
                            if len(parts) >= 2:
                                kb = int(parts[1])
                                mem_limit = kb * 1024
                            break
            except Exception:
                pass

    # (d) os.sysconf (最終フォールバック)
    if mem_limit is None:
        try:
            mem_limit = os.sysconf('SC_PAGE_SIZE') * os.sysconf('SC_PHYS_PAGES')
        except Exception:
            mem_limit = 4 * 1024 * 1024 * 1024  # 4 GiB fallback

    # 3. メモリに基づく最大並列度の計算
    safety_margin = 2.0 * 1024 * 1024 * 1024  # 2.0 GiB
    instance_memory = 700 * 1024 * 1024       # 700 MiB

    max_by_mem = int((mem_limit - safety_margin) / instance_memory)
    max_by_mem = max(1, max_by_mem)

    # 4. 最適な並列度の決定
    optimal = min(max_by_cpu, max_by_mem)
    
    print(f"[Auto-detect] CPU Count: {cpu_count} -> Max by CPU: {max_by_cpu}")
    print(f"[Auto-detect] Memory Limit: {mem_limit / (1024**3):.2f} GiB -> Max by Mem: {max_by_mem}")
    print(f"[Auto-detect] Optimal Concurrency: {optimal}")
    
    return optimal

progress_lock = threading.Lock()

def log_progress(line: str):
    """進捗ログにタイムスタンプ付きで1行書き込む (スレッドセーフ)"""
    with progress_lock:
        with open(PROGRESS_LOG, 'a', encoding='utf-8') as lf:
            lf.write(line + "\n")
            lf.flush()

# ---------------------------------------------------------
# パラメータスイープ設定
# ---------------------------------------------------------
Y_POSITIONS = [1.0] # 基地局のY位置 (m)
ANGLES_DEG = [round(0.2 * i, 2) for i in range(76)]

CONFIG_PATH = "config/sim_params.yaml"
BACKUP_PATH = "config/sim_params.yaml.bak"

def average_summaries(summary_files, output_file):
    """各ランのCSVファイルを読み込んで平均値を集計・保存する"""
    import pandas as pd
    dfs = []
    for f in summary_files:
        if os.path.exists(f):
            try:
                dfs.append(pd.read_csv(f))
            except Exception as e:
                print(f"Warning: Failed to read {f}: {e}")
    if not dfs:
        print("No successful sweep summary files found to average.")
        return

    # すべてのデータフレームを結合
    combined = pd.concat(dfs, ignore_index=True)

    # 平均化する数値カラムの集計ルール
    agg_rules = {
        'total_data_MB': 'mean',
        'connected_time_s': 'mean',
        'average_throughput_Gbps': 'mean',
        'average_rssi_dBm': 'mean',
        'handover_count': 'mean'
    }

    # y_position, antenna_angle, vehicle_name でグループ化して平均値を計算
    averaged = combined.groupby(['y_position', 'antenna_angle', 'vehicle_name'], as_index=False).agg(agg_rules)

    # 表示をきれいにするために四捨五入
    averaged['total_data_MB'] = averaged['total_data_MB'].round(3)
    averaged['connected_time_s'] = averaged['connected_time_s'].round(3)
    averaged['average_throughput_Gbps'] = averaged['average_throughput_Gbps'].round(3)
    averaged['average_rssi_dBm'] = averaged['average_rssi_dBm'].round(3)
    averaged['handover_count'] = averaged['handover_count'].round(1)

    # run_id カラムの先頭への挿入
    averaged.insert(0, 'run_id', 'AVERAGE')

    # 並び順をソートして保存
    averaged = averaged.sort_values(by=['y_position', 'antenna_angle', 'vehicle_name'])
    
    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    averaged.to_csv(output_file, index=False)

def run_single_task(task_info, worker_id, sweep_start_time, total_runs_tasks, is_docker):
    run_idx, y, angle_deg, overall_task_no = task_info
    angle_rad = math.radians(angle_deg)
    
    # 基地局は -y 方向(yaw=-1.5708)を向いており、アンテナはそこから -90度(yaw=-1.5708) で -x を向く
    # それに angle_rad を足してスイープする
    base_yaw = -1.5708
    antenna_yaw = -1.5708 + angle_rad

    summary_filename = f"sweep_summary_{sweep_start_time}_run{run_idx}_w{worker_id}.csv"
    tmp_config_path = f"config/sim_params_tmp_{worker_id}.yaml"

    pct = (overall_task_no - 1) / total_runs_tasks * 100
    print(f"\n=======================================================")
    print(f"[Worker {worker_id}] [Run {run_idx}/{NUM_RUNS}] Task {overall_task_no}/{total_runs_tasks} ({pct:.1f}%) Y = {y} m, Angle = {angle_deg} deg")
    print(f"=======================================================")
    log_progress(f"RUNNING task={overall_task_no} y={y} angle={angle_deg} ts={datetime.datetime.now().isoformat()}")

    # YAMLファイルを文字列として読み込み、一時設定ファイルを生成
    try:
        with open(CONFIG_PATH, 'r', encoding='utf-8') as f:
            content = f.read()

        # summary_filenameの更新
        content = re.sub(r'\n\s*summary_filename:\s*".*?"', '', content)
        content = re.sub(r'(simulation:)', rf'\1\n  summary_filename: "{summary_filename}"', content)

        # headless: false -> true
        content = re.sub(r'headless:\s*false', 'headless: true', content)

        # real_time_factor を高速化
        content = re.sub(r'real_time_factor:\s*[\d\.]+', f'real_time_factor: {SWEEP_REAL_TIME_FACTOR}', content)

        # 基地局Y座標の更新 (antenna_0 / antenna_1 の pose)
        content = re.sub(
            r'(antenna\w*:\s*.*?pose:\s*\[\s*)([-\d\.]+),\s*[-\d\.]+(.*?\])', 
            rf'\g<1>\g<2>, {y}\g<3>', 
            content, 
            flags=re.DOTALL
        )

        # 基地局アンテナ角度の更新 (antenna_relative_rpy)
        content = re.sub(
            r'(antenna_relative_rpy:\s*\[\s*[-\d\.]+,\s*[-\d\.]+,\s*)[-\d\.]+(\s*\])',
            rf'\g<1>{antenna_yaw:.4f}\g<2>',
            content
        )

        # 新幹線のアンテナ角度の更新
        for ant_name in ["shinkansen_front", "shinkansen_mid", "shinkansen_rear"]:
            content = re.sub(
                rf'(name:\s*"{ant_name}".*?relative_rpy:\s*\[\s*[-\d\.]+,\s*[-\d\.]+,\s*)[-\d\.]+(\s*\])',
                rf'\g<1>{angle_rad:.4f}\g<2>',
                content,
                flags=re.DOTALL
            )

        with open(tmp_config_path, 'w', encoding='utf-8') as f:
            f.write(content)
            
    except Exception as e:
        print(f"[Worker {worker_id}] Error creating config {tmp_config_path}: {e}")
        log_progress(f"DONE task={overall_task_no} y={y} angle={angle_deg} status=FAIL ts={datetime.datetime.now().isoformat()}")
        return False

    # コマンドの構築（ROS_DOMAIN_ID, GZ_PARTITION, GZ_PORT の分離）
    ros_domain_id = 10 + worker_id
    gz_partition = f"comms_sim_partition_{worker_id}"
    gz_port = 11345 + worker_id

    if is_docker:
        cmd = ["bash", "-c", 
               f"export ROS_DOMAIN_ID={ros_domain_id} && "
               f"export GZ_PARTITION={gz_partition} && "
               f"export GZ_PORT={gz_port} && "
               f"export FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/config/fastdds_no_shm.xml && "
               f"source /opt/ros/humble/setup.bash && source install/setup.bash && "
               f"ros2 launch comms_sim_pkg sim_launch.py config_file:=/workspace/{tmp_config_path}"]
    else:
        cmd = ["docker", "compose", "exec", "-T", "sim", "bash", "-c", 
               f"export ROS_DOMAIN_ID={ros_domain_id} && "
               f"export GZ_PARTITION={gz_partition} && "
               f"export GZ_PORT={gz_port} && "
               f"export FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/config/fastdds_no_shm.xml && "
               f"source /opt/ros/humble/setup.bash && source install/setup.bash && "
               f"ros2 launch comms_sim_pkg sim_launch.py config_file:=/workspace/{tmp_config_path}"]

    proc = subprocess.Popen(
        cmd,
        preexec_fn=os.setsid
    )
    
    timed_out = False
    try:
        proc.wait(timeout=TASK_TIMEOUT_SEC)
    except subprocess.TimeoutExpired:
        timed_out = True
        print(f"[Worker {worker_id}] Overall Task {overall_task_no}/{total_runs_tasks} Simulation TIMEOUT ({TASK_TIMEOUT_SEC}s): Y={y}, Angle={angle_deg}")
        log_progress(f"DONE task={overall_task_no} y={y} angle={angle_deg} status=TIMEOUT ts={datetime.datetime.now().isoformat()}")
    finally:
        # プロセスグループ全体を強制終了
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass

        # ホスト実行時の残存プロセスをこのワーカー分だけ kill
        if not is_docker:
            try:
                subprocess.run(["docker", "compose", "exec", "-T", "sim", "pkill", "-9", "-f", f"sim_params_tmp_{worker_id}"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                subprocess.run(["docker", "compose", "exec", "-T", "sim", "pkill", "-9", "-f", f"ROS_DOMAIN_ID={ros_domain_id}"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except Exception:
                pass

        # 一時ファイルのクリーンアップ
        if os.path.exists(tmp_config_path):
            try:
                os.remove(tmp_config_path)
            except Exception:
                pass

    if not timed_out:
        if proc.returncode == 0:
            print(f"[Worker {worker_id}] Overall Task {overall_task_no}/{total_runs_tasks} Simulation finished: Y={y}, Angle={angle_deg}")
            log_progress(f"DONE task={overall_task_no} y={y} angle={angle_deg} status=OK ts={datetime.datetime.now().isoformat()}")
            return True
        else:
            print(f"[Worker {worker_id}] Overall Task {overall_task_no}/{total_runs_tasks} Simulation FAILED (rc={proc.returncode}): Y={y}, Angle={angle_deg}")
            log_progress(f"DONE task={overall_task_no} y={y} angle={angle_deg} status=FAIL ts={datetime.datetime.now().isoformat()}")
            return False
    return False

def main():
    parser = argparse.ArgumentParser(description="Parallel Parameter Sweep Simulation")
    parser.add_argument("-j", "--concurrency", type=int, default=0, help="Number of parallel workers (0 for auto)")
    args, unknown = parser.parse_known_args()
    concurrency = args.concurrency
    if concurrency <= 0:
        concurrency = get_optimal_concurrency()

    # 常に実行時の最新設定ファイルをバックアップする
    if os.path.exists(BACKUP_PATH):
        os.remove(BACKUP_PATH)
    shutil.copy2(CONFIG_PATH, BACKUP_PATH)

    sweep_start_time = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')

    # 進捗ログの保存先を実行時のタイムスタンプサブディレクトリ配下に動的決定
    global PROGRESS_LOG
    PROGRESS_LOG = f"sweep/log/{sweep_start_time}/sweep_progress.log"
    os.makedirs(os.path.dirname(PROGRESS_LOG), exist_ok=True)

    total_tasks_per_run = len(Y_POSITIONS) * len(ANGLES_DEG)
    total_runs_tasks = total_tasks_per_run * NUM_RUNS

    # 進捗ログを初期化
    if os.path.exists(PROGRESS_LOG):
        os.remove(PROGRESS_LOG)
    log_progress(f"START {datetime.datetime.now().isoformat()} TOTAL={total_runs_tasks} CONCURRENCY={concurrency}")

    print(f"Starting parameter sweep: Y_POSITIONS={Y_POSITIONS}, ANGLES_DEG={ANGLES_DEG}")
    print(f"Number of runs per task: {NUM_RUNS}")
    print(f"Total tasks across all runs: {total_runs_tasks}")
    print(f"Concurrency level: {concurrency}")

    # タスクリストの作成
    tasks_list = []
    overall_task_no = 1
    for run_idx in range(1, NUM_RUNS + 1):
        for y in Y_POSITIONS:
            for angle_deg in ANGLES_DEG:
                tasks_list.append((run_idx, y, angle_deg, overall_task_no))
                overall_task_no += 1

    # ワーカーID管理キュー
    worker_queue = queue.Queue()
    for idx in range(concurrency):
        worker_queue.put(idx)

    is_docker = os.path.exists('/.dockerenv')

    def worker_thread_fn(task_info):
        worker_id = worker_queue.get()
        try:
            success = run_single_task(task_info, worker_id, sweep_start_time, total_runs_tasks, is_docker)
            return success
        finally:
            worker_queue.put(worker_id)
            time.sleep(0.5)  # 各シミュレーション終了後のクールダウン

    # スレッドプールで並列実行
    with concurrent.futures.ThreadPoolExecutor(max_workers=concurrency) as executor:
        futures = {executor.submit(worker_thread_fn, t): t for t in tasks_list}
        concurrent.futures.wait(futures.keys())

    # ---------------------------------------------------------
    # 各ワーカーが書き出した CSV ファイルを run_idx ごとに結合する
    # ---------------------------------------------------------
    print("\nMerging worker results...")
    for run_idx in range(1, NUM_RUNS + 1):
        merged_summary_file = os.path.join("sim_results", f"sweep_{sweep_start_time}", f"sweep_summary_run{run_idx}.csv")
        os.makedirs(os.path.dirname(merged_summary_file), exist_ok=True)
        
        header_written = False
        with open(merged_summary_file, 'w', encoding='utf-8') as outfile:
            for w_id in range(concurrency):
                worker_csv = os.path.join("sim_results", f"sweep_{sweep_start_time}", f"sweep_summary_{sweep_start_time}_run{run_idx}_w{w_id}.csv")
                if os.path.exists(worker_csv):
                    with open(worker_csv, 'r', encoding='utf-8') as infile:
                        lines = infile.readlines()
                        if not lines:
                            continue
                        if not header_written:
                            outfile.writelines(lines)
                            header_written = True
                        else:
                            outfile.writelines(lines[1:])
                    # クリーンアップ
                    try:
                        os.remove(worker_csv)
                    except Exception as e:
                        print(f"Warning: Failed to delete worker csv {worker_csv}: {e}")

    # 平均化の処理を実行
    print("\nAveraging results across all runs...")
    try:
        final_summary_file = f"sim_results/sweep_{sweep_start_time}/sweep_summary.csv"
        summary_files = [os.path.join("sim_results", f"sweep_{sweep_start_time}", f"sweep_summary_run{run_idx}.csv") for run_idx in range(1, NUM_RUNS + 1)]
        average_summaries(summary_files, final_summary_file)
        print(f"Averaged summary successfully saved to: {final_summary_file}")
    except Exception as e:
        print(f"Failed to average summaries: {e}")

    log_progress(f"DONE task={total_runs_tasks} y=- angle=- status=SWEEP_COMPLETE ts={datetime.datetime.now().isoformat()}")
    print("\nSweep completed! Restoring original config...")
    shutil.copy2(BACKUP_PATH, CONFIG_PATH)

if __name__ == "__main__":
    main()
