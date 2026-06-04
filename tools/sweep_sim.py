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
import sys
import glob

# スクリプトがあるディレクトリをパスに追加し、サブディレクトリ lib からのインポートを保証する
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# =========================================================================
# パラメータスイープ設定のインポート (Single Responsibility Principle)
# =========================================================================
from lib.sweep_config import (
    Y_POSITIONS,
    RX_YAW_DEG as BASE_STATION_YAW_DEG,
    START_ANGLE,
    END_ANGLE,
    STEP_ANGLE,
    NUM_RUNS,
    TASK_TIMEOUT_SEC,
    SWEEP_REAL_TIME_FACTOR,
    PROGRESS_LOG,
    CONFIG_PATH,
    BACKUP_PATH,
    ANGLES_DEG
)

from lib.sweep_kinematics import estimate_expected_duration
from lib.sweep_data import average_summaries, get_completed_tasks

def get_optimal_concurrency() -> int:
    """CPUコア数・現在のシステム負荷・および利用可能な空きメモリ容量から最適な並列度を決定する"""
    try:
        cpu_count = os.cpu_count()
    except Exception:
        cpu_count = 4
    if cpu_count is None:
        cpu_count = 4

    load_1min = 0.0
    loadavg_path = "/proc/loadavg"
    if os.path.exists(loadavg_path):
        try:
            with open(loadavg_path, "r") as f:
                load_1min = float(f.read().split()[0])
        except Exception:
            pass

    effective_load = load_1min * 0.5
    avail_cpus = cpu_count - effective_load
    max_by_cpu = max(2, int(math.floor(avail_cpus)))

    mem_per_worker = 1.2 * (1024**3)  # bytes
    mem_available = 8 * (1024**3)     # 8 GiB fallback
    
    meminfo_path = "/proc/meminfo"
    if os.path.exists(meminfo_path):
        try:
            with open(meminfo_path, "r") as f:
                for line in f:
                    if line.startswith("MemAvailable:"):
                        mem_available = int(line.split()[1]) * 1024
                        break
        except Exception:
            pass

    safe_mem_available = mem_available * 0.8
    max_by_mem = max(1, int(math.floor(safe_mem_available / mem_per_worker)))

    optimal = min(max_by_cpu, max_by_mem)
    
    # Gazebo has a significant CPU/GPU footprint even in headless mode.
    # Running too many parallel instances causes thread starvation, DDS message drops,
    # and inconsistent telemetry logging. We cap the default auto-detected concurrency to 4.
    MAX_CONCURRENCY = 4
    optimal = min(optimal, MAX_CONCURRENCY)
    
    print(f"[Auto-detect] CPU Count: {cpu_count}, Load 1min: {load_1min:.2f} -> Max by CPU: {max_by_cpu}")
    print(f"[Auto-detect] Available Memory: {mem_available / (1024**3):.2f} GiB -> Max by Mem: {max_by_mem}")
    print(f"[Auto-detect] Optimal Concurrency (capped at {MAX_CONCURRENCY}): {optimal}")
    
    return optimal

progress_lock = threading.Lock()

def log_progress(line: str):
    """進捗ログにタイムスタンプ付きで1行書き込む (スレッドセーフ)"""
    with progress_lock:
        with open(PROGRESS_LOG, 'a', encoding='utf-8') as lf:
            lf.write(line + "\n")
            lf.flush()

def run_single_task(task_info, worker_id, sweep_start_time, total_runs_tasks, is_docker, rtf=5.0, timeout=120, base_station_yaw_deg=-90.0):
    run_idx, y, angle_deg, overall_task_no = task_info
    
    world_yaw = math.radians(angle_deg) - math.pi
    entity_yaw = math.radians(base_station_yaw_deg)
    antenna_yaw = world_yaw - entity_yaw
 
    summary_filename = f"sweep_summary_{sweep_start_time}_run{run_idx}_w{worker_id}.csv"
    tmp_config_path = f"tools/sweep_build/sim_params_tmp_{worker_id}.yaml"
 
    pct = (overall_task_no - 1) / total_runs_tasks * 100
    print(f"\n=======================================================")
    print(f"[Worker {worker_id}] [Run {run_idx}/{NUM_RUNS}] Task {overall_task_no}/{total_runs_tasks} ({pct:.1f}%) Y = {y} m, Angle = {angle_deg} deg")
    print(f"=======================================================")
    log_progress(f"RUNNING task={overall_task_no} y={y} angle={angle_deg} ts={datetime.datetime.now().isoformat()}")
 
    t_expected = estimate_expected_duration(CONFIG_PATH, rtf)
    
    # Use the user-defined timeout. Under high parallelization, actual simulation RTF drops
    # significantly below the target RTF. Terminating via a dynamic expected time is too aggressive.
    task_timeout = timeout

    max_retries = 3
    for attempt in range(max_retries):
        try:
            with open(CONFIG_PATH, 'r', encoding='utf-8') as f:
                content = f.read()
     
            content = re.sub(r'\n\s*summary_filename:\s*["\']?[^"\']*["\']?', '', content)
            content = re.sub(r'\n\s*output_subdir:\s*["\']?[^"\']*["\']?', '', content)
            content = re.sub(
                r'(simulation:)', 
                rf'\1\n  summary_filename: "{summary_filename}"\n  output_subdir: "sweep_{sweep_start_time}"', 
                content
            )
     
            content = re.sub(r'headless:\s*false', 'headless: true', content)
            content = re.sub(r'real_time_factor:\s*[\d\.]+', f'real_time_factor: {rtf}', content)
     
            content = re.sub(
                r'(antenna\w*:\s*.*?pose:\s*\[\s*)([-\d\.]+),\s*[-\d\.]+,\s*([-\d\.]+),\s*([-\d\.]+),\s*([-\d\.]+),\s*([-\d\.]+)(\s*\])',
                rf'\g<1>\g<2>, {y}, \g<3>, \g<4>, \g<5>, {entity_yaw:.4f}\g<7>',
                content,
                flags=re.DOTALL
            )
     
            content = re.sub(
                r'(antenna_relative_rpy:\s*\[\s*[-\d\.]+,\s*[-\d\.]+,\s*)[-\d\.]+(\s*\])',
                rf'\g<1>{antenna_yaw:.4f}\g<2>',
                content
            )
     
            ugv_antenna_yaw = world_yaw - math.pi
            for ant_name in ["shinkansen_front", "shinkansen_mid", "shinkansen_rear"]:
                content = re.sub(
                    rf'(name:\s*"{ant_name}".*?relative_rpy:\s*\[\s*[-\d\.]+,\s*[-\d\.]+,\s*)[-\d\.]+(\s*\])',
                    rf'\g<1>{ugv_antenna_yaw:.4f}\g<2>',
                    content,
                    flags=re.DOTALL
                )
     
            with open(tmp_config_path, 'w', encoding='utf-8') as f:
                f.write(content)
                
        except Exception as e:
            print(f"[Worker {worker_id}] Error creating config {tmp_config_path}: {e}")
            log_progress(f"DONE task={overall_task_no} y={y} angle={angle_deg} status=FAIL ts={datetime.datetime.now().isoformat()}")
            return False
     
        ros_domain_id = 10 + worker_id
        gz_partition = f"comms_sim_partition_{worker_id}"
        gz_port = 11345 + worker_id
     
        if is_docker:
            cmd = ["bash", "-c", 
                   f"export ROS_DOMAIN_ID={ros_domain_id} && "
                   f"export GZ_PARTITION={gz_partition} && "
                   f"export GZ_PORT={gz_port} && "
                   f"export FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/src/comms_sim_pkg/config/fastdds_no_shm.xml && "
                   f"source /opt/ros/humble/setup.bash && source install/setup.bash && "
                   f"ros2 launch comms_sim_pkg sim_launch.py config_file:=/workspace/{tmp_config_path}"]
        else:
            cmd = ["docker", "compose", "exec", "-T", "sim", "bash", "-c", 
                   f"export ROS_DOMAIN_ID={ros_domain_id} && "
                   f"export GZ_PARTITION={gz_partition} && "
                   f"export GZ_PORT={gz_port} && "
                   f"export FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/src/comms_sim_pkg/config/fastdds_no_shm.xml && "
                   f"source /opt/ros/humble/setup.bash && source install/setup.bash && "
                   f"ros2 launch comms_sim_pkg sim_launch.py config_file:=/workspace/{tmp_config_path}"]
     
        start_time = time.time()
        proc = subprocess.Popen(
            cmd,
            preexec_fn=os.setsid
        )
        
        timed_out = False
        try:
            proc.wait(timeout=task_timeout)
        except subprocess.TimeoutExpired:
            timed_out = True
            print(f"[Worker {worker_id}] Overall Task {overall_task_no}/{total_runs_tasks} Simulation TIMEOUT ({task_timeout}s): Y={y}, Angle={angle_deg}")
        finally:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
     
            if not is_docker:
                try:
                    subprocess.run(["docker", "compose", "exec", "-T", "sim", "pkill", "-9", "-f", f"sim_params_tmp_{worker_id}"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    subprocess.run(["docker", "compose", "exec", "-T", "sim", "pkill", "-9", "-f", f"ROS_DOMAIN_ID={ros_domain_id}"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                except Exception:
                    pass
     
            if os.path.exists(tmp_config_path):
                try:
                    os.remove(tmp_config_path)
                except Exception:
                    pass

        actual_duration = time.time() - start_time
        
        is_valid = True
        reason = ""
        
        if timed_out:
            is_valid = False
            reason = "Timeout"
        elif proc.returncode != 0:
            is_valid = False
            reason = f"Process exited with error code {proc.returncode}"
        elif t_expected is not None:
            min_expected = max(5.0, t_expected * 0.7)
            if actual_duration < min_expected:
                is_valid = False
                reason = f"Simulation ended too quickly ({actual_duration:.1f}s < minimum expected {min_expected:.1f}s)"
                
        if is_valid:
            expected_str = f"{t_expected:.1f}s" if t_expected is not None else "Unknown"
            print(f"[Worker {worker_id}] Overall Task {overall_task_no}/{total_runs_tasks} Simulation finished successfully in {actual_duration:.1f}s (Expected: {expected_str}): Y={y}, Angle={angle_deg}")
            log_progress(f"DONE task={overall_task_no} y={y} angle={angle_deg} status=OK ts={datetime.datetime.now().isoformat()}")
            return True
        else:
            attempt_info = f"Attempt {attempt + 1}/{max_retries}"
            print(f"[Worker {worker_id}] {attempt_info} FAILED: {reason}. Re-running task with same parameters...")
            time.sleep(2.0)

    print(f"[Worker {worker_id}] Overall Task {overall_task_no}/{total_runs_tasks} All {max_retries} attempts failed: Y={y}, Angle={angle_deg}")
    log_progress(f"DONE task={overall_task_no} y={y} angle={angle_deg} status=FAIL ts={datetime.datetime.now().isoformat()}")
    return False

def main():
    parser = argparse.ArgumentParser(description="Parallel Parameter Sweep Simulation")
    parser.add_argument("-j", "--concurrency", type=int, default=0, help="Number of parallel workers (0 for auto)")
    parser.add_argument("--start-angle", type=float, default=None, help="Start angle of sweep in degrees (default from config)")
    parser.add_argument("--end-angle", type=float, default=None, help="End angle of sweep in degrees (default from config)")
    parser.add_argument("--step-angle", type=float, default=None, help="Angle step in degrees (default from config)")
    parser.add_argument("--num-runs", type=int, default=None, help="Number of runs per task (default from config)")
    parser.add_argument("--y-positions", type=str, default=None, help="Comma-separated Y positions of base stations (default from config)")
    parser.add_argument("--base-station-yaw", type=float, default=None, help="Yaw angle of base station entity in degrees (default from config)")
    parser.add_argument("--rtf", "--real-time-factor", type=float, default=None, help="Acceleration factor (default from config)")
    parser.add_argument("--timeout", type=int, default=None, help="Timeout in seconds per task (default from config)")
    parser.add_argument("--resume", type=str, default=None, help="Resume a previous sweep using its timestamp or directory path")
    
    args, unknown = parser.parse_known_args()
    concurrency = args.concurrency
    if concurrency <= 0:
        concurrency = get_optimal_concurrency()

    num_runs = args.num_runs if args.num_runs is not None else NUM_RUNS
    rtf = args.rtf if args.rtf is not None else SWEEP_REAL_TIME_FACTOR
    timeout = args.timeout if args.timeout is not None else TASK_TIMEOUT_SEC
    base_station_yaw_deg = args.base_station_yaw if args.base_station_yaw is not None else BASE_STATION_YAW_DEG

    if args.y_positions is not None:
        y_positions = [float(y.strip()) for y in args.y_positions.split(',') if y.strip()]
    else:
        y_positions = Y_POSITIONS
        
    if args.start_angle is not None or args.end_angle is not None or args.step_angle is not None:
        start_angle = args.start_angle if args.start_angle is not None else min(ANGLES_DEG)
        end_angle = args.end_angle if args.end_angle is not None else max(ANGLES_DEG)
        step_angle = args.step_angle if args.step_angle is not None else 0.2
        
        angles_deg = []
        curr_angle = start_angle
        while curr_angle <= end_angle + 1e-5:
            angles_deg.append(round(curr_angle, 1))
            curr_angle += step_angle
    else:
        angles_deg = ANGLES_DEG

    os.makedirs("tools/sweep_build", exist_ok=True)

    if os.path.exists(BACKUP_PATH):
        os.remove(BACKUP_PATH)
    shutil.copy2(CONFIG_PATH, BACKUP_PATH)

    for f in glob.glob("tools/sweep_build/sim_params_tmp_*.yaml"):
        try:
            os.remove(f)
        except Exception as e:
            print(f"Warning: Failed to clean up leftover config file {f}: {e}")

    if args.resume:
        resume_input = args.resume.strip()
        if '/' in resume_input or '\\' in resume_input:
            sweep_dir = resume_input
            sweep_start_time = os.path.basename(sweep_dir).replace("sweep_", "")
        else:
            sweep_start_time = resume_input
            sweep_dir = os.path.join("sim_results", f"sweep_{sweep_start_time}")
            
        if not os.path.exists(sweep_dir):
            print(f"Error: Resume directory {sweep_dir} does not exist.")
            sys.exit(1)
        print(f"Resuming parameter sweep from existing directory: {sweep_dir}")
    else:
        sweep_start_time = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        sweep_dir = os.path.join("sim_results", f"sweep_{sweep_start_time}")

    global PROGRESS_LOG
    PROGRESS_LOG = f"tools/log/{sweep_start_time}/sweep_progress.log"
    os.makedirs(os.path.dirname(PROGRESS_LOG), exist_ok=True)

    if not args.resume and os.path.exists(PROGRESS_LOG):
        os.remove(PROGRESS_LOG)

    tasks_list = []
    overall_task_no = 1
    skipped_count = 0
    
    for run_idx in range(1, num_runs + 1):
        completed_set = set()
        if args.resume:
            completed_set = get_completed_tasks(sweep_dir, run_idx)
            
        for y in y_positions:
            for angle_deg in angles_deg:
                is_completed = False
                for cy, cang in completed_set:
                    if abs(cy - y) < 0.01 and abs(cang - angle_deg) < 0.05:
                        is_completed = True
                        break
                
                if is_completed:
                    skipped_count += 1
                else:
                    tasks_list.append((run_idx, y, angle_deg, overall_task_no))
                overall_task_no += 1

    total_tasks_per_run = len(y_positions) * len(angles_deg)
    total_runs_tasks = total_tasks_per_run * num_runs

    log_progress(f"START {datetime.datetime.now().isoformat()} TOTAL={total_runs_tasks} CONCURRENCY={concurrency}")

    print(f"Starting parameter sweep: Y_POSITIONS={y_positions}, ANGLES_DEG={angles_deg}")
    print(f"Base Station Yaw: {base_station_yaw_deg} deg")
    print(f"Number of runs per task: {num_runs}")
    print(f"Total tasks across all runs: {total_runs_tasks}")
    if skipped_count > 0:
        print(f"Resuming: Skipped {skipped_count} already completed tasks. {len(tasks_list)} tasks remaining.")
    print(f"Concurrency level: {concurrency}")

    worker_queue = queue.Queue()
    for idx in range(concurrency):
        worker_queue.put(idx)

    is_docker = os.path.exists('/.dockerenv')

    def worker_thread_fn(task_info):
        worker_id = worker_queue.get()
        try:
            success = run_single_task(task_info, worker_id, sweep_start_time, total_runs_tasks, is_docker, rtf=rtf, timeout=timeout, base_station_yaw_deg=base_station_yaw_deg)
            return success
        finally:
            worker_queue.put(worker_id)
            time.sleep(0.5)

    with concurrent.futures.ThreadPoolExecutor(max_workers=concurrency) as executor:
        futures = {executor.submit(worker_thread_fn, t): t for t in tasks_list}
        concurrent.futures.wait(futures.keys())

    print("\nMerging worker results...")
    for run_idx in range(1, num_runs + 1):
        merged_summary_file = os.path.join("sim_results", f"sweep_{sweep_start_time}", f"sweep_summary_run{run_idx}.csv")
        os.makedirs(os.path.dirname(merged_summary_file), exist_ok=True)
        
        merged_rows = []
        header = None
        if os.path.exists(merged_summary_file):
            try:
                with open(merged_summary_file, 'r', encoding='utf-8') as f:
                    lines = f.readlines()
                    if lines:
                        header = lines[0]
                        merged_rows.extend(lines[1:])
            except Exception:
                pass

        pattern = os.path.join("sim_results", f"sweep_{sweep_start_time}", f"sweep_summary_{sweep_start_time}_run{run_idx}_w*.csv")
        worker_csvs = glob.glob(pattern)
        
        for worker_csv in sorted(worker_csvs):
            if os.path.exists(worker_csv):
                try:
                    with open(worker_csv, 'r', encoding='utf-8') as infile:
                        lines = infile.readlines()
                        if lines:
                            if not header:
                                header = lines[0]
                            merged_rows.extend(lines[1:])
                except Exception as e:
                    print(f"Warning: Failed to read worker csv {worker_csv}: {e}")
                
                try:
                    os.remove(worker_csv)
                except Exception as e:
                    print(f"Warning: Failed to delete worker csv {worker_csv}: {e}")
        
        if header and merged_rows:
            with open(merged_summary_file, 'w', encoding='utf-8') as outfile:
                outfile.write(header)
                outfile.writelines(merged_rows)

    print("\nAveraging results across all runs...")
    try:
        final_summary_file = f"sim_results/sweep_{sweep_start_time}/sweep_summary.csv"
        summary_files = [os.path.join("sim_results", f"sweep_{sweep_start_time}", f"sweep_summary_run{run_idx}.csv") for run_idx in range(1, num_runs + 1)]
        average_summaries(summary_files, final_summary_file)
        print(f"Averaged summary successfully saved to: {final_summary_file}")
    except Exception as e:
        print(f"Failed to average summaries: {e}")

    log_progress(f"DONE task={total_runs_tasks} y=- angle=- status=SWEEP_COMPLETE ts={datetime.datetime.now().isoformat()}")
    print("\nSweep completed! Restoring original config...")
    shutil.copy2(BACKUP_PATH, CONFIG_PATH)

if __name__ == "__main__":
    main()
