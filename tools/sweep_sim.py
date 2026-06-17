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
import psutil
import atexit

# Initialize psutil CPU measurement reference point
try:
    psutil.cpu_percent(interval=None)
except Exception:
    pass

# スクリプトがあるディレクトリをパスに追加し、サブディレクトリ lib からのインポートを保証する
_script_dir = os.path.dirname(os.path.realpath(__file__))
if _script_dir not in sys.path:
    sys.path.insert(0, _script_dir)
# フォールバック: /workspace/tools を明示的に追加（Docker内で相対パス実行時の対策）
_tools_dir = os.path.join(os.path.dirname(_script_dir), "tools") if os.path.basename(_script_dir) != "tools" else _script_dir
if _tools_dir != _script_dir and _tools_dir not in sys.path and os.path.isdir(os.path.join(_tools_dir, "lib")):
    sys.path.insert(0, _tools_dir)

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

# 動的プロファイリングがまだ十分に機能していない場合の初期想定負荷パラメータ
DEFAULT_CPU_PER_SIM = 1.5
DEFAULT_MEM_PER_SIM_GIB = 1.2
LAUNCH_COOLDOWN_SEC = 3.0  # 起動時の負荷スパイクとロードアベレージ遅延を防ぐため、新規起動の間隔を最低3秒空ける

# =========================================================================
# 終了シグナルのハンドリングとアクティブプロセスの追跡
# =========================================================================
active_tasks_lock = threading.RLock()
active_tasks = {}  # worker_id -> { 'proc': proc, 'tmp_config_path': tmp_config_path, 'ros_domain_id': ros_domain_id }
shutdown_requested = False
sweep_start_time = None

launch_lock = threading.Lock()
last_launch_time = 0.0

def send_sigint_to_worker(proc, worker_id):
    """Sends SIGINT to the process group and all child processes."""
    if proc:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except Exception:
            pass
            
    procs = get_worker_processes(proc, worker_id)
    for p in procs:
        try:
            p.send_signal(signal.SIGINT)
        except Exception:
            pass

def send_sigkill_to_worker(proc, worker_id):
    """Sends SIGKILL to the process group and all child processes."""
    if proc:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass
            
    procs = get_worker_processes(proc, worker_id)
    for p in procs:
        try:
            p.kill()
        except Exception:
            pass

def terminate_process_cleanly(proc, worker_id, timeout=2.0):
    if not proc and worker_id is None:
        return

    send_sigint_to_worker(proc, worker_id)

    t_start = time.time()
    while time.time() - t_start < timeout:
        if proc and proc.poll() is not None:
            # Check if children are also dead
            if not get_worker_processes(None, worker_id):
                return
        time.sleep(0.1)

    send_sigkill_to_worker(proc, worker_id)

def fix_ownership(start_time_str=None):
    """結果ディレクトリの所有権をホストのユーザーに変更する"""
    is_docker = os.path.exists('/.dockerenv')
    try:
        if is_docker:
            # Container side: read host user's UID/GID from /workspace mount
            if os.path.exists('/workspace'):
                stat_info = os.stat('/workspace')
                uid = stat_info.st_uid
                gid = stat_info.st_gid
                
                paths_to_fix = [
                    "/workspace/sim_results",
                    "/workspace/tools/log"
                ]
                for p in paths_to_fix:
                    if os.path.exists(p):
                        subprocess.run(
                            ["chown", "-R", f"{uid}:{gid}", p],
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL,
                            timeout=60
                        )
        else:
            # Host side
            uid = os.getuid()
            gid = os.getgid()
            paths_to_fix = [
                "/workspace/sim_results",
                "/workspace/tools/log"
            ]
            for p in paths_to_fix:
                subprocess.run(
                    ["docker", "compose", "exec", "-T", "sim", "chown", "-R", f"{uid}:{gid}", p],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=60
                )
    except Exception as e:
        print(f"[Sweep Sim] Warning: Failed to fix ownership: {e}")

atexit.register(fix_ownership)

def handle_shutdown(signum, frame):
    global shutdown_requested
    if shutdown_requested:
        return
    shutdown_requested = True
    
    # Log abortion to sweep_progress.log
    try:
        log_progress(f"ABORT ts={datetime.datetime.now().isoformat()}")
    except Exception:
        pass
        
    print("\n\n[Sweep Sim] Interrupt received. Shutting down all simulation tasks cleanly...")
    
    # 全てのアクティブなプロセスを終了
    with active_tasks_lock:
        tasks_to_kill = list(active_tasks.items())
        active_tasks.clear()
        
    is_docker = os.path.exists('/.dockerenv')
    
    # 全プロセスに並列で SIGINT を送信
    for worker_id, task_data in tasks_to_kill:
        proc = task_data.get('proc')
        if proc:
            print(f"[Sweep Sim] Requesting clean shutdown (SIGINT) for Worker {worker_id} (PID {proc.pid})...")
            send_sigint_to_worker(proc, worker_id)

    # デストラクタでのCSV書き込みを待つ
    time.sleep(2.0)

    # 終了していないものを SIGKILL
    for worker_id, task_data in tasks_to_kill:
        proc = task_data.get('proc')
        tmp_config_path = task_data.get('tmp_config_path')
        
        if proc and proc.poll() is None:
            print(f"[Sweep Sim] Killing remaining process group for Worker {worker_id} (PID {proc.pid})...")
            send_sigkill_to_worker(proc, worker_id)
                    
        # 一時設定ファイルの削除
        if tmp_config_path and os.path.exists(tmp_config_path):
            try:
                os.remove(tmp_config_path)
            except Exception:
                pass

    # その他の残存する一時ファイルを削除
    for f in glob.glob("tools/sweep_build/sim_params_tmp_*.yaml"):
        try:
            os.remove(f)
        except Exception:
            pass

    # オリジナル設定ファイルの復元
    if os.path.exists(BACKUP_PATH):
        print(f"[Sweep Sim] Restoring original configuration to {CONFIG_PATH}...")
        try:
            shutil.copy2(BACKUP_PATH, CONFIG_PATH)
        except Exception as e:
            print(f"[Sweep Sim] Error restoring config: {e}")

    # 成果物の所有権をホストユーザーに変更
    if sweep_start_time:
        fix_ownership(sweep_start_time)

    print("[Sweep Sim] Shutdown completed. Exiting.")
    os._exit(1)

def get_worker_processes(proc, worker_id, all_system_procs=None) -> list:
    """Finds all processes associated with a worker, using child-tree and command line matching."""
    procs = []
    # 1. Add subprocess and its local children (handles is_docker=True)
    if proc:
        try:
            parent = psutil.Process(proc.pid)
            procs.append(parent)
            procs.extend(parent.children(recursive=True))
        except Exception:
            pass
            
    # 2. Search system processes for command line keywords (handles is_docker=False)
    cfg_keyword = f"sim_params_tmp_{worker_id}.yaml"
    part_keyword = f"comms_sim_partition_{worker_id}"
    gz_port_keyword = str(11345 + worker_id)
    
    proc_list = all_system_procs if all_system_procs is not None else psutil.process_iter(['pid', 'cmdline', 'environ'])
    
    for p in proc_list:
        try:
            # Skip if already in the list
            if any(x.pid == p.pid for x in procs):
                continue
            
            cmdline = p.info['cmdline']
            if cmdline:
                cmdline_str = " ".join(cmdline)
                if (cfg_keyword in cmdline_str or 
                    part_keyword in cmdline_str or 
                    gz_port_keyword in cmdline_str):
                    procs.append(p)
                    continue
                    
            # Check environment if available
            env = p.info['environ']
            if env:
                if env.get('ROS_DOMAIN_ID') == str(10 + worker_id) or env.get('GZ_PARTITION') == part_keyword:
                    procs.append(p)
                    continue
        except Exception:
            pass
            
    return procs

# =========================================================================
# Completed Tasks Counter and Resource Profiling state
# =========================================================================
completed_tasks_lock = threading.Lock()
completed_tasks_count = 0
first_task_cpu_samples = []
first_task_mem_samples = []
first_task_samples_lock = threading.Lock()
first_task_profiled = False
monitor_running = False

# =========================================================================
# Resource Measurement Cache and Monitor Thread
# =========================================================================
resource_cache_lock = threading.Lock()
cached_dyn_cpu_per_sim = DEFAULT_CPU_PER_SIM
cached_dyn_mem_per_sim = DEFAULT_MEM_PER_SIM_GIB

def increment_completed_tasks():
    global completed_tasks_count, first_task_profiled, cached_dyn_cpu_per_sim, cached_dyn_mem_per_sim
    global first_task_cpu_samples, first_task_mem_samples
    
    with completed_tasks_lock:
        completed_tasks_count += 1
        current_completed = completed_tasks_count

    if current_completed == 1:
        with first_task_samples_lock:
            cpu_samples = list(first_task_cpu_samples)
            mem_samples = list(first_task_mem_samples)
            
        if cpu_samples and mem_samples:
            avg_cpu = sum(cpu_samples) / len(cpu_samples)
            avg_mem_gib = (sum(mem_samples) / len(mem_samples)) / (1024**3)
            peak_cpu = max(cpu_samples)
            peak_mem_gib = max(mem_samples) / (1024**3)
            
            # Enforce safety clips on calculated resource values
            avg_cpu = max(1.0, min(avg_cpu, 4.0))
            avg_mem_gib = max(0.3, min(avg_mem_gib, 3.0))
            
            with resource_cache_lock:
                cached_dyn_cpu_per_sim = avg_cpu
                cached_dyn_mem_per_sim = avg_mem_gib
                
            first_task_profiled = True
            print(f"\n[Resource Profiling] ========================================================")
            print(f"[Resource Profiling] First instance execution profiling completed successfully!")
            print(f"[Resource Profiling] - Samples collected: {len(cpu_samples)}")
            print(f"[Resource Profiling] - Measured CPU Load: {avg_cpu:.2f} cores (Peak: {peak_cpu:.2f} cores)")
            print(f"[Resource Profiling] - Measured Memory: {avg_mem_gib:.2f} GiB (Peak: {peak_mem_gib:.2f} GiB)")
            print(f"[Resource Profiling] Updated dynamic resource parameters to CPU: {avg_cpu:.2f} cores, Mem: {avg_mem_gib:.2f} GiB")
            print(f"[Resource Profiling] ========================================================\n")
        else:
            print(f"\n[Resource Profiling] Warning: No resource samples collected during first execution. Using defaults.\n")

def measure_resources():
    global cached_dyn_cpu_per_sim, cached_dyn_mem_per_sim
    global first_task_cpu_samples, first_task_mem_samples

    # Measure active tasks metrics (CPU/Mem per sim)
    measured_cpu = 0.0
    measured_mem_bytes = 0.0
    measured_count = 0
    
    try:
        all_system_procs = list(psutil.process_iter(['pid', 'cmdline', 'environ']))
    except Exception:
        all_system_procs = []
        
    active_pids = []
    with active_tasks_lock:
        for w_id, task_data in active_tasks.items():
            proc = task_data.get('proc')
            if proc and proc.poll() is None:
                active_pids.append((w_id, proc))
                
    for w_id, proc in active_pids:
        with active_tasks_lock:
            task_data = active_tasks.get(w_id, {})
            ps_procs = task_data.get('ps_procs', [])
        
        # Check / resolve processes
        if not ps_procs:
            procs = get_worker_processes(proc, w_id, all_system_procs)
            if procs:
                for p in procs:
                    try:
                        p.cpu_percent(interval=None)
                    except Exception:
                        pass
                with active_tasks_lock:
                    if w_id in active_tasks:
                        active_tasks[w_id]['ps_procs'] = procs
                ps_procs = procs
        else:
            has_sim_proc = False
            for p in ps_procs:
                try:
                    if p.is_running():
                        name = p.name().lower()
                        if any(x in name for x in ["gz", "python", "comms_sim", "ruby"]):
                            has_sim_proc = True
                            break
                except Exception:
                    pass
            
            if not has_sim_proc:
                procs = get_worker_processes(proc, w_id, all_system_procs)
                if procs:
                    for p in procs:
                        try:
                            p.cpu_percent(interval=None)
                        except Exception:
                            pass
                    with active_tasks_lock:
                        if w_id in active_tasks:
                            active_tasks[w_id]['ps_procs'] = procs
                    ps_procs = procs
            else:
                try:
                    parent_proc = ps_procs[0]
                    current_children = parent_proc.children(recursive=True)
                    for child in current_children:
                        if not any(x.pid == child.pid for x in ps_procs):
                            child.cpu_percent(interval=None)
                            ps_procs.append(child)
                except Exception:
                    pass
                    
        # Calculate metrics for this worker
        w_cpu = 0.0
        w_mem = 0.0
        for p in ps_procs:
            try:
                if p.is_running():
                    w_cpu += p.cpu_percent(interval=None) / 100.0
                    w_mem += p.memory_info().rss
            except Exception:
                pass
        
        if w_cpu > 0.1 and w_mem > 100 * 1024 * 1024:
            measured_cpu += w_cpu
            measured_mem_bytes += w_mem
            measured_count += 1
            
            # If we are in the first task execution, collect samples
            with completed_tasks_lock:
                current_completed = completed_tasks_count
            if current_completed < 1:
                with first_task_samples_lock:
                    first_task_cpu_samples.append(w_cpu)
                    first_task_mem_samples.append(w_mem)

    # Dynamic resource caching (only update if we are not in first execution)
    with completed_tasks_lock:
        current_completed = completed_tasks_count
    
    if current_completed >= 1:
        if measured_count > 0:
            cpu_val = measured_cpu / measured_count
            mem_val = (measured_mem_bytes / measured_count) / (1024**3)
            # Clip safety values
            cpu_val = max(1.0, min(cpu_val, 4.0))
            mem_val = max(0.3, min(mem_val, 3.0))
            with resource_cache_lock:
                cached_dyn_cpu_per_sim = cpu_val
                cached_dyn_mem_per_sim = mem_val

def resource_monitor_loop():
    # Initialize CPU reference
    try:
        psutil.cpu_percent(interval=None)
    except Exception:
        pass

    while not shutdown_requested and monitor_running:
        try:
            measure_resources()
        except Exception:
            pass
        time.sleep(1.0)

def get_optimal_concurrency(max_limit: int = 4, active_count: int = 0, silent: bool = False) -> int:
    """
    【静的計算方式】
    CPUコア数とメモリ容量の両方から最適な並列数を算出する。
    1インスタンスあたりのリソース消費量は初回タスクのプロファイリングで更新される。
    """
    global cached_dyn_cpu_per_sim, cached_dyn_mem_per_sim
    
    try:
        cpu_count = os.cpu_count() or 4
    except Exception:
        cpu_count = 4

    # --- CPU制約 ---
    cpu_val = cached_dyn_cpu_per_sim if cached_dyn_cpu_per_sim > 0.0 else DEFAULT_CPU_PER_SIM
    cpu_val = max(0.5, cpu_val)
    # CPUコアの95%を使用可能とし、1インスタンスあたりのCPU消費で割る
    safe_cpu_count = cpu_count * 0.95
    max_by_cpu = max(1, int(math.floor(safe_cpu_count / cpu_val)))

    # --- メモリ制約 ---
    mem_val = cached_dyn_mem_per_sim if cached_dyn_mem_per_sim > 0.0 else DEFAULT_MEM_PER_SIM_GIB
    mem_val = max(0.5, mem_val)
    mem_per_worker = mem_val * (1024**3)  # bytes
    mem_available = 8 * (1024**3)         # 8 GiB fallback
    
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

    # メモリ全体の80%を安全ラインとする
    safe_mem_available = mem_available * 0.8
    max_by_mem = max(1, int(math.floor(safe_mem_available / mem_per_worker)))

    # CPU制約とメモリ制約の厳しい方を採用し、ユーザー指定上限でキャップ
    optimal = min(max_limit, max_by_cpu, max_by_mem)
    
    if not silent:
        print(f"[Auto-detect] CPU cores: {cpu_count} (safe: {safe_cpu_count:.1f})")
        print(f"[Auto-detect] Per-sim CPU: {cpu_val:.2f} cores → max {max_by_cpu} parallel")
        print(f"[Auto-detect] Mem Available: {mem_available / (1024**3):.2f} GiB (safe: {safe_mem_available / (1024**3):.2f} GiB)")
        print(f"[Auto-detect] Per-sim Mem: {mem_val:.2f} GiB → max {max_by_mem} parallel")
        print(f"[Auto-detect] Optimal Concurrency: {optimal} (CPU-limited={max_by_cpu <= max_by_mem})")
    
    return optimal


progress_lock = threading.Lock()

def log_progress(line: str):
    """進捗ログにタイムスタンプ付きで1行書き込む (スレッドセーフ)"""
    with progress_lock:
        with open(PROGRESS_LOG, 'a', encoding='utf-8') as lf:
            lf.write(line + "\n")
            lf.flush()

def apply_dither(content, dither_x):
    """
    Apply spatial dithering to the vehicle starting pose and waypoints
    to smooth out discrete time-step binning artifacts.
    """
    match = re.search(r'\nvehicles:\s*\n(.*?)(?=\n\w+:|\Z)', content, re.DOTALL)
    if not match:
        return content
    vehicles_block = match.group(1)
    
    def repl_pose(m):
        prefix = m.group(1)
        val = float(m.group(2))
        suffix = m.group(3)
        return f"{prefix}{val + dither_x:.6f}{suffix}"
    
    new_vehicles_block = re.sub(
        r'(pose:\s*\[\s*)([-\d\.]+)(.*?\])',
        repl_pose,
        vehicles_block
    )
    
    def repl_wp(m):
        prefix = m.group(1)
        val = float(m.group(2))
        suffix = m.group(3)
        return f"{prefix}{val + dither_x:.6f}{suffix}"
        
    new_vehicles_block = re.sub(
        r'(-\s*\[\s*)([-\d\.]+)(.*?\])',
        repl_wp,
        new_vehicles_block
    )
    
    start, end = match.span(1)
    return content[:start] + new_vehicles_block + content[end:]

def run_single_task(task_info, worker_id, sweep_start_time, total_runs_tasks, is_docker, rtf=5.0, timeout=120, base_station_yaw_deg=-90.0, max_concurrency=4, num_runs=1):
    if len(task_info) == 5:
        run_idx, y, angle_deg, overall_task_no, local_task_no = task_info
    else:
        run_idx, y, angle_deg, overall_task_no = task_info
        local_task_no = overall_task_no
    
    world_yaw = math.radians(angle_deg) - math.pi
    entity_yaw = math.radians(base_station_yaw_deg)
    antenna_yaw = world_yaw - entity_yaw
 
    y_str = f"{round(y, 2):g}"
    angle_str = f"{angle_deg:g}"
    summary_filename = f"sweep_summary_{sweep_start_time}_run{run_idx}_y{y_str}_a{angle_str}_w{worker_id}.csv"
    tmp_config_path = f"tools/sweep_build/sim_params_tmp_{worker_id}.yaml"
    ros_domain_id = 10 + worker_id
 
    pct = (local_task_no - 1) / total_runs_tasks * 100
    print(f"\n=======================================================")
    print(f"[Worker {worker_id}] [Run {run_idx}/{NUM_RUNS}] Task {overall_task_no}/{total_runs_tasks} ({pct:.1f}%) Y = {y} m, Angle = {angle_deg} deg")
    print(f"=======================================================")
 
    t_expected = estimate_expected_duration(CONFIG_PATH, rtf)
    
    # Use the user-defined timeout. Under high parallelization, actual simulation RTF drops
    # significantly below the target RTF. Terminating via a dynamic expected time is too aggressive.
    task_timeout = timeout

    max_retries = 3
    for attempt in range(max_retries):
        if shutdown_requested:
            return False
        try:
            # Wait until the system is not too busy and launch cooldown has elapsed
            last_wait_log_time = 0
            while True:
                if shutdown_requested:
                    return False
                
                # Check launch cooldown first to prevent feedback lag overshoot
                now_time = time.time()
                global last_launch_time
                with launch_lock:
                    elapsed = now_time - last_launch_time
                
                if elapsed < LAUNCH_COOLDOWN_SEC:
                    time.sleep(1.0)
                    continue
                
                with active_tasks_lock:
                    current_active = len(active_tasks)
                
                current_optimal = get_optimal_concurrency(max_limit=max_concurrency, active_count=current_active, silent=True)
                
                with active_tasks_lock:
                    current_active = len(active_tasks)
                    if current_active < current_optimal:
                        # Reserve slot with placeholder
                        active_tasks[worker_id] = {
                            'proc': None,
                            'tmp_config_path': tmp_config_path,
                            'ros_domain_id': ros_domain_id
                        }
                        # Update launch time immediately to block other workers
                        with launch_lock:
                            last_launch_time = time.time()
                        break
                
                if now_time - last_wait_log_time > 15.0:
                    print(f"[Worker {worker_id}] System is busy (Active tasks: {current_active}/{current_optimal}). Waiting for system load/memory to decrease...")
                    last_wait_log_time = now_time
                time.sleep(2.0)

            with open(CONFIG_PATH, 'r', encoding='utf-8') as f:
                content = f.read()

            # Apply spatial dithering to UGV starting pose/waypoints if running multiple loops
            if num_runs > 1:
                wp_matches = re.findall(r'-\s*\[\s*([-\d\.]+),\s*([-\d\.]+),\s*([-\d\.]+),\s*([\d\.]+)\s*\]', content)
                speed = float(wp_matches[0][3]) if wp_matches else 83.33
                dx = speed * 0.001
                dither_x = (((run_idx - 1) / num_runs) - 0.5) * dx
                content = apply_dither(content, dither_x)
     
            content = re.sub(r'\n\s*summary_filename:\s*["\']?[^"\']*["\']?', '', content)
            content = re.sub(r'\n\s*output_subdir:\s*["\']?[^"\']*["\']?', '', content)
            content = re.sub(r'\n\s*y_position:\s*[-\d\.]+', '', content)
            content = re.sub(r'\n\s*angle_deg:\s*[-\d\.]+', '', content)
            content = re.sub(
                r'(simulation:)', 
                rf'\1\n  summary_filename: "{summary_filename}"\n  output_subdir: "sweep_{sweep_start_time}"\n  y_position: {y}\n  angle_deg: {angle_deg}', 
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
     
            # [-π, π] に正規化して Gazebo/SDF に渡す値を明確にする
            ugv_antenna_yaw = math.atan2(math.sin(world_yaw - math.pi), math.cos(world_yaw - math.pi))
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
            with active_tasks_lock:
                active_tasks.pop(worker_id, None)
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
     
        if shutdown_requested:
            with active_tasks_lock:
                active_tasks.pop(worker_id, None)
            return False

        log_progress(f"RUNNING task={overall_task_no} y={y} angle={angle_deg} ts={datetime.datetime.now().isoformat()}")

        start_time = time.time()
        log_dir = f"tools/log/{sweep_start_time}"
        os.makedirs(log_dir, exist_ok=True)
        out_log_path = os.path.join(log_dir, f"stdout_worker_{worker_id}.log")
        err_log_path = os.path.join(log_dir, f"stderr_worker_{worker_id}.log")
        out_f = open(out_log_path, "w")
        err_f = open(err_log_path, "w")
        proc = subprocess.Popen(
            cmd,
            stdout=out_f,
            stderr=err_f,
            preexec_fn=os.setsid
        )

        with active_tasks_lock:
            if shutdown_requested:
                active_tasks.pop(worker_id, None)
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except Exception:
                    pass
                return False
            active_tasks[worker_id] = {
                'proc': proc,
                'tmp_config_path': tmp_config_path,
                'ros_domain_id': ros_domain_id,
                'ps_procs': []
            }
        
        timed_out = False
        try:
            proc.wait(timeout=task_timeout)
        except subprocess.TimeoutExpired:
            timed_out = True
            print(f"[Worker {worker_id}] Overall Task {overall_task_no}/{total_runs_tasks} Simulation TIMEOUT ({task_timeout}s): Y={y}, Angle={angle_deg}")
        finally:
            out_f.close()
            err_f.close()
            with active_tasks_lock:
                if worker_id in active_tasks:
                    del active_tasks[worker_id]

            terminate_process_cleanly(proc, worker_id)
     
            pass

        actual_duration = time.time() - start_time
        
        is_valid = True
        reason = ""
        
        summary_path = os.path.join(os.getcwd(), "sim_results", f"sweep_{sweep_start_time}", summary_filename)
        
        min_allowed_duration = 3.0
        if t_expected is not None:
            min_allowed_duration = max(3.0, 0.5 * t_expected)

        if timed_out:
            is_valid = False
            reason = "Timeout"
        elif actual_duration < min_allowed_duration:
            is_valid = False
            reason = f"Simulation ended too quickly (took {actual_duration:.1f}s, expected at least {min_allowed_duration:.1f}s)"
        elif proc.returncode != 0 and proc.returncode not in (-2, -15):
            if not os.path.exists(summary_path):
                is_valid = False
                reason = f"Process exited with error code {proc.returncode}"
        else:
            if not os.path.exists(summary_path):
                is_valid = False
                reason = "Summary CSV not found (Simulation crashed or exited silently)"
                
        if is_valid:
            expected_str = f"{t_expected:.1f}s" if t_expected is not None else "Unknown"
            print(f"[Worker {worker_id}] Overall Task {overall_task_no}/{total_runs_tasks} Simulation finished successfully in {actual_duration:.1f}s (Expected: {expected_str}): Y={y}, Angle={angle_deg}")
            log_progress(f"DONE task={overall_task_no} y={y} angle={angle_deg} status=OK ts={datetime.datetime.now().isoformat()}")

            increment_completed_tasks()
            return True
        else:
            if os.path.exists(summary_path):
                try:
                    os.remove(summary_path)
                except Exception:
                    pass
            attempt_info = f"Attempt {attempt + 1}/{max_retries}"
            print(f"[Worker {worker_id}] {attempt_info} FAILED: {reason}. Re-running task with same parameters...")
            time.sleep(2.0)

    print(f"[Worker {worker_id}] Overall Task {overall_task_no}/{total_runs_tasks} All {max_retries} attempts failed: Y={y}, Angle={angle_deg}")
    log_progress(f"DONE task={overall_task_no} y={y} angle={angle_deg} status=FAIL ts={datetime.datetime.now().isoformat()}")
    increment_completed_tasks()
    return False

def main():
    global sweep_start_time
    signal.signal(signal.SIGINT, handle_shutdown)
    signal.signal(signal.SIGTERM, handle_shutdown)
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
    parser.add_argument("--resume", type=str, nargs='?', default=None, const='latest', help="Resume a previous sweep. Without a value, resumes the most recently executed sweep. Optionally specify a timestamp or directory path.")
    parser.add_argument("--no-build", action="store_true", help="Skip automatic colcon build at start")
    parser.add_argument("--manifest", type=str, default=None, help="Path to a JSON manifest file for split sweep execution")
    args = parser.parse_args()

    is_docker = os.path.exists('/.dockerenv')

    if not args.no_build:
        print("[Sweep Sim] Running automatic build (colcon build)...")
        if is_docker:
            build_cmd = ["bash", "-c", "source /opt/ros/humble/setup.bash && cd /workspace && colcon build --symlink-install"]
        else:
            build_cmd = ["docker", "compose", "exec", "-T", "sim", "bash", "-c", "source /opt/ros/humble/setup.bash && cd /workspace && colcon build --symlink-install"]
        
        try:
            subprocess.run(build_cmd, check=True)
            print("[Sweep Sim] Build completed successfully.\n")
        except subprocess.CalledProcessError as e:
            print(f"[Sweep Sim] Build failed. Exiting.")
            sys.exit(1)

    try:
        cpu_count = os.cpu_count() or 4
    except Exception:
        cpu_count = 4
    MAX_CONCURRENCY_CAP = cpu_count
    concurrency_arg = args.concurrency
    if concurrency_arg <= 0:
        # 自動モード: CPU・メモリから最適並列数を算出
        # スレッドプールはMAX_CONCURRENCY_CAPで作成し、ゲートで実際の並列数を制御
        # (プロファイリング後に最適値が上がっても対応可能にするため)
        initial_optimal = get_optimal_concurrency(max_limit=MAX_CONCURRENCY_CAP)
        concurrency = MAX_CONCURRENCY_CAP
        print(f"[Sweep Sim] Auto concurrency: {initial_optimal} parallel (pool size: {MAX_CONCURRENCY_CAP}, will adjust after profiling)")
    else:
        concurrency = concurrency_arg
        print(f"[Sweep Sim] Concurrency manually set to limit: {concurrency}")

    manifest_data = None
    chunk_idx = None
    manifest_path = None
    if args.manifest:
        import json
        manifest_path = os.path.abspath(os.path.expanduser(os.path.expandvars(args.manifest)))
        if not os.path.exists(manifest_path):
            print(f"Error: Manifest file {args.manifest} does not exist.")
            sys.exit(1)
        with open(manifest_path, 'r', encoding='utf-8') as f:
            manifest_data = json.load(f)
        
        sweep_start_time = manifest_data['sweep_id']
        chunk_idx = manifest_data['chunk_idx']
        print(f"[Sweep Sim] Running chunk {chunk_idx} of distributed sweep {sweep_start_time} from manifest.")

    if manifest_data:
        cfg = manifest_data['config']
        num_runs = cfg.get('NUM_RUNS', NUM_RUNS)
        rtf = cfg.get('SWEEP_REAL_TIME_FACTOR', SWEEP_REAL_TIME_FACTOR)
        timeout = cfg.get('TASK_TIMEOUT_SEC', TASK_TIMEOUT_SEC)
        base_station_yaw_deg = cfg.get('RX_YAW_DEG', BASE_STATION_YAW_DEG)
        y_positions = cfg.get('Y_POSITIONS', Y_POSITIONS)
        if 'ANGLES_DEG' in cfg:
            angles_deg = cfg['ANGLES_DEG']
        else:
            start_ang = cfg.get('START_ANGLE', START_ANGLE)
            end_ang = cfg.get('END_ANGLE', END_ANGLE)
            step_ang = cfg.get('STEP_ANGLE', STEP_ANGLE)
            angles_deg = []
            curr_ang = start_ang
            while curr_ang <= end_ang + 1e-5:
                angles_deg.append(round(curr_ang, 1))
                curr_ang += step_ang
    else:
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
        resume_input = os.path.expanduser(os.path.expandvars(args.resume.strip()))
        
        if resume_input == 'latest':
            if manifest_data:
                target_dir = os.path.join("sim_results", f"sweep_{sweep_start_time}")
                if os.path.exists(target_dir):
                    sweep_dir = target_dir
                    print(f"[Sweep Sim] Resuming distributed sweep chunk from manifest-defined directory: {sweep_dir}")
                else:
                    sweep_dirs = sorted(glob.glob("sim_results/sweep_*"), key=os.path.getmtime, reverse=True)
                    if not sweep_dirs:
                        print("Error: No previous sweep directories found in sim_results/.")
                        sys.exit(1)
                    sweep_dir = sweep_dirs[0]
                    sweep_start_time = os.path.basename(sweep_dir).replace("sweep_", "")
                    print(f"[Sweep Sim] Auto-detected most recent sweep (fallback): {sweep_dir}")
            else:
                # 直近に実行されたスイープディレクトリを自動検出
                sweep_dirs = sorted(glob.glob("sim_results/sweep_*"), key=os.path.getmtime, reverse=True)
                if not sweep_dirs:
                    print("Error: No previous sweep directories found in sim_results/.")
                    sys.exit(1)
                sweep_dir = sweep_dirs[0]
                sweep_start_time = os.path.basename(sweep_dir).replace("sweep_", "")
                print(f"[Sweep Sim] Auto-detected most recent sweep: {sweep_dir}")
        elif '/' in resume_input or '\\' in resume_input:
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
        if manifest_data:
            sweep_dir = os.path.join("sim_results", f"sweep_{sweep_start_time}")
        else:
            sweep_start_time = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            sweep_dir = os.path.join("sim_results", f"sweep_{sweep_start_time}")

    global PROGRESS_LOG
    PROGRESS_LOG = f"tools/log/{sweep_start_time}/sweep_progress.log"
    os.makedirs(os.path.dirname(PROGRESS_LOG), exist_ok=True)

    if not args.resume and os.path.exists(PROGRESS_LOG):
        os.remove(PROGRESS_LOG)

    # Copy manifest to results folder if chunk_idx is not None
    if chunk_idx is not None and manifest_path:
        os.makedirs(sweep_dir, exist_ok=True)
        dest_manifest = os.path.join(sweep_dir, f"manifest_chunk_{chunk_idx}.json")
        try:
            shutil.copy2(manifest_path, dest_manifest)
            print(f"[Sweep Sim] Preserved manifest to results directory: {dest_manifest}")
        except Exception as e:
            print(f"[Sweep Sim] Warning: Failed to copy manifest: {e}")

    tasks_list = []
    skipped_count = 0
    
    if manifest_data:
        manifest_tasks = manifest_data.get('tasks', [])
        completed_cache = {}
        for local_idx, task in enumerate(manifest_tasks):
            r_idx = task['run_idx']
            y_val = task['y']
            ang_val = task['angle_deg']
            t_no = task['overall_task_no']
            
            is_completed = False
            if args.resume:
                if r_idx not in completed_cache:
                    completed_cache[r_idx] = get_completed_tasks(sweep_dir, r_idx)
                for cy, cang in completed_cache[r_idx]:
                    if abs(cy - y_val) < 0.01 and abs(cang - ang_val) < 0.05:
                        is_completed = True
                        break
            
            if is_completed:
                skipped_count += 1
            else:
                tasks_list.append((r_idx, y_val, ang_val, t_no, local_idx + 1))
        
        total_runs_tasks = len(manifest_tasks)
    else:
        overall_task_no = 1
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
                        tasks_list.append((run_idx, y, angle_deg, overall_task_no, overall_task_no))
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

    # Start resource monitor thread in background
    global monitor_running
    monitor_running = True
    monitor_thread = threading.Thread(target=resource_monitor_loop, daemon=True)
    monitor_thread.start()

    def worker_thread_fn(task_info):
        worker_id = worker_queue.get()
        try:
            success = run_single_task(task_info, worker_id, sweep_start_time, total_runs_tasks, is_docker, rtf=rtf, timeout=timeout, base_station_yaw_deg=base_station_yaw_deg, max_concurrency=concurrency, num_runs=num_runs)
            return success
        finally:
            worker_queue.put(worker_id)
            time.sleep(0.5)

    try:
        with concurrent.futures.ThreadPoolExecutor(max_workers=concurrency) as executor:
            futures = {executor.submit(worker_thread_fn, t): t for t in tasks_list}
            concurrent.futures.wait(futures.keys())
            for future in futures:
                try:
                    future.result()
                except Exception as e:
                    print(f"Exception in worker thread: {e}")
                    import traceback
                    traceback.print_exc()
    finally:
        monitor_running = False
        # Clean up temporary config files
        for f in glob.glob("tools/sweep_build/sim_params_tmp_*.yaml"):
            try:
                os.remove(f)
            except Exception:
                pass

    fix_ownership(sweep_start_time)

    print("\nMerging worker results...")
    for run_idx in range(1, num_runs + 1):
        if chunk_idx is not None:
            merged_summary_file = os.path.join("sim_results", f"sweep_{sweep_start_time}", f"sweep_summary_run{run_idx}_chunk{chunk_idx}.csv")
        else:
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

        pattern = os.path.join("sim_results", f"sweep_{sweep_start_time}", f"sweep_summary_{sweep_start_time}_run{run_idx}_*_w*.csv")
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

    if chunk_idx is None:
        print("\nAveraging results across all runs...")
        try:
            final_summary_file = f"sim_results/sweep_{sweep_start_time}/sweep_summary.csv"
            summary_files = [os.path.join("sim_results", f"sweep_{sweep_start_time}", f"sweep_summary_run{run_idx}.csv") for run_idx in range(1, num_runs + 1)]
            average_summaries(summary_files, final_summary_file)
            print(f"Averaged summary successfully saved to: {final_summary_file}")
        except Exception as e:
            print(f"Failed to average summaries: {e}")
    else:
        print(f"\nChunk {chunk_idx} execution finished. Results merged into chunk-specific files.")
        print(f"Please copy the directory 'sim_results/sweep_{sweep_start_time}' back to your primary PC and run the merge tool:")
        print(f"  python3 tools/sweep_dist.py merge --sweep-dir sim_results/sweep_{sweep_start_time}")

    # すべてのファイル生成が完了したため、再度所有権を修正
    fix_ownership(sweep_start_time)

    log_progress(f"DONE task={total_runs_tasks} y=- angle=- status=SWEEP_COMPLETE ts={datetime.datetime.now().isoformat()}")
    print("\nSweep completed! Restoring original config...")
    shutil.copy2(BACKUP_PATH, CONFIG_PATH)

if __name__ == "__main__":
    main()
