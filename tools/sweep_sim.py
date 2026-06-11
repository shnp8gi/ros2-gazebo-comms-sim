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
CPU_SAFE_RATIO = 0.95  # [廃止] 旧ロジックで使用。新ロジックではreserved_cpus (10%)を直接使用
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

def send_sigint_to_worker(proc, ros_domain_id, is_docker):
    """Sends SIGINT to the process group and ROS nodes matching the domain ID."""
    if not proc:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except Exception:
        pass

    if is_docker:
        try:
            subprocess.run(
                f"grep -l 'ROS_DOMAIN_ID={ros_domain_id}' /proc/[0-9]*/environ 2>/dev/null | cut -d '/' -f 3 | xargs -r kill -2",
                shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=2
            )
        except Exception:
            pass
    else:
        try:
            subprocess.run(
                ["docker", "compose", "exec", "-T", "sim", "sh", "-c", f"grep -l 'ROS_DOMAIN_ID={ros_domain_id}' /proc/[0-9]*/environ 2>/dev/null | cut -d '/' -f 3 | xargs -r kill -2"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=2
            )
        except Exception:
            pass

def send_sigkill_to_worker(proc, ros_domain_id, is_docker):
    """Sends SIGKILL to the process group and ROS nodes matching the domain ID."""
    if not proc:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except Exception:
        pass

    if is_docker:
        try:
            subprocess.run(
                f"grep -l 'ROS_DOMAIN_ID={ros_domain_id}' /proc/[0-9]*/environ 2>/dev/null | cut -d '/' -f 3 | xargs -r kill -9",
                shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=2
            )
        except Exception:
            pass
    else:
        try:
            subprocess.run(
                ["docker", "compose", "exec", "-T", "sim", "sh", "-c", f"grep -l 'ROS_DOMAIN_ID={ros_domain_id}' /proc/[0-9]*/environ 2>/dev/null | cut -d '/' -f 3 | xargs -r kill -9"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=2
            )
        except Exception:
            pass

def terminate_process_cleanly(proc, ros_domain_id, is_docker, timeout=2.0):
    if not proc:
        return

    send_sigint_to_worker(proc, ros_domain_id, is_docker)

    t_start = time.time()
    while time.time() - t_start < timeout:
        if proc.poll() is not None:
            return
        time.sleep(0.1)

    if proc.poll() is None:
        send_sigkill_to_worker(proc, ros_domain_id, is_docker)

def fix_ownership(start_time_str):
    """結果ディレクトリの所有権をホストのユーザーに変更する"""
    if not start_time_str:
        return
    is_docker = os.path.exists('/.dockerenv')
    try:
        if is_docker:
            # Container side: read host user's UID/GID from /workspace mount
            if os.path.exists('/workspace'):
                stat_info = os.stat('/workspace')
                uid = stat_info.st_uid
                gid = stat_info.st_gid
                
                paths_to_fix = [
                    f"/workspace/sim_results/sweep_{start_time_str}",
                    f"/workspace/tools/log/{start_time_str}"
                ]
                for p in paths_to_fix:
                    if os.path.exists(p):
                        subprocess.run(
                            ["chown", "-R", f"{uid}:{gid}", p],
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL,
                            timeout=10
                        )
        else:
            # Host side
            uid = os.getuid()
            gid = os.getgid()
            paths_to_fix = [
                f"/workspace/sim_results/sweep_{start_time_str}",
                f"/workspace/tools/log/{start_time_str}"
            ]
            for p in paths_to_fix:
                subprocess.run(
                    ["docker", "compose", "exec", "-T", "sim", "chown", "-R", f"{uid}:{gid}", p],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=10
                )
    except Exception as e:
        print(f"[Sweep Sim] Warning: Failed to fix ownership: {e}")

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
        ros_domain_id = task_data.get('ros_domain_id')
        if proc:
            print(f"[Sweep Sim] Requesting clean shutdown (SIGINT) for Worker {worker_id} (PID {proc.pid})...")
            send_sigint_to_worker(proc, ros_domain_id, is_docker)

    # デストラクタでのCSV書き込みを待つ
    time.sleep(2.0)

    # 終了していないものを SIGKILL
    for worker_id, task_data in tasks_to_kill:
        proc = task_data.get('proc')
        ros_domain_id = task_data.get('ros_domain_id')
        tmp_config_path = task_data.get('tmp_config_path')
        
        if proc and proc.poll() is None:
            print(f"[Sweep Sim] Killing remaining process group for Worker {worker_id} (PID {proc.pid})...")
            send_sigkill_to_worker(proc, ros_domain_id, is_docker)
                    
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
cached_system_load = 0.0
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
    global cached_system_load, cached_dyn_cpu_per_sim, cached_dyn_mem_per_sim
    global first_task_cpu_samples, first_task_mem_samples
    
    try:
        cpu_count = os.cpu_count() or 4
    except Exception:
        cpu_count = 4

    # 1. Measure system CPU load
    system_load = 0.0
    if psutil:
        try:
            sys_cpu_pct = psutil.cpu_percent(interval=None)
            system_load = (sys_cpu_pct / 100.0) * cpu_count
        except Exception:
            pass
    if system_load == 0.0:
        loadavg_path = "/proc/loadavg"
        if os.path.exists(loadavg_path):
            try:
                with open(loadavg_path, "r") as f:
                    system_load = float(f.read().split()[0])
            except Exception:
                pass
    
    with resource_cache_lock:
        cached_system_load = system_load

    # 2. Measure active tasks metrics (CPU/Mem per sim)
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
    
    while not shutdown_requested:
        try:
            measure_resources()
        except Exception:
            pass
        time.sleep(1.0)

def get_optimal_concurrency(max_limit: int = 4, active_count: int = 0, silent: bool = False) -> int:
    """CPUコア数・現在のシステム負荷・および利用可能な空きメモリ容量から最適な並列度を決定する

    旧ロジックでは external_load = system_load - our_sims_load として外部負荷を推定していたが、
    CPUコンテンション下では各シミュレーションのCPU計測値が低下し our_sims_load が過大推定され、
    external_load が 0 になる循環依存バグがあった。
    新ロジックでは「システム全体のアイドルCPUコア数」を直接使い、追加でRTFフィードバック制御を行う。
    """
    global cached_system_load, cached_dyn_cpu_per_sim, cached_dyn_mem_per_sim

    try:
        cpu_count = os.cpu_count()
    except Exception:
        cpu_count = 4
    if cpu_count is None:
        cpu_count = 4

    load_1min = cached_system_load
    dyn_cpu_per_sim = cached_dyn_cpu_per_sim
    dyn_mem_per_sim = cached_dyn_mem_per_sim

    # ==========================================
    # 方式1: アイドルCPUコアから直接計算 (メイン)
    # ==========================================
    # システム全体のアイドルCPUコア数 = 全コア × (1 - 使用率)
    idle_cpus = max(0.0, cpu_count - load_1min)

    # 自分のシミュレーションが使っている分を「空きに戻す」
    # (自分のワーカーを減らせばその分空くため)
    our_sims_load = active_count * dyn_cpu_per_sim
    potential_avail = idle_cpus + our_sims_load

    # 安全マージン: CPU全体の10%は常に確保 (他ユーザー・OS用)
    reserved_cpus = cpu_count * 0.10
    avail_for_sims = max(0.0, potential_avail - reserved_cpus)
    max_by_cpu = max(1, int(math.floor(avail_for_sims / dyn_cpu_per_sim)))

    # ==========================================
    # 方式2: システム全体の使用率による絶対上限
    # ==========================================
    # CPUが85%以上使用中なら、現在のアクティブ数以上には増やさない
    system_cpu_pct = (load_1min / cpu_count) * 100.0 if cpu_count > 0 else 100.0
    if system_cpu_pct > 85.0 and active_count > 0:
        max_by_cpu = min(max_by_cpu, active_count)
    # CPUが95%以上なら、現在のアクティブ数から1つ減らす
    if system_cpu_pct > 95.0 and active_count > 1:
        max_by_cpu = min(max_by_cpu, active_count - 1)

    # ==========================================
    # 方式3: RTFフィードバック制御
    # ==========================================
    # タスクの実行時間が想定の2倍以上 → コンテンションの兆候 → 並列数を制限
    rtf_penalty = 1.0
    with completed_tasks_lock:
        current_completed = completed_tasks_count
    if current_completed >= 3:
        try:
            with first_task_samples_lock:
                pass  # ロック取得確認のみ
            # task_duration_history からRTF劣化を検出
            recent_durations = getattr(get_optimal_concurrency, '_recent_durations', [])
            if recent_durations:
                baseline = getattr(get_optimal_concurrency, '_baseline_duration', None)
                if baseline and baseline > 0:
                    avg_recent = sum(recent_durations[-5:]) / len(recent_durations[-5:])
                    ratio = avg_recent / baseline
                    if ratio > 2.0:
                        rtf_penalty = 0.5  # 半分に制限
                    elif ratio > 1.5:
                        rtf_penalty = 0.7  # 30%削減
        except Exception:
            pass

    if rtf_penalty < 1.0:
        max_by_cpu = max(1, int(max_by_cpu * rtf_penalty))

    # ==========================================
    # メモリ制限
    # ==========================================
    mem_per_worker = dyn_mem_per_sim * (1024**3)  # bytes
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
    optimal = max(1, min(optimal, max_limit))
    
    # Force concurrency to 1 during the first task execution for load measurement
    if current_completed < 1:
        optimal = 1
    
    if not silent:
        print(f"[Auto-detect] CPU Count: {cpu_count}, System Load: {load_1min:.2f}/{cpu_count} cores ({system_cpu_pct:.1f}%), Idle: {idle_cpus:.2f} cores")
        print(f"[Auto-detect] Estimated Resource per Sim -> CPU: {dyn_cpu_per_sim:.2f} cores, Mem: {dyn_mem_per_sim:.2f} GiB")
        print(f"[Auto-detect] Available for sims (after 10% reserve): {avail_for_sims:.2f} cores -> Max by CPU: {max_by_cpu}, Max by Mem: {max_by_mem}")
        if rtf_penalty < 1.0:
            print(f"[Auto-detect] RTF degradation detected! Penalty factor: {rtf_penalty:.1f}")
        if current_completed < 1:
            print(f"[Auto-detect] Profiling phase active (completed: {current_completed}/1). Concurrency forced to 1.")
        print(f"[Auto-detect] Optimal Concurrency (capped at {max_limit}): {optimal}")
    
    return optimal


progress_lock = threading.Lock()

def log_progress(line: str):
    """進捗ログにタイムスタンプ付きで1行書き込む (スレッドセーフ)"""
    with progress_lock:
        with open(PROGRESS_LOG, 'a', encoding='utf-8') as lf:
            lf.write(line + "\n")
            lf.flush()

def run_single_task(task_info, worker_id, sweep_start_time, total_runs_tasks, is_docker, rtf=5.0, timeout=120, base_station_yaw_deg=-90.0, max_concurrency=4):
    if len(task_info) == 5:
        run_idx, y, angle_deg, overall_task_no, local_task_no = task_info
    else:
        run_idx, y, angle_deg, overall_task_no = task_info
        local_task_no = overall_task_no
    
    world_yaw = math.radians(angle_deg) - math.pi
    entity_yaw = math.radians(base_station_yaw_deg)
    antenna_yaw = world_yaw - entity_yaw
 
    summary_filename = f"sweep_summary_{sweep_start_time}_run{run_idx}_w{worker_id}.csv"
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
     
        if shutdown_requested:
            return False

        log_progress(f"RUNNING task={overall_task_no} y={y} angle={angle_deg} ts={datetime.datetime.now().isoformat()}")

        start_time = time.time()
        proc = subprocess.Popen(
            cmd,
            preexec_fn=os.setsid
        )

        with active_tasks_lock:
            if shutdown_requested:
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
            with active_tasks_lock:
                if worker_id in active_tasks:
                    del active_tasks[worker_id]

            terminate_process_cleanly(proc, ros_domain_id, is_docker)
     
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

            # RTFフィードバック用: タスク実行時間を記録
            if not hasattr(get_optimal_concurrency, '_recent_durations'):
                get_optimal_concurrency._recent_durations = []
            get_optimal_concurrency._recent_durations.append(actual_duration)
            # 最初の成功タスクの期待時間をベースラインとして記録
            if t_expected is not None and not hasattr(get_optimal_concurrency, '_baseline_duration'):
                get_optimal_concurrency._baseline_duration = t_expected

            increment_completed_tasks()
            return True
        else:
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
    parser.add_argument("--resume", type=str, default=None, help="Resume a previous sweep using its timestamp or directory path")
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
        # In auto mode, we set the pool capacity to MAX_CONCURRENCY_CAP,
        # but check and print the initial optimal concurrency level.
        initial_optimal = get_optimal_concurrency(max_limit=MAX_CONCURRENCY_CAP)
        concurrency = MAX_CONCURRENCY_CAP
        print(f"[Sweep Sim] Dynamic concurrency enabled (Initial optimal: {initial_optimal}, Limit: {MAX_CONCURRENCY_CAP})")
    else:
        concurrency = concurrency_arg
        print(f"[Sweep Sim] Concurrency manually set to limit: {concurrency}")

    manifest_data = None
    chunk_idx = None
    if args.manifest:
        import json
        if not os.path.exists(args.manifest):
            print(f"Error: Manifest file {args.manifest} does not exist.")
            sys.exit(1)
        with open(args.manifest, 'r', encoding='utf-8') as f:
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
    if chunk_idx is not None and args.manifest:
        os.makedirs(sweep_dir, exist_ok=True)
        dest_manifest = os.path.join(sweep_dir, f"manifest_chunk_{chunk_idx}.json")
        try:
            shutil.copy2(args.manifest, dest_manifest)
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
            success = run_single_task(task_info, worker_id, sweep_start_time, total_runs_tasks, is_docker, rtf=rtf, timeout=timeout, base_station_yaw_deg=base_station_yaw_deg, max_concurrency=concurrency)
            return success
        finally:
            worker_queue.put(worker_id)
            time.sleep(0.5)

    try:
        with concurrent.futures.ThreadPoolExecutor(max_workers=concurrency) as executor:
            futures = {executor.submit(worker_thread_fn, t): t for t in tasks_list}
            concurrent.futures.wait(futures.keys())
    finally:
        monitor_running = False

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
