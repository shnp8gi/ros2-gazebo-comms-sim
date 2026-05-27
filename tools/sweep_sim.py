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

# =========================================================================
# パラメータスイープ設定 (シミュレーションパラメータ)
# =========================================================================
# 基地局のY位置リスト (m)
Y_POSITIONS = [3.0]

# 基地局自体の向き (ヨー角) (deg)
BASE_STATION_YAW_DEG = -90.0

# 角度スイープの設定 (下限, 上限, 刻み幅)
START_ANGLE = 80.0   # 下限 (deg)
END_ANGLE = 100.0    # 上限 (deg)
STEP_ANGLE = 0.5     # 刻み幅 (deg)

# パラメータスイープを繰り返す回数 (ラン数)
NUM_RUNS = 1

# 1タスクあたりの最大待機時間 [秒]
TASK_TIMEOUT_SEC = 120

# スイープ時の加速倍率 (ヘッドレス時のみ有効.1.0=リアルタイム)
SWEEP_REAL_TIME_FACTOR = 3.0

# =========================================================================
# 設定値から自動生成されるパラメータ・内部変数
# =========================================================================
PROGRESS_LOG = "tools/log/sweep_progress.log"

ANGLES_DEG = []
_curr_ang = START_ANGLE
while _curr_ang <= END_ANGLE + 1e-5:
    ANGLES_DEG.append(round(_curr_ang, 1))
    _curr_ang += STEP_ANGLE

def get_optimal_concurrency() -> int:
    """CPUコア数・現在のシステム負荷・および利用可能な空きメモリ容量から最適な並列度を決定する"""
    # 1. CPU制限の計算 (物理コア数および現在のロードアベレージに基づく)
    try:
        cpu_count = os.cpu_count()
    except Exception:
        cpu_count = 4
    if cpu_count is None:
        cpu_count = 4

    # 1分間のロードアベレージを取得して、利用可能な空きコア数を算出する
    load_1min = 0.0
    loadavg_path = "/proc/loadavg"
    if os.path.exists(loadavg_path):
        try:
            with open(loadavg_path, "r") as f:
                load_1min = float(f.read().split()[0])
        except Exception:
            pass

    # 一時的なロードアベレージのスパイクによって並列度が極小化（1並列）するのを防ぐため、
    # ロードアベレージの影響を半分に抑え、最低でも物理コア数の 25% (または2並列) を下限値として確保する。
    cpu_available = max(2.0, cpu_count - 0.5 * load_1min)
    
    min_concurrency = max(2, int(cpu_count * 0.25))
    max_by_cpu = min(
        max(min_concurrency, int(cpu_available * 0.75)),
        max(min_concurrency, int(cpu_count * 0.75))
    )

    # 2. 空きメモリ容量の取得
    cgroup_available = None

    # (a) cgroup v2
    cgroup2_max = "/sys/fs/cgroup/memory.max"
    cgroup2_current = "/sys/fs/cgroup/memory.current"
    if os.path.exists(cgroup2_max) and os.path.exists(cgroup2_current):
        try:
            with open(cgroup2_max, "r") as f:
                limit_val = f.read().strip()
            with open(cgroup2_current, "r") as f:
                current_val = f.read().strip()
            if limit_val != "max":
                cgroup_available = int(limit_val) - int(current_val)
        except Exception:
            pass

    # (b) cgroup v1 (フォールバック)
    if cgroup_available is None:
        cgroup1_limit = "/sys/fs/cgroup/memory/memory.limit_in_bytes"
        cgroup1_usage = "/sys/fs/cgroup/memory/memory.usage_in_bytes"
        if os.path.exists(cgroup1_limit) and os.path.exists(cgroup1_usage):
            try:
                with open(cgroup1_limit, "r") as f:
                    limit_val = int(f.read().strip())
                with open(cgroup1_usage, "r") as f:
                    usage_val = int(f.read().strip())
                if limit_val < 9223372036854771712:
                    cgroup_available = limit_val - usage_val
            except Exception:
                pass

    # (c) /proc/meminfo から MemAvailable を取得 (システム全体の空きメモリ)
    system_available = None
    meminfo_path = "/proc/meminfo"
    if os.path.exists(meminfo_path):
        try:
            with open(meminfo_path, "r") as f:
                for line in f:
                    if line.startswith("MemAvailable:"):
                        parts = line.split()
                        if len(parts) >= 2:
                            system_available = int(parts[1]) * 1024  # KiB to bytes
                        break
        except Exception:
            pass

        # MemAvailable が見つからない場合は MemFree + Buffers + Cached でフォールバック
        if system_available is None:
            try:
                mem_free = 0
                buffers = 0
                cached = 0
                with open(meminfo_path, "r") as f:
                    for line in f:
                        if line.startswith("MemFree:"):
                            mem_free = int(line.split()[1]) * 1024
                        elif line.startswith("Buffers:"):
                            buffers = int(line.split()[1]) * 1024
                        elif line.startswith("Cached:"):
                            cached = int(line.split()[1]) * 1024
                system_available = mem_free + buffers + cached
            except Exception:
                pass

    # cgroup制限とシステム全体の空きメモリの最小値を「利用可能な空きメモリ」として採用
    mem_available = None
    if cgroup_available is not None and system_available is not None:
        mem_available = min(cgroup_available, system_available)
    elif cgroup_available is not None:
        mem_available = cgroup_available
    elif system_available is not None:
        mem_available = system_available
    else:
        # (d) 最終フォールバック (総メモリの 50% を空きと仮定)
        try:
            total = os.sysconf('SC_PAGE_SIZE') * os.sysconf('SC_PHYS_PAGES')
            mem_available = total // 2
        except Exception:
            mem_available = 4 * 1024 * 1024 * 1024  # 4 GiB fallback

    # 3. メモリに基づく最大並列度の計算
    safety_margin = 2.0 * 1024 * 1024 * 1024  # 2.0 GiB
    instance_memory = 700 * 1024 * 1024       # 700 MiB

    max_by_mem = int((mem_available - safety_margin) / instance_memory)
    max_by_mem = max(1, max_by_mem)

    # 4. 最適な並列度の決定
    optimal = min(max_by_cpu, max_by_mem)
    
    print(f"[Auto-detect] CPU Count: {cpu_count}, Load 1min: {load_1min:.2f} -> Max by CPU: {max_by_cpu}")
    print(f"[Auto-detect] Available Memory: {mem_available / (1024**3):.2f} GiB -> Max by Mem: {max_by_mem}")
    print(f"[Auto-detect] Optimal Concurrency: {optimal}")
    
    return optimal

progress_lock = threading.Lock()

def log_progress(line: str):
    """進捗ログにタイムスタンプ付きで1行書き込む (スレッドセーフ)"""
    with progress_lock:
        with open(PROGRESS_LOG, 'a', encoding='utf-8') as lf:
            lf.write(line + "\n")
            lf.flush()

def launch_progress_monitor():
    """sweep_progress.py --watch を別ターミナル（または別プロセス）で自動起動する。
    優先順位:
      1. tmux セッション内であれば新規ウィンドウで起動
      2. gnome-terminal / xterm が使えれば新規ウィンドウで起動
      3. どちらも使えない場合はバックグラウンドサブプロセスとして起動
    """
    script_dir = os.path.dirname(os.path.abspath(__file__))
    progress_script = os.path.join(script_dir, "sweep_progress.py")
    if not os.path.exists(progress_script):
        print("[Progress Monitor] sweep_progress.py が見つからないためスキップします。")
        return

    cmd_str = f"python3 {progress_script} --watch"

    # --- 1. tmux セッション内かどうか確認 ---
    if os.environ.get("TMUX"):
        try:
            subprocess.Popen(
                ["tmux", "new-window", "-n", "sweep-progress", cmd_str]
            )
            print("[Progress Monitor] tmux の新規ウィンドウで sweep_progress.py を起動しました。")
            print("                  (Ctrl+B → 数字キーでウィンドウ切替)")
            return
        except Exception as e:
            print(f"[Progress Monitor] tmux 起動失敗: {e}")

    # --- 2. gnome-terminal ---
    if shutil.which("gnome-terminal"):
        try:
            subprocess.Popen(
                ["gnome-terminal", "--", "bash", "-c", f"{cmd_str}; exec bash"]
            )
            print("[Progress Monitor] gnome-terminal で sweep_progress.py を起動しました。")
            return
        except Exception as e:
            print(f"[Progress Monitor] gnome-terminal 起動失敗: {e}")

    # --- 3. xterm ---
    if shutil.which("xterm"):
        try:
            subprocess.Popen(
                ["xterm", "-title", "Sweep Progress", "-e", cmd_str]
            )
            print("[Progress Monitor] xterm で sweep_progress.py を起動しました。")
            return
        except Exception as e:
            print(f"[Progress Monitor] xterm 起動失敗: {e}")

    # --- 4. バックグラウンドサブプロセス（フォールバック） ---
    try:
        subprocess.Popen(
            ["python3", progress_script, "--watch"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        print("[Progress Monitor] バックグラウンドで sweep_progress.py を起動しました。")
        print("                  (別ターミナルで 'python3 tools/sweep_progress.py --watch' を実行すると進捗が見えます)")
    except Exception as e:
        print(f"[Progress Monitor] 起動失敗: {e}")
        print("                  手動で 'python3 tools/sweep_progress.py --watch' を別ターミナルで実行してください。")



CONFIG_PATH = "src/comms_sim_pkg/config/sim_params.yaml"
BACKUP_PATH = "tools/sweep_build/sim_params.yaml.bak"

def average_summaries(summary_files, output_file):
    """各ランのCSVファイルを読み込んで平均値を集計・保存する"""
    is_docker = os.path.exists('/.dockerenv')
    if not is_docker:
        import subprocess
        ws_summary_files = [f"/workspace/{f}" if not f.startswith('/') else f for f in summary_files]
        ws_output_file = f"/workspace/{output_file}" if not output_file.startswith('/') else output_file
        
        script = f"""
import sys
import pandas as pd
import os

summary_files = {ws_summary_files}
output_file = "{ws_output_file}"

dfs = []
for f in summary_files:
    if os.path.exists(f):
        try:
            dfs.append(pd.read_csv(f))
        except Exception as e:
            print(f"Warning: Failed to read {{f}}: {{e}}")
if not dfs:
    print("No successful sweep summary files found to average.")
    sys.exit(0)

combined = pd.concat(dfs, ignore_index=True)
agg_rules = {{
    'total_data_MB': 'mean',
    'connected_time_s': 'mean',
    'average_throughput_Gbps': 'mean',
    'average_rssi_dBm': 'mean',
    'handover_count': 'mean'
}}
averaged = combined.groupby(['y_position', 'antenna_angle', 'vehicle_name'], as_index=False).agg(agg_rules)
averaged['total_data_MB'] = averaged['total_data_MB'].round(3)
averaged['connected_time_s'] = averaged['connected_time_s'].round(3)
averaged['average_throughput_Gbps'] = averaged['average_throughput_Gbps'].round(3)
averaged['average_rssi_dBm'] = averaged['average_rssi_dBm'].round(3)
averaged['handover_count'] = averaged['handover_count'].round(1)
averaged.insert(0, 'run_id', 'AVERAGE')
averaged = averaged.sort_values(by=['y_position', 'antenna_angle', 'vehicle_name'])
os.makedirs(os.path.dirname(output_file), exist_ok=True)
averaged.to_csv(output_file, index=False)
"""
        cmd = ["docker", "compose", "exec", "-T", "sim", "python3", "-c", script]
        try:
            subprocess.run(cmd, check=True)
        except Exception:
            cmd = ["docker", "exec", "comms_sim", "python3", "-c", script]
            subprocess.run(cmd, check=True)
        return

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

def run_single_task(task_info, worker_id, sweep_start_time, total_runs_tasks, is_docker, rtf=5.0, timeout=120, base_station_yaw_deg=-90.0):
    run_idx, y, angle_deg, overall_task_no = task_info
    
    # ユーザー定義の角度変換:
    #   angle_deg=0   → -X方向 (world_yaw=-pi)  ← 西
    #   angle_deg=90  → -Y方向 (world_yaw=-pi/2) ← 南
    #   angle_deg=180 → +X方向 (world_yaw=0)     ← 東
    world_yaw = math.radians(angle_deg) - math.pi
    
    # 基地局エンティティの向き (ヨー角)
    entity_yaw = math.radians(base_station_yaw_deg)
    antenna_yaw = world_yaw - entity_yaw
 
    summary_filename = f"sweep_summary_{sweep_start_time}_run{run_idx}_w{worker_id}.csv"
    tmp_config_path = f"tools/sweep_build/sim_params_tmp_{worker_id}.yaml"
 
    pct = (overall_task_no - 1) / total_runs_tasks * 100
    print(f"\n=======================================================")
    print(f"[Worker {worker_id}] [Run {run_idx}/{NUM_RUNS}] Task {overall_task_no}/{total_runs_tasks} ({pct:.1f}%) Y = {y} m, Angle = {angle_deg} deg")
    print(f"=======================================================")
    log_progress(f"RUNNING task={overall_task_no} y={y} angle={angle_deg} ts={datetime.datetime.now().isoformat()}")
 
    # YAMLファイルを文字列として読み込み、一時設定ファイルを生成
    try:
        with open(CONFIG_PATH, 'r', encoding='utf-8') as f:
            content = f.read()
 
        # summary_filenameとoutput_subdirの更新
        content = re.sub(r'\n\s*summary_filename:\s*["\']?[^"\']*["\']?', '', content)
        content = re.sub(r'\n\s*output_subdir:\s*["\']?[^"\']*["\']?', '', content)
        content = re.sub(
            r'(simulation:)', 
            rf'\1\n  summary_filename: "{summary_filename}"\n  output_subdir: "sweep_{sweep_start_time}"', 
            content
        )
 
        # headless: false -> true
        content = re.sub(r'headless:\s*false', 'headless: true', content)
 
        # real_time_factor を高速化
        content = re.sub(r'real_time_factor:\s*[\d\.]+', f'real_time_factor: {rtf}', content)
 
        # 基地局Y座標と向きの更新 (antenna_0 / antenna_1 / antenna_2 の pose)
        # pose: [x, y, z, roll, pitch, yaw]
        content = re.sub(
            r'(antenna\w*:\s*.*?pose:\s*\[\s*)([-\d\.]+),\s*[-\d\.]+,\s*([-\d\.]+),\s*([-\d\.]+),\s*([-\d\.]+),\s*([-\d\.]+)(\s*\])',
            rf'\g<1>\g<2>, {y}, \g<3>, \g<4>, \g<5>, {entity_yaw:.4f}\g<7>',
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
        # UGVアンテナは、基地局に対向（対面）させるため、
        # 基地局アンテナの方向 (world_yaw) と逆方向 (world_yaw - pi) を向かせる。
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

    # コマンドの構築（ROS_DOMAIN_ID, GZ_PARTITION, GZ_PORT の分離）
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

    proc = subprocess.Popen(
        cmd,
        preexec_fn=os.setsid
    )
    
    timed_out = False
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        timed_out = True
        print(f"[Worker {worker_id}] Overall Task {overall_task_no}/{total_runs_tasks} Simulation TIMEOUT ({timeout}s): Y={y}, Angle={angle_deg}")
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

def get_completed_tasks(sweep_dir, run_idx):
    import csv
    import glob
    completed = set()
    
    # 1. 未マージのワーカー個別 CSV から読み込み
    pattern = os.path.join(sweep_dir, f"sweep_summary_*_run{run_idx}_w*.csv")
    for csv_file in glob.glob(pattern):
        try:
            with open(csv_file, 'r', encoding='utf-8') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    try:
                        y_val = float(row['y_position'])
                        ang_val = float(row['antenna_angle'])
                        completed.add((y_val, ang_val))
                    except (ValueError, KeyError):
                        continue
        except Exception:
            continue
            
    # 2. すでにマージ済みの CSV があればそこからも読み込み
    merged_file = os.path.join(sweep_dir, f"sweep_summary_run{run_idx}.csv")
    if os.path.exists(merged_file):
        try:
            with open(merged_file, 'r', encoding='utf-8') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    try:
                        y_val = float(row['y_position'])
                        ang_val = float(row['antenna_angle'])
                        completed.add((y_val, ang_val))
                    except (ValueError, KeyError):
                        continue
        except Exception:
            pass
            
    return completed

def main():
    parser = argparse.ArgumentParser(description="Parallel Parameter Sweep Simulation")
    parser.add_argument("-j", "--concurrency", type=int, default=0, help="Number of parallel workers (0 for auto)")
    parser.add_argument("--start-angle", type=float, default=None, help=f"Start angle of sweep in degrees (default from script: {min(ANGLES_DEG)})")
    parser.add_argument("--end-angle", type=float, default=None, help=f"End angle of sweep in degrees (default from script: {max(ANGLES_DEG)})")
    parser.add_argument("--step-angle", type=float, default=None, help="Angle step in degrees (default from script: 0.2)")
    parser.add_argument("--num-runs", type=int, default=None, help=f"Number of runs per task (default from script: {NUM_RUNS})")
    parser.add_argument("--y-positions", type=str, default=None, help=f"Comma-separated Y positions of base stations (default from script: '{','.join(map(str, Y_POSITIONS))}')")
    parser.add_argument("--base-station-yaw", type=float, default=None, help=f"Yaw angle of base station entity in degrees (default from script: {BASE_STATION_YAW_DEG})")
    parser.add_argument("--rtf", "--real-time-factor", type=float, default=None, help=f"Acceleration factor (default from script: {SWEEP_REAL_TIME_FACTOR})")
    parser.add_argument("--timeout", type=int, default=None, help=f"Timeout in seconds per task (default from script: {TASK_TIMEOUT_SEC})")
    parser.add_argument("--resume", type=str, default=None, help="Resume a previous sweep using its timestamp or directory path")
    
    args, unknown = parser.parse_known_args()
    concurrency = args.concurrency
    if concurrency <= 0:
        concurrency = get_optimal_concurrency()

    # Extract parameters from arguments or fallback to global variables
    num_runs = args.num_runs if args.num_runs is not None else NUM_RUNS
    rtf = args.rtf if args.rtf is not None else SWEEP_REAL_TIME_FACTOR
    timeout = args.timeout if args.timeout is not None else TASK_TIMEOUT_SEC
    base_station_yaw_deg = args.base_station_yaw if args.base_station_yaw is not None else BASE_STATION_YAW_DEG

    if args.y_positions is not None:
        y_positions = [float(y.strip()) for y in args.y_positions.split(',') if y.strip()]
    else:
        y_positions = Y_POSITIONS
        
    # Generate angles range or fallback to global variables
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

    # tools/sweep_build ディレクトリの作成
    os.makedirs("tools/sweep_build", exist_ok=True)

    # 常に実行時の最新設定ファイルをバックアップする
    if os.path.exists(BACKUP_PATH):
        os.remove(BACKUP_PATH)
    shutil.copy2(CONFIG_PATH, BACKUP_PATH)

    # クリーンアップ
    import glob
    for f in glob.glob("tools/sweep_build/sim_params_tmp_*.yaml"):
        try:
            os.remove(f)
        except Exception as e:
            print(f"Warning: Failed to clean up leftover config file {f}: {e}")

    # タイムスタンプおよび出力ディレクトリの決定
    if args.resume:
        resume_input = args.resume.strip()
        import sys
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

    # 進捗ログの保存先を実行時のタイムスタンプサブディレクトリ配下に動的決定
    global PROGRESS_LOG
    PROGRESS_LOG = f"tools/log/{sweep_start_time}/sweep_progress.log"
    os.makedirs(os.path.dirname(PROGRESS_LOG), exist_ok=True)

    # 新規実行の場合は既存ログを削除、レジュームの場合は追記
    if not args.resume and os.path.exists(PROGRESS_LOG):
        os.remove(PROGRESS_LOG)

    # タスクリストの作成と既完了タスクの除外
    tasks_list = []
    overall_task_no = 1
    skipped_count = 0
    
    for run_idx in range(1, num_runs + 1):
        completed_set = set()
        if args.resume:
            completed_set = get_completed_tasks(sweep_dir, run_idx)
            
        for y in y_positions:
            for angle_deg in angles_deg:
                # 既完了か判定
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

    # 進捗監視スクリプトを別ターミナル（または別プロセス）で自動起動
    launch_progress_monitor()

    print(f"Starting parameter sweep: Y_POSITIONS={y_positions}, ANGLES_DEG={angles_deg}")
    print(f"Base Station Yaw: {base_station_yaw_deg} deg")
    print(f"Number of runs per task: {num_runs}")
    print(f"Total tasks across all runs: {total_runs_tasks}")
    if skipped_count > 0:
        print(f"Resuming: Skipped {skipped_count} already completed tasks. {len(tasks_list)} tasks remaining.")
    print(f"Concurrency level: {concurrency}")

    # ワーカーID管理キュー
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
            time.sleep(0.5)  # 各シミュレーション終了後のクールダウン

    # スレッドプールで並列実行
    with concurrent.futures.ThreadPoolExecutor(max_workers=concurrency) as executor:
        futures = {executor.submit(worker_thread_fn, t): t for t in tasks_list}
        concurrent.futures.wait(futures.keys())

    # ---------------------------------------------------------
    # 各ワーカーが書き出した CSV ファイルを run_idx ごとに結合する
    # ---------------------------------------------------------
    print("\nMerging worker results...")
    for run_idx in range(1, num_runs + 1):
        merged_summary_file = os.path.join("sim_results", f"sweep_{sweep_start_time}", f"sweep_summary_run{run_idx}.csv")
        os.makedirs(os.path.dirname(merged_summary_file), exist_ok=True)
        
        # 既存のマージファイルがある場合は行を読み込む
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

        # ワイルドカードでその run_idx に対するすべてのワーカーCSVファイルを探す
        import glob
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

    # 平均化の処理を実行
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
