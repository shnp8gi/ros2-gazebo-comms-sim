#!/usr/bin/env python3
import subprocess
import signal
import os
import math
import time
import shutil
import re
import datetime

PROGRESS_LOG = "sweep/log/sweep_progress.log"
# 1タスクあたりの最大待機時間 [秒]
TASK_TIMEOUT_SEC = 120
# スイープ時の加速倍率 (ヘッドレス時のみ有効。1.0=リアルタイム)
SWEEP_REAL_TIME_FACTOR = 5.0
# パラメータスイープを繰り返す回数
NUM_RUNS = 10

def log_progress(line: str):
    """進捗ログにタイムスタンプ付きで1行書き込む"""
    with open(PROGRESS_LOG, 'a', encoding='utf-8') as lf:
        lf.write(line + "\n")
        lf.flush()

# ---------------------------------------------------------
# パラメータスイープ設定
# ---------------------------------------------------------
Y_POSITIONS = [1.0] # 基地局のY位置 (m)
ANGLES_DEG = [round(0.2 * i, 2) for i in range(76)] # 0.0, 0.2, 0.4, ..., 15.0 (76 steps)


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

def main():
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
    log_progress(f"START {datetime.datetime.now().isoformat()} TOTAL={total_runs_tasks}")

    print(f"Starting parameter sweep: Y_POSITIONS={Y_POSITIONS}, ANGLES_DEG={ANGLES_DEG}")
    print(f"Number of runs per task: {NUM_RUNS}")
    print(f"Total tasks across all runs: {total_runs_tasks}")

    summary_files = []

    for run_idx in range(1, NUM_RUNS + 1):
        summary_filename = f"sweep_summary_{sweep_start_time}_run{run_idx}.csv"
        summary_files.append(os.path.join("sim_results", f"sweep_{sweep_start_time}", f"sweep_summary_run{run_idx}.csv"))

        print(f"\n=======================================================")
        print(f"Starting Run {run_idx}/{NUM_RUNS}")
        print(f"=======================================================")

        task_no = 0
        for y in Y_POSITIONS:
            for angle_deg in ANGLES_DEG:
                task_no += 1
                overall_task_no = (run_idx - 1) * total_tasks_per_run + task_no
                angle_rad = math.radians(angle_deg)
                
                # 基地局は -y 方向(yaw=-1.5708)を向いており、アンテナはそこから -90度(yaw=-1.5708) で -x を向く
                # それに angle_rad を足してスイープする
                base_yaw = -1.5708
                antenna_yaw = -1.5708 + angle_rad

                pct = (overall_task_no - 1) / total_runs_tasks * 100
                print(f"\n=======================================================")
                print(f"[Run {run_idx}/{NUM_RUNS}] Task {task_no}/{total_tasks_per_run} (Overall: {overall_task_no}/{total_runs_tasks}, {pct:.1f}%) Y = {y} m, Angle = {angle_deg} deg")
                print(f"=======================================================")
                log_progress(f"RUNNING task={overall_task_no} y={y} angle={angle_deg} ts={datetime.datetime.now().isoformat()}")

                # YAMLファイルを文字列として読み込む（コメント保持のため）
                with open(CONFIG_PATH, 'r', encoding='utf-8') as f:
                    content = f.read()

                # summary_filenameの更新
                content = re.sub(r'\n\s*summary_filename:\s*".*?"', '', content)
                content = re.sub(r'(simulation:)', rf'\1\n  summary_filename: "{summary_filename}"', content)

                # headless: false -> true
                content = re.sub(r'headless:\s*false', 'headless: true', content)

                # real_time_factor を高速化 (sweep高速化)
                content = re.sub(r'real_time_factor:\s*[\d\.]+', f'real_time_factor: {SWEEP_REAL_TIME_FACTOR}', content)

                # 基地局Y座標の更新 (antenna_0 / antenna_1 の pose)
                # pose: [ X, Y, Z, roll, pitch, yaw ]
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

                # 新幹線のアンテナ角度の更新 (基地局のアンテナと対向させるため angle_rad をそのまま適用)
                for ant_name in ["shinkansen_front", "shinkansen_mid", "shinkansen_rear"]:
                    content = re.sub(
                        rf'(name:\s*"{ant_name}".*?relative_rpy:\s*\[\s*[-\d\.]+,\s*[-\d\.]+,\s*)[-\d\.]+(\s*\])',
                        rf'\g<1>{angle_rad:.4f}\g<2>',
                        content,
                        flags=re.DOTALL
                    )

                # YAMLの書き込み
                with open(CONFIG_PATH, 'w', encoding='utf-8') as f:
                    f.write(content)

                # 書き込み後の内容を検証
                with open(CONFIG_PATH, 'r', encoding='utf-8') as f_verify:
                    verify_content = f_verify.read()
                    verify_match = re.findall(r'summary_filename:\s*"(.*?)"', verify_content)
                    print(f"DEBUG_VERIFY: Written summary_filename in yaml is: {verify_match}")

                # シミュレーションの実行
                # Docker内かホストかを判定してコマンドを選択
                is_docker = os.path.exists('/.dockerenv')
                if is_docker:
                    cmd = ["bash", "-c", "export FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/config/fastdds_no_shm.xml && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch comms_sim_pkg sim_launch.py"]
                else:
                    cmd = ["docker", "compose", "exec", "-T", "sim", "bash", "-c", "export FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/config/fastdds_no_shm.xml && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch comms_sim_pkg sim_launch.py"]

                # os.setsid() で新規プロセスグループを作成し、タイムアウト時に
                # gz sim / ROS ノードなど孫プロセスまで SIGKILL で一括終了できるようにする
                proc = subprocess.Popen(
                    cmd,
                    preexec_fn=os.setsid  # 新規プロセスグループリーダーに昇格
                )
                timed_out = False
                try:
                    proc.wait(timeout=TASK_TIMEOUT_SEC)
                except subprocess.TimeoutExpired:
                    timed_out = True
                    print(f"[Run {run_idx}/{NUM_RUNS}] Overall Task {overall_task_no}/{total_runs_tasks} Simulation TIMEOUT ({TASK_TIMEOUT_SEC}s): Y={y}, Angle={angle_deg}")
                    log_progress(f"DONE task={overall_task_no} y={y} angle={angle_deg} status=TIMEOUT ts={datetime.datetime.now().isoformat()}")
                finally:
                    # プロセスグループ全体を確実に終了 (gz sim, ros nodes 含む)
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    except ProcessLookupError:
                        pass  # 既に終了済み

                    # ホストで実行されている場合は、コンテナ内の残存プロセスも強制終了
                    if not is_docker:
                        try:
                            subprocess.run(["docker", "compose", "exec", "-T", "sim", "pkill", "-9", "-f", "python3"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                            subprocess.run(["docker", "compose", "exec", "-T", "sim", "pkill", "-9", "-f", "gz"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                        except Exception:
                            pass

                if not timed_out:
                    if proc.returncode == 0:
                        print(f"[Run {run_idx}/{NUM_RUNS}] Overall Task {overall_task_no}/{total_runs_tasks} Simulation finished: Y={y}, Angle={angle_deg}")
                        log_progress(f"DONE task={overall_task_no} y={y} angle={angle_deg} status=OK ts={datetime.datetime.now().isoformat()}")
                    else:
                        print(f"[Run {run_idx}/{NUM_RUNS}] Overall Task {overall_task_no}/{total_runs_tasks} Simulation FAILED (rc={proc.returncode}): Y={y}, Angle={angle_deg}")
                        log_progress(f"DONE task={overall_task_no} y={y} angle={angle_deg} status=FAIL ts={datetime.datetime.now().isoformat()}")

                time.sleep(1) # クールダウン

    # 平均化の処理を実行
    print("\nAveraging results across all runs...")
    try:
        final_summary_file = f"sim_results/sweep_{sweep_start_time}/sweep_summary.csv"
        average_summaries(summary_files, final_summary_file)
        print(f"Averaged summary successfully saved to: {final_summary_file}")
    except Exception as e:
        print(f"Failed to average summaries: {e}")

    log_progress(f"DONE task={total_runs_tasks} y=- angle=- status=SWEEP_COMPLETE ts={datetime.datetime.now().isoformat()}")
    print("\nSweep completed! Restoring original config...")
    shutil.copy2(BACKUP_PATH, CONFIG_PATH)

if __name__ == "__main__":
    main()
