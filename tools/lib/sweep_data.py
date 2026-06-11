import os
import csv
import glob
import subprocess

def average_summaries(summary_files, output_file):
    """各ランのCSVファイルを読み込んで平均値を集計・保存する"""
    is_docker = os.path.exists('/.dockerenv')
    if not is_docker:
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

def get_completed_tasks(sweep_dir, run_idx):
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

    # 3. チャンクごとのマージ済み CSV があればそこからも読み込み
    chunk_pattern = os.path.join(sweep_dir, f"sweep_summary_run{run_idx}_chunk*.csv")
    for csv_file in glob.glob(chunk_pattern):
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
            
    return completed
