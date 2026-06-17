import os
import csv
import glob
import subprocess

# ---------------------------------------------------------------------------
# 内部ヘルパー: pandas による集計処理 (Docker 内・外で共通利用)
# ---------------------------------------------------------------------------
_PANDAS_AGGREGATE_SCRIPT = """
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
    'total_data_MB': ['mean', 'std'],
    'connected_time_s': ['mean', 'std'],
    'average_throughput_Gbps': ['mean', 'std'],
    'average_rssi_dBm': ['mean', 'std'],
    'handover_count': ['mean', 'std']
}}
averaged = combined.groupby(['y_position', 'antenna_angle', 'vehicle_name'], as_index=False).agg(agg_rules)

new_cols = []
for col in averaged.columns:
    if isinstance(col, tuple) and col[0] not in ['y_position', 'antenna_angle', 'vehicle_name']:
        if col[1] == 'mean':
            new_cols.append(col[0])
        else:
            new_cols.append(f"{{col[0]}}_std")
    else:
        new_cols.append(col[0])
averaged.columns = new_cols

for col in ['total_data_MB_std', 'connected_time_s_std', 'average_throughput_Gbps_std', 'average_rssi_dBm_std', 'handover_count_std']:
    averaged[col] = averaged[col].fillna(0.0)

averaged['total_data_MB'] = averaged['total_data_MB'].round(3)
averaged['total_data_MB_std'] = averaged['total_data_MB_std'].round(3)
averaged['connected_time_s'] = averaged['connected_time_s'].round(3)
averaged['connected_time_s_std'] = averaged['connected_time_s_std'].round(3)
averaged['average_throughput_Gbps'] = averaged['average_throughput_Gbps'].round(3)
averaged['average_throughput_Gbps_std'] = averaged['average_throughput_Gbps_std'].round(3)
averaged['average_rssi_dBm'] = averaged['average_rssi_dBm'].round(3)
averaged['average_rssi_dBm_std'] = averaged['average_rssi_dBm_std'].round(3)
averaged['handover_count'] = averaged['handover_count'].round(1)
averaged['handover_count_std'] = averaged['handover_count_std'].round(1)

averaged.insert(0, 'run_id', 'AVERAGE')
averaged = averaged.sort_values(by=['y_position', 'antenna_angle', 'vehicle_name'])
os.makedirs(os.path.dirname(output_file), exist_ok=True)
averaged.to_csv(output_file, index=False)
"""


def _build_averaged_df(dfs):
    """pandas による集計処理を行い averaged DataFrame を返す (共通ロジック)"""
    import pandas as pd

    combined = pd.concat(dfs, ignore_index=True)
    agg_rules = {
        'total_data_MB': ['mean', 'std'],
        'connected_time_s': ['mean', 'std'],
        'average_throughput_Gbps': ['mean', 'std'],
        'average_rssi_dBm': ['mean', 'std'],
        'handover_count': ['mean', 'std']
    }
    averaged = combined.groupby(['y_position', 'antenna_angle', 'vehicle_name'], as_index=False).agg(agg_rules)

    new_cols = []
    for col in averaged.columns:
        if isinstance(col, tuple) and col[0] not in ['y_position', 'antenna_angle', 'vehicle_name']:
            new_cols.append(col[0] if col[1] == 'mean' else f"{col[0]}_std")
        else:
            new_cols.append(col[0])
    averaged.columns = new_cols

    std_cols = ['total_data_MB_std', 'connected_time_s_std', 'average_throughput_Gbps_std',
                'average_rssi_dBm_std', 'handover_count_std']
    for col in std_cols:
        averaged[col] = averaged[col].fillna(0.0)

    averaged['total_data_MB'] = averaged['total_data_MB'].round(3)
    averaged['total_data_MB_std'] = averaged['total_data_MB_std'].round(3)
    averaged['connected_time_s'] = averaged['connected_time_s'].round(3)
    averaged['connected_time_s_std'] = averaged['connected_time_s_std'].round(3)
    averaged['average_throughput_Gbps'] = averaged['average_throughput_Gbps'].round(3)
    averaged['average_throughput_Gbps_std'] = averaged['average_throughput_Gbps_std'].round(3)
    averaged['average_rssi_dBm'] = averaged['average_rssi_dBm'].round(3)
    averaged['average_rssi_dBm_std'] = averaged['average_rssi_dBm_std'].round(3)
    averaged['handover_count'] = averaged['handover_count'].round(1)
    averaged['handover_count_std'] = averaged['handover_count_std'].round(1)

    averaged.insert(0, 'run_id', 'AVERAGE')
    averaged = averaged.sort_values(by=['y_position', 'antenna_angle', 'vehicle_name'])
    return averaged


def average_summaries(summary_files, output_file):
    """各ランのCSVファイルを読み込んで平均値と標準偏差を集計・保存する"""
    is_docker = os.path.exists('/.dockerenv')

    if not is_docker:
        # ホスト側実行: Docker コンテナ内の python3 に集計スクリプトを渡す
        ws_summary_files = [f"/workspace/{f}" if not f.startswith('/') else f for f in summary_files]
        ws_output_file = f"/workspace/{output_file}" if not output_file.startswith('/') else output_file

        script = _PANDAS_AGGREGATE_SCRIPT.format(
            ws_summary_files=ws_summary_files,
            ws_output_file=ws_output_file
        )
        cmd = ["docker", "compose", "exec", "-T", "sim", "python3", "-c", script]
        try:
            subprocess.run(cmd, check=True)
        except Exception:
            cmd = ["docker", "exec", "comms_sim", "python3", "-c", script]
            subprocess.run(cmd, check=True)
        return

    # Docker 内実行: 直接 pandas で集計
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

    averaged = _build_averaged_df(dfs)
    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    averaged.to_csv(output_file, index=False)


def get_completed_tasks(sweep_dir, run_idx):
    completed = set()

    # 1. 未マージのワーカー個別 CSV から読み込み (旧形式と新形式の両方をサポート)
    patterns = [
        os.path.join(sweep_dir, f"sweep_summary_*_run{run_idx}_w*.csv"),
        os.path.join(sweep_dir, f"sweep_summary_*_run{run_idx}_*_w*.csv")
    ]
    for pattern in patterns:
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
