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


class ScenarioConfigProvider:
    """
    シミュレーションシナリオ設定を抽象化し、期待されるエンティティ名を提供する
    """
    def __init__(self, config_path):
        self.config_path = config_path

    def get_expected_entities(self):
        import yaml
        expected = []
        if not self.config_path or not os.path.exists(self.config_path):
            return expected
        try:
            with open(self.config_path, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
            vehicles = config.get('vehicles', [])
            if not vehicles:
                return expected
            for v in vehicles:
                v_name = v.get('name', 'suv')
                antennas = v.get('antennas', [])
                if antennas:
                    for a in antennas:
                        expected.append(a.get('name'))
                else:
                    expected.append(v_name)
            
            # Shinkansen has special _total aggregation if >= 3 antennas
            shinkansen_vns = [vn for vn in expected if 'shinkansen' in vn]
            if len(shinkansen_vns) >= 3:
                expected.append('shinkansen_total')
        except Exception as e:
            print(f"Warning: Failed to parse entities from config: {e}")
        return sorted(list(set(expected)))

class SweepResultRepository:
    """
    CSV結果から実績結果を取得する
    """
    def __init__(self, summary_path):
        self.summary_path = summary_path
    
    def get_actual_entities(self):
        actual = set()
        if not os.path.exists(self.summary_path):
            return actual
        try:
            with open(self.summary_path, 'r', encoding='utf-8') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    v_name = row.get('vehicle_name')
                    if v_name:
                        actual.add(v_name)
        except Exception:
            pass
        return actual

class SweepValidator:
    """
    シナリオの期待値と実際の結果を比較するドメインロジック
    """
    def __init__(self, expected_entities):
        self.expected_entities = expected_entities
        
    def validate(self, actual_entities):
        missing = [ent for ent in self.expected_entities if ent not in actual_entities]
        return len(missing) == 0, missing

def get_expected_vehicles(config_path):
    """(Backward compatibility wrapper)"""
    return ScenarioConfigProvider(config_path).get_expected_entities()

def validate_sweep_summary(sweep_dir, config_path, y_positions, angles_deg, num_runs):
    """
    スイープ結果のデータ完全性と健全性を検証する
    """
    provider = ScenarioConfigProvider(config_path)
    expected_entities = provider.get_expected_entities()
    
    if not expected_entities:
        return True, []

    failed_tasks = []
    validator = SweepValidator(expected_entities)

    for run_idx in range(1, num_runs + 1):
        summary_path = os.path.join(sweep_dir, f"sweep_summary_run{run_idx}.csv")
        if not os.path.exists(summary_path):
            # Missing completely
            for ry in y_positions:
                for ra in angles_deg:
                    failed_tasks.append({
                        'run_idx': run_idx, 'y': ry, 'angle': ra,
                        'reason': 'Summary file not found'
                    })
            continue

        repo = SweepResultRepository(summary_path)
        actual_entities = repo.get_actual_entities()
        
        is_valid, missing = validator.validate(actual_entities)
        if not is_valid:
            print(f"Validation failed for {summary_path}. Missing: {missing}")
            failed_tasks.append({'run_idx': run_idx, 'y': -1, 'angle': -1, 'reason': f"Missing entities: {missing}"})
            
    is_valid = len(failed_tasks) == 0
    return is_valid, failed_tasks


def get_completed_tasks(sweep_dir, run_idx, config_path=None):
    expected_vehicles = get_expected_vehicles(config_path) if config_path else []
    task_records = {}

    def parse_file(csv_file):
        if not os.path.exists(csv_file):
            return
        try:
            with open(csv_file, 'r', encoding='utf-8') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    try:
                        y_val = float(row['y_position'])
                        ang_val = float(row['antenna_angle'])
                        v_name = row.get('vehicle_name')
                        if not v_name:
                            continue
                        key = (round(y_val, 2), round(ang_val, 2))
                        if key not in task_records:
                            task_records[key] = {}
                        task_records[key][v_name] = row
                    except (ValueError, KeyError):
                        continue
        except Exception:
            pass

    # 1. 未マージのワーカー個別 CSV
    patterns = [
        os.path.join(sweep_dir, f"sweep_summary_*_run{run_idx}_w*.csv"),
        os.path.join(sweep_dir, f"sweep_summary_*_run{run_idx}_*_w*.csv")
    ]
    for pattern in patterns:
        for csv_file in glob.glob(pattern):
            parse_file(csv_file)

    # 2. すでにマージ済みの CSV
    merged_file = os.path.join(sweep_dir, f"sweep_summary_run{run_idx}.csv")
    parse_file(merged_file)

    # 3. チャンクごとのマージ済み CSV
    chunk_pattern = os.path.join(sweep_dir, f"sweep_summary_run{run_idx}_chunk*.csv")
    for csv_file in glob.glob(chunk_pattern):
        parse_file(csv_file)

    completed = set()
    numeric_cols = ['total_data_MB', 'connected_time_s', 'average_throughput_Gbps', 'average_rssi_dBm', 'handover_count']

    for (ry, ra), vehicles_dict in task_records.items():
        if expected_vehicles:
            # 1. 全車両が存在するか
            missing_v = [v for v in expected_vehicles if v not in vehicles_dict]
            if missing_v:
                continue
            
            # 2. NaN/無効値が無いか
            has_invalid = False
            for v_name in expected_vehicles:
                row = vehicles_dict[v_name]
                for col in numeric_cols:
                    val = row.get(col)
                    if val is None or val == '' or val.lower() in ['nan', 'null', 'none']:
                        has_invalid = True
                        break
                    try:
                        float(val)
                    except ValueError:
                        has_invalid = True
                        break
                if has_invalid:
                    break
            
            if has_invalid:
                continue

        completed.add((ry, ra))

    return completed
