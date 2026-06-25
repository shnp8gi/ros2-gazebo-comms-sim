import os
import yaml

# =========================================================================
# パラメータスイープ設定 (デフォルト値)
# =========================================================================
# 基地局のY位置リスト (m)
Y_POSITIONS = [3.0]

# 基地局自体の向き (ヨー角) (deg)
RX_YAW_DEG = -90.0

# 角度スイープの設定 (下限, 上限, 刻み幅)
START_ANGLE = 0.0# 下限 (deg)
END_ANGLE = 180.0    # 上限 (deg)
STEP_ANGLE = 0.1  # 刻み幅 (deg)

# パラメータスイープを繰り返す回数 (ラン数)
NUM_RUNS = 1

# 1タスクあたりの最大待機時間 [秒]
TASK_TIMEOUT_SEC = 300

# スイープ時の加速倍率 (ヘッドレス時のみ有効.1.0=リアルタイム)
SWEEP_REAL_TIME_FACTOR = 1.0

# =========================================================================
# 設定値から自動生成されるパラメータ・内部変数
# =========================================================================
PROGRESS_LOG = "tools/log/sweep_progress.log"
CONFIG_PATH = "src/comms_sim_pkg/config/sim_params.yaml"
BACKUP_PATH = "tools/sweep_build/sim_params.yaml.bak"

ANGLES_DEG = []

def load_sweep_config(yaml_path=None):
    global Y_POSITIONS, RX_YAW_DEG, START_ANGLE, END_ANGLE, STEP_ANGLE
    global NUM_RUNS, TASK_TIMEOUT_SEC, SWEEP_REAL_TIME_FACTOR, ANGLES_DEG
    global GENERIC_VARIABLES
    
    sweep_data = None
    GENERIC_VARIABLES = []
    
    if yaml_path and os.path.exists(yaml_path):
        with open(yaml_path, 'r', encoding='utf-8') as f:
            sweep_data = yaml.safe_load(f).get('sweep', {})
            
    if sweep_data:
        # Load from YAML
        exec_cfg = sweep_data.get('execution', {})
        NUM_RUNS = exec_cfg.get('num_runs', NUM_RUNS)
        TASK_TIMEOUT_SEC = exec_cfg.get('task_timeout_sec', TASK_TIMEOUT_SEC)
        SWEEP_REAL_TIME_FACTOR = exec_cfg.get('real_time_factor', SWEEP_REAL_TIME_FACTOR)
        RX_YAW_DEG = exec_cfg.get('base_station_yaw_deg', RX_YAW_DEG)
        
        # Parse variables
        for var in sweep_data.get('variables', []):
            if var.get('name') == 'rx_y_position':
                Y_POSITIONS = var.get('values', Y_POSITIONS)
            elif var.get('name') == 'rx_antenna_yaw':
                v_range = var.get('range', {})
                START_ANGLE = v_range.get('start', START_ANGLE)
                END_ANGLE = v_range.get('end', END_ANGLE)
                STEP_ANGLE = v_range.get('step', STEP_ANGLE)
                
            # Store everything for generic processing
            var_data = {
                'name': var.get('name'),
                'target': var.get('target', {}),
                'values': []
            }
            if 'values' in var:
                var_data['values'] = var['values']
            elif 'range' in var:
                r = var['range']
                curr = r.get('start', 0.0)
                end = r.get('end', 1.0)
                step = r.get('step', 0.1)
                vals = []
                while curr <= end + 1e-5:
                    vals.append(round(curr, 5))
                    curr += step
                var_data['values'] = vals
            GENERIC_VARIABLES.append(var_data)
                
    ANGLES_DEG.clear()
    _curr_ang = START_ANGLE
    while _curr_ang <= END_ANGLE + 1e-5:
        ANGLES_DEG.append(round(_curr_ang, 1))
        _curr_ang += STEP_ANGLE
        
    # If no generic variables defined, add the default ones to maintain backward compatibility
    if not GENERIC_VARIABLES:
        GENERIC_VARIABLES.append({
            'name': 'rx_y_position',
            'target': {'role': 'rx', 'field': 'pose[1]'},
            'values': Y_POSITIONS
        })
        GENERIC_VARIABLES.append({
            'name': 'rx_antenna_yaw',
            'target': {'role': 'rx', 'field': 'antennas.*.relative_rpy[2]'},
            'values': ANGLES_DEG
        })
        
    return sweep_data

# Initialize default
load_sweep_config()
