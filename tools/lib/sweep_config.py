import os

# =========================================================================
# パラメータスイープ設定 (シミュレーションパラメータ)
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
_curr_ang = START_ANGLE
while _curr_ang <= END_ANGLE + 1e-5:
    ANGLES_DEG.append(round(_curr_ang, 1))
    _curr_ang += STEP_ANGLE
