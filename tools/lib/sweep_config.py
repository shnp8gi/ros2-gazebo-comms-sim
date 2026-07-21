import os
import yaml
import math

# =========================================================================
# 単位プリセット (Unit Registry)
# =========================================================================
UNIT_CONVERTERS = {
    'degree': lambda x: math.radians(x),
    'degrees': lambda x: math.radians(x),
    'km/h': lambda x: x / 3.6,
    'kmh': lambda x: x / 3.6,
}

# =========================================================================
# パラメータスイープ設定 (デフォルト値)
# =========================================================================
# パラメータスイープを繰り返す回数 (ラン数)
NUM_RUNS = 1

# 1タスクあたりの最大待機時間 [秒]
TASK_TIMEOUT_SEC = 300

# スイープ時の加速倍率 (ヘッドレス時のみ有効.1.0=リアルタイム)
SWEEP_REAL_TIME_FACTOR = 1.0

# run毎シード導出 (CRN: Common Random Numbers)
# シードは run_idx のみから導出し、スイープ変数・シナリオには依存させない。
# → 同一 run_idx は全方式・全条件で同一の乱数系列 (CRN)、run 間は独立となり、
#   方式間の対応のある比較 (paired t / Wilcoxon) の分散が大幅に減る。
DERIVE_RUN_SEEDS = True
BASE_SEED = 12345
RUN_SEED_STRIDE = 1000  # run間のシード間隔 (派生シード +1,+2,+10,+20 と衝突しない幅)

# =========================================================================
# 設定値から自動生成されるパラメータ・内部変数
# =========================================================================
PROGRESS_LOG = "tools/log/sweep_progress.log"
CONFIG_PATH = "src/comms_sim_pkg/config/sim_params.yaml"
BACKUP_PATH = "tools/sweep_build/sim_params.yaml.bak"

GENERIC_VARIABLES = []

def resolve_values(config_node):
    if 'values' in config_node:
        return config_node['values']
    elif 'range' in config_node:
        r = config_node['range']
        curr = float(r.get('start', 0.0))
        end = float(r.get('end', 1.0))
        step = float(r.get('step', 0.1))
        vals = []
        if step > 0:
            while curr <= end + 1e-5:
                vals.append(round(curr, 5))
                curr += step
        elif step < 0:
            while curr >= end - 1e-5:
                vals.append(round(curr, 5))
                curr += step
        else:
            vals.append(curr)
        return vals
    return [0.0]

def parse_target(target_node, val, fallback_unit=None):
    unit = target_node.get('unit', fallback_unit)
    converted_val = val
    if unit in UNIT_CONVERTERS:
        converted_val = UNIT_CONVERTERS[unit](val)

    out = {
        'field': target_node.get('field'),
        'value': converted_val,
        'raw_value': val,
        'unit': unit if unit else ""
    }
    # config ターゲット: エンティティではなく生成後の sim_params 辞書への
    # ドットパス (例 "link_controller_node.ros__parameters.control_plane")
    if 'config' in target_node:
        out['config_path'] = target_node['config']

    if 'entity_role' in target_node:
        out['role'] = target_node['entity_role']
    elif 'role' in target_node:
        out['role'] = target_node['role']

    if 'entity_name' in target_node:
        out['entity_name'] = target_node['entity_name']
    elif 'name' in target_node:
        out['entity_name'] = target_node['name']
    return out

def run_seed_for(run_idx, base_seed=None):
    """run_idx (1始まり) から run 固有の基点シードを導出する。"""
    if base_seed is None:
        base_seed = BASE_SEED
    return int(base_seed) + RUN_SEED_STRIDE * (int(run_idx) - 1)


def inject_run_seeds(config_dict, run_idx):
    """生成済み sim_params 辞書へ run 導出シードを注入する (CRN対応)。

    シナリオ yaml に固定シードが書かれていても本注入が優先される
    (固定シードのままでは num_runs を増やしても全 run が同一乱数になるため)。
    execution.derive_run_seeds: false で従来動作 (yaml のシードそのまま) に戻せる。

    注入先 (存在するセクションのみ):
      - comms_simulator_node...channel.seed            (内部で +1=shadowing, +2=fading)
      - comms_simulator_node...measurement_report.seed (制御プレーンへの観測雑音)
      - link_controller_node...kkf_seed                (C++組込 kkf_predictive の観測雑音)
    channel セクションの有無は物理モデル構成自体を変えるため、無い場合に
    seed だけを新設することはしない。
    """
    if not DERIVE_RUN_SEEDS:
        return None
    seed0 = run_seed_for(run_idx)
    comms = config_dict.get('comms_simulator_node', {}).get('ros__parameters', {})
    channel = comms.get('channel')
    if isinstance(channel, dict):
        channel['seed'] = seed0
    report = comms.get('measurement_report')
    if isinstance(report, dict):
        report['seed'] = seed0 + 10
    link = config_dict.get('link_controller_node', {}).get('ros__parameters', {})
    if isinstance(link, dict) and link:
        link['kkf_seed'] = seed0 + 20
    return seed0


def expand_cases(var):
    """行指向 `cases:` 記法を states に展開する。

    列指向 targets (値の並列リスト) の可読性問題への対策。1条件=1ブロックで
    「この方式は何が違うのか」がその場で完結する:

      - name: method
        cases:
          lut:
            config:
              link_controller_node.ros__parameters.scheduling_policy: feedforward_optimal
          kkf_full:
            config:
              link_controller_node.ros__parameters.control_plane: kkf_mpc
            entities:
              truck_co_0: { enabled: true }   # エンティティ上書きも可

    config: 生成後 sim_params 辞書へのドットパス→値。
    entities: エンティティ名→{field: value}。
    ケース間でキーが揃っている必要はない (書いたものだけ上書きされる)。
    """
    states = []
    for label, spec in var['cases'].items():
        spec = spec or {}
        overrides = []
        for path, value in (spec.get('config') or {}).items():
            overrides.append({'config_path': path, 'value': value,
                              'raw_value': value, 'field': None, 'unit': ''})
        for ent_name, fields in (spec.get('entities') or {}).items():
            for field, value in (fields or {}).items():
                overrides.append({'entity_name': ent_name, 'field': field,
                                  'value': value, 'raw_value': value, 'unit': ''})
        if not overrides:
            # 空ケース (基準条件など): ラベルを運ぶだけの no-op
            overrides.append({'field': None, 'value': None,
                              'raw_value': label, 'unit': ''})
        for ovr in overrides:
            ovr['state_label'] = str(label)
        states.append(overrides)
    return states


def expand_entity_groups(var):
    """`entity_groups:` 記法を states に展開する (台数・組合せ制御の宣言形)。

      - name: density
        entity_groups:
          "0": []
          "2": [truck_co_0, truck_opp_0]
          "4": [truck_co_0, truck_opp_0, truck_co_1, truck_opp_1]

    意味: 全グループの和集合が制御対象。各状態で、リストに載る
    エンティティは enabled=true、載らないものは enabled=false。
    シナリオ側は全エンティティを定義しておくだけでよい。
    """
    groups = var['entity_groups']
    controlled = []
    for members in groups.values():
        for name in (members or []):
            if name not in controlled:
                controlled.append(name)
    states = []
    for label, members in groups.items():
        members = set(members or [])
        overrides = []
        for name in controlled:
            overrides.append({'entity_name': name, 'field': 'enabled',
                              'value': name in members,
                              'raw_value': name in members, 'unit': '',
                              'state_label': str(label)})
        if not overrides:
            overrides.append({'field': None, 'value': None, 'raw_value': label,
                              'unit': '', 'state_label': str(label)})
        states.append(overrides)
    return states


def load_sweep_config(yaml_path=None):
    global NUM_RUNS, TASK_TIMEOUT_SEC, SWEEP_REAL_TIME_FACTOR
    global GENERIC_VARIABLES
    global DERIVE_RUN_SEEDS, BASE_SEED
    
    sweep_data = None
    GENERIC_VARIABLES = []
    
    if yaml_path and os.path.exists(yaml_path):
        with open(yaml_path, 'r', encoding='utf-8') as f:
            sweep_data = yaml.safe_load(f).get('sweep', {})
            
    if sweep_data:
        exec_cfg = sweep_data.get('execution', {})
        NUM_RUNS = exec_cfg.get('num_runs', NUM_RUNS)
        TASK_TIMEOUT_SEC = exec_cfg.get('task_timeout_sec', TASK_TIMEOUT_SEC)
        SWEEP_REAL_TIME_FACTOR = exec_cfg.get('real_time_factor', SWEEP_REAL_TIME_FACTOR)
        DERIVE_RUN_SEEDS = exec_cfg.get('derive_run_seeds', DERIVE_RUN_SEEDS)
        BASE_SEED = exec_cfg.get('base_seed', BASE_SEED)
        
        for var in sweep_data.get('variables', []):
            var_data = {
                'name': var.get('name'),
                'states': []
            }

            unit = var.get('unit')
            # labels: 状態ごとの表示名 (結果CSVの列値・ログに使用)。
            # 省略時は先頭ターゲットの raw_value が使われる。
            labels = var.get('labels')

            if 'cases' in var:
                var_data['states'] = expand_cases(var)

            elif 'entity_groups' in var:
                var_data['states'] = expand_entity_groups(var)

            elif 'targets' in var:
                targets_defs = var['targets']
                target_value_lists = []
                for t in targets_defs:
                    target_value_lists.append(resolve_values(t))

                if not target_value_lists:
                    continue

                min_len = min(len(l) for l in target_value_lists)
                states = []
                for i in range(min_len):
                    state_overrides = []
                    for t_idx, t in enumerate(targets_defs):
                        raw_val = target_value_lists[t_idx][i]
                        state_overrides.append(parse_target(t, raw_val, unit))
                    if labels and i < len(labels):
                        for ovr in state_overrides:
                            ovr['state_label'] = labels[i]
                    states.append(state_overrides)
                var_data['states'] = states

            elif 'target' in var:
                target_def = var['target']
                vals = resolve_values(var)
                states = []
                for i, val in enumerate(vals):
                    state_overrides = [parse_target(target_def, val, unit)]
                    if labels and i < len(labels):
                        state_overrides[0]['state_label'] = labels[i]
                    states.append(state_overrides)
                var_data['states'] = states

            if var_data['states']:
                GENERIC_VARIABLES.append(var_data)
                
    return sweep_data

# Initialize default
load_sweep_config()
