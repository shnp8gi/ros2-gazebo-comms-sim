"""
task_formatting.py
------------------
スイープタスクの変数情報 (task_vars) から、ログ/ファイル名用の文字列や
出力メタデータ用の値を導出する純粋関数群。

責務: task_vars の「解釈・整形」のみ。ファイルI/O・プロセス起動・状態を持たない。
task_vars の形式: {変数名: [{'value': ..., 'raw_value': ..., 'unit': ...}, ...]}
"""


def _format_number(value) -> str:
    """float は %g 表記、それ以外は str() で文字列化する。"""
    return f"{value:g}" if isinstance(value, float) else str(value)


def build_logging_strings(task_vars: dict) -> tuple:
    """
    タスク変数からファイル名用サフィックスと表示用パラメータ文字列を生成する。

    同期スイープ等で複数ターゲットが存在する場合でも、ファイルパスの過剰な
    肥大化を防ぐため代表値(最初のターゲット値)を採用する。

    Returns:
        (task_suffix, params_str)
    """
    task_suffix_parts = []
    params_list = []

    for name, state_overrides in task_vars.items():
        if not state_overrides:
            continue

        val = state_overrides[0].get('value', 0)
        raw_val = state_overrides[0].get('raw_value', val)
        unit_str = state_overrides[0].get('unit', '')

        task_suffix_parts.append(f"{name}_{_format_number(val)}")
        params_list.append(f"{name}={_format_number(raw_val)}{unit_str}")

    task_suffix = "_".join(task_suffix_parts) if task_suffix_parts else "default"
    params_str = ",".join(params_list)
    return task_suffix, params_str


def extract_legacy_overrides(task_vars: dict) -> tuple:
    """
    出力メタデータ(旧形式)に必要な値を抽出する。
    'rx_y_position' の値と、変数名に 'yaw' を含む最初の変数の値を返す。

    Returns:
        (y_val, angle_val)
    """
    y_val = 0.0
    angle_val = 0.0
    for name, state_overrides in task_vars.items():
        if not state_overrides:
            continue
        val = state_overrides[0].get('value', 0)

        if name == 'rx_y_position':
            y_val = float(val)
        if 'yaw' in name.lower():
            angle_val = float(val)

    return y_val, angle_val
