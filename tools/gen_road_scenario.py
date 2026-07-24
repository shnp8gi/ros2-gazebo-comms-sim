#!/usr/bin/env python3
"""
本番シナリオ (幹線道路 N台×M RSU) のパラメトリック生成 (本番仕様 §3.2)。

責務: 幾何パラメータ → scenario yaml の決定論生成のみ。
生成物はチェックインし、真実は生成された yaml とする (評価ディレクトリ契約)。
遮蔽交通は生成しない — traffic: セクションを書き、run 毎の展開は
tools/lib/traffic_gen.py (sweep 実行時) の責務。

アンテナ角の規約 (すべて設定可能、既定は元仕様の字義どおり):
  - RSU: 車線正対 (-y, yaw=-90°) を基準に、下流 (+x) へ rsu_tilt_deg 振る
         → yaw = -90° + rsu_tilt_deg
  - 車:  進行方向 (+x, yaw=0) を基準に、RSU側 (+y) へ veh_tilt_deg 振る
         → relative_rpy yaw = +veh_tilt_deg
  注: 両者 15° では厳密な相互ボアサイト対向は成立しない (幾何的に不可能)。
  意図は「向き合うハの字」による通信ウィンドウの前方シフトであり、
  厳密対向を作りたい場合は rsu_tilt_deg = atan2(rsu_y, Δs) - 90° 等を指定する。

使い方:
  python3 tools/gen_road_scenario.py --out config/scenarios/road_10car.yaml
"""
import argparse
import math
import os
import sys

import yaml


def build_scenario(a):
    rsu_x0 = -a.rsu_spacing * (a.num_rsu - 1) / 2.0
    rsu_xs = [rsu_x0 + i * a.rsu_spacing for i in range(a.num_rsu)]
    rsu_yaw = math.radians(-90.0 + a.rsu_tilt_deg)
    veh_yaw = math.radians(a.veh_tilt_deg)

    x_end = rsu_xs[-1] + a.approach_m
    x_lead = rsu_xs[0] - a.approach_m
    car_starts = [x_lead - a.car_gap * k for k in range(a.num_cars)]
    x_first = car_starts[-1]                      # 最後尾車の開始位置
    total_time = (x_end - x_first) / a.speed      # 最後尾車の総走行時間

    # 走行間固定シャドウ場は全走行域 + 余裕をカバーする
    u_min, u_max = x_first - 50.0, x_end + 50.0

    entities = []
    for i, x in enumerate(rsu_xs):
        entities.append({
            'name': f"antenna_{i}",
            'model': 'antenna',
            'role': 'rx',
            'static': True,
            'pose': [float(x), float(a.rsu_y), 0.0, 0.0, 0.0, round(rsu_yaw, 6)],
            'antennas': [{'name': f"antenna_{i}",
                          'offset': [0.0, 0.0, float(a.rsu_h)],
                          'relative_rpy': [0.0, 0.0, 0.0]}],
        })
    for k, x0 in enumerate(car_starts, start=1):
        entities.append({
            'name': f"car_{k}",
            'model': 'Car',
            'role': 'tx',
            'pose': [float(x0), 0.0, 0.0, 0.0, 0.0, 0.0],
            'waypoints': [[float(x0), 0.0, 0.0, float(a.speed)],
                          [float(x_end), 0.0, 0.0, float(a.speed)]],
            # 単一アンテナ (本番仕様 §3.1): 車室内天井 ant_h【実測】
            'antennas': [{'name': f"car_{k}_ant",
                          'offset': [0.0, 0.0, float(a.ant_h)],
                          'relative_rpy': [0.0, 0.0, round(veh_yaw, 6)]}],
        })

    scenario = {
        'scenario': {
            'name': a.name,
            'description': (
                f"幹線道路 {a.speed*3.6:.0f}km/h・乗用車{a.num_cars}台×RSU{a.num_rsu}局 "
                f"(需要超過) + 車種混合の確率交通遮蔽 (traffic: を run 毎展開)。"
                f"RSU yaw=-90°+{a.rsu_tilt_deg}° (下流へ), 車アンテナ +{a.veh_tilt_deg}° (RSU側へ)。"
                "tools/gen_road_scenario.py により生成"),
            'base_config': 'src/comms_sim_pkg/config/sim_params.yaml',
            'simulation_overrides': {
                'headless': True,
                'real_time_factor': a.rtf,
                'logging_level': 3,
                'physics_max_step_size': 0.001,
            },
            'config_overrides': {
                'link_controller_node': {
                    'ros__parameters': {
                        'scheduling_policy': 'external_schedule',
                        'control_plane': 'kkf_mpc',
                        'schedule_topic': '/comms/ho_schedule',
                        'ff_max_pairs': 1,
                        # trend/kf フォールバック用の決定論プロファイル出力。
                        # kkf_cold/conv は prior を使わない (kkf_use_prior: false)
                        'export_rssi_profile': True,
                        'kkf_use_prior': False,
                        # --- L3 (本番仕様 §6): ハンガリアン割当 μ−κσ ---
                        'kkf_assigner': 'hungarian',
                        'kkf_switch_bonus_db': 3.0,
                        'kkf_replan_period_s': 0.2,
                        'kkf_horizon_s': 4.0,
                        'kkf_plan_dt_s': 0.2,
                        'kkf_kappa': 1.0,
                        'kkf_idle_lcb_db': -110.0,
                        # --- 第1層: RBF基底 + σ_ν²(s) 自己組織化 (§5) ---
                        'kkf_basis': 'rbf',
                        'kkf_rbf_num_bases': 20,
                        'kkf_rbf_width_m': 0.0,   # 0 = 中心間隔
                        'kkf_sigma_nu_db': 4.0,
                        'kkf_corr_length_s_m': 20.0,
                        'kkf_corr_length_t_s': 5.0,
                        'kkf_varmap_enabled': True,
                        'kkf_varmap_grid_m': 2.0,
                        'kkf_varmap_prior_var_db2': 0.0,
                        # --- 走行間永続化 (§5.3)。空 = cold start ---
                        'kkf_state_dir': '',
                        'kkf_state_save': False,
                        'kkf_q_forget': 0.05,
                        'kkf_state_gamma': 0.997,
                        # --- 第2層トラッカーは本番では無効 (σ_ν² が代替、§5.2) ---
                        'kkf_tracker_enabled': False,
                        # --- ts_kf ベースライン ---
                        'tskf_process_noise_q': 25.0,
                        'tskf_prior_mean_dbm': -120.0,
                        'tskf_prior_var': 400.0,
                    },
                },
                'comms_simulator_node': {
                    'ros__parameters': {
                        'tx_power': a.tx_power,
                        # 通信計算の間引き (§P6 高速化)。物理 1kHz に対し 200Hz。
                        # report 周期 0.05s 以下なので観測レポートは不変、
                        # データ会計は実効dtで総量一致。多車両×多遮蔽体の RTF 対策
                        'comms_update_period_s': a.comms_update_period_s,
                        # 雑音床基準の Shannon レート写像 (§4.3)
                        'rate_model': {
                            'type': 'shannon',
                            'bandwidth_hz': 100.0e6,
                            'noise_floor_dbm': -95.0,
                            'efficiency': 1.0,
                            'snr_min_db': 0.0,
                            'snr_cap_db': 50.0,
                        },
                        'channel': {
                            'seed': 42,  # run 毎に注入 (frozen シャドウは参照しない)
                            'blockage': {'enabled': True, 'max_total_loss_db': 60.0},
                            # 走行間固定の持続シャドウ (§4.2)。environment_seed
                            # のみが場を決める (学習相・評価相で同一環境)
                            'shadowing': {'enabled': True, 'type': 'frozen',
                                          'sigma_db': 4.0, 'corr_length_m': 6.0,
                                          'environment_seed': a.environment_seed,
                                          'grid_m': 1.5, 'axis': 'x',
                                          'u_min': round(u_min, 1),
                                          'u_max': round(u_max, 1)},
                            # マルチパス・高速フェージングは入れない (§4.1)
                            'fading': {'enabled': False},
                        },
                        'measurement_report': {
                            'enabled': True,
                            'period_s': 0.05,
                            'noise_std_db': 2.0,
                            'observe_all_pairs': False,  # arm が sweep で上書き
                            'seed': 123,                 # run 毎に注入
                        },
                    },
                },
            },
            'model_catalog': {
                'Car': {
                    'sdf_path': 'models://SimpleBox',
                    'description': '配信対象の乗用車 (軽量box)',
                    'default_antenna_offset': [0.0, 0.0, float(a.ant_h)],
                },
                'antenna': {
                    'sdf_path': 'models://antenna',
                    'description': '沿道RSUアンテナ',
                    'default_antenna_offset': [0.0, 0.0, float(a.rsu_h)],
                },
                # 車種構成 (§3.3): 乗用車は RSU→車載の視線 (1.7〜2.2m) の下を
                # 通過するため遮蔽しない。大型車のみ遮蔽 = 車種構成が発生率を決める
                'SedanBlocker': {
                    'sdf_path': 'models://GhostBox',
                    'description': '乗用車 (車高1.5m、視線下=遮蔽せず)',
                    'blockage': {'size': [4.5, 1.8, 1.5], 'loss_db': 8.0},
                },
                'TruckBlocker': {
                    'sdf_path': 'models://GhostBox',
                    'description': 'トラック (車高3.8m)',
                    'blockage': {'size': [12.0, 2.5, 3.8], 'loss_db': 26.0},
                },
                'BusBlocker': {
                    'sdf_path': 'models://GhostBox',
                    'description': 'バス (車高3.2m)',
                    'blockage': {'size': [11.0, 2.5, 3.2], 'loss_db': 22.0},
                },
            },
            # run 毎に traffic_gen が展開する確率交通 (§3.3)。密度・混合比は
            # sweep 変数で上書きして掃引する
            'traffic': {
                'seed': 0,
                'window_s': [0.0, round(total_time + 2.0, 1)],
                'corridor': [round(rsu_xs[0] - a.approach_m, 1),
                             round(rsu_xs[-1] + a.approach_m, 1)],
                'margin_m': 30.0,
                'max_vehicles': 200,
                'tx_entry_jitter_s': 0.0,
                'vehicle_mix': [
                    {'model': 'SedanBlocker', 'ratio': 0.75},
                    {'model': 'TruckBlocker', 'ratio': 0.15},
                    {'model': 'BusBlocker', 'ratio': 0.10},
                ],
                'lanes': [
                    {'y': 2.5, 'direction': 1, 'speed_mps': 5.0,
                     'headway_s': {'dist': 'lognormal', 'mean': 4.0,
                                   'sigma': 0.6, 'min': 1.5}},
                    {'y': 4.5, 'direction': -1, 'speed_mps': 16.7,
                     'headway_s': {'dist': 'exponential', 'mean': 6.0, 'min': 1.5}},
                    {'y': 6.0, 'direction': -1, 'speed_mps': 16.7,
                     'headway_s': {'dist': 'exponential', 'mean': 8.0, 'min': 1.5}},
                ],
            },
            'entities': entities,
        },
    }
    return scenario


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', required=True, help='出力 yaml パス')
    ap.add_argument('--name', default='road_10car')
    ap.add_argument('--num-rsu', type=int, default=3)
    ap.add_argument('--rsu-spacing', type=float, default=60.0)
    ap.add_argument('--rsu-y', type=float, default=8.0, help='RSU 横位置 [m]')
    ap.add_argument('--rsu-h', type=float, default=2.5, help='RSU アンテナ高 [m]【実測】')
    ap.add_argument('--rsu-tilt-deg', type=float, default=15.0,
                    help='車線正対(-y)から下流(+x)への振り角 [deg]')
    ap.add_argument('--num-cars', type=int, default=10)
    ap.add_argument('--car-gap', type=float, default=18.0, help='車間 [m]')
    ap.add_argument('--speed', type=float, default=16.67, help='車速 [m/s] (60km/h)')
    ap.add_argument('--veh-tilt-deg', type=float, default=15.0,
                    help='進行方向(+x)からRSU側(+y)への振り角 [deg]')
    ap.add_argument('--ant-h', type=float, default=1.35, help='車載アンテナ高 [m]【実測】')
    ap.add_argument('--approach-m', type=float, default=90.0,
                    help='端RSUの外側の助走/退出区間 [m]')
    ap.add_argument('--tx-power', type=float, default=-7.0)
    ap.add_argument('--environment-seed', type=int, default=1)
    ap.add_argument('--rtf', type=float, default=2.0)
    ap.add_argument('--comms-update-period-s', type=float, default=0.005,
                    help='通信計算の間引き周期 [s] (0=物理毎)。report周期以下に保つ')
    a = ap.parse_args()

    scenario = build_scenario(a)
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    header = (
        "# 本番シナリオ (本番仕様 docs/production_sim_spec.md)。\n"
        f"# tools/gen_road_scenario.py により生成 — 手編集より再生成を推奨:\n"
        f"#   python3 tools/gen_road_scenario.py --out {a.out}"
        + ''.join(f" \\\n#     {k} {v}" for k, v in []) + "\n"
        "# 角度規約: RSU yaw = -90°+rsu_tilt (下流へ) / 車アンテナ relative yaw = +veh_tilt (RSU側へ)\n"
        "# traffic: は run 毎に tools/lib/traffic_gen.py が blocker へ決定論展開する (CRN)\n"
    )
    with open(a.out, 'w', encoding='utf-8') as f:
        f.write(header)
        yaml.dump(scenario, f, default_flow_style=None, sort_keys=False,
                  allow_unicode=True, width=100)
    print(f"generated: {a.out}")


if __name__ == '__main__':
    main()
