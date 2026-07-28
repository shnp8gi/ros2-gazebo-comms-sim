#!/usr/bin/env python3
"""
都市部2車線シナリオのパラメトリック生成 (仕様 docs/urban_2lane_scenario_spec.md §1)。

責務: 幾何パラメータ → scenario yaml の決定論生成のみ。
生成物はチェックインし、**真実は生成された yaml** とする (評価ディレクトリ契約)。

road_10car (gen_road_scenario.py) との違い:
  - 対象車を **生成しない**。対象車は一般車と同じ交通流から tools/lib/traffic_gen.py が
    run 毎に展開する (仕様 §2.1)。ここが「対象車と一般車の混在」の実体。
  - 道路断面 (車線数・車線幅・路肩・歩道) から RSU の横位置を積み上げる (仕様 §1.1)。
  - 対向車線に対応するため、車載アンテナの relative yaw を走行方向で反転する (§1.4)。

gen_road_scenario.py は road_10car の再現性契約を持つため **変更しない**。
共通する幾何計算は tools/lib/road_geometry.py に純関数として置いた。

使い方:
  python3 tools/gen_urban_scenario.py --out config/scenarios/road_urban_2lane.yaml
"""
import argparse
import os
import sys

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from lib.road_geometry import (  # noqa: E402
    lane_layout, rsu_positions, rsu_tilts_deg, rsu_y_from_section, rsu_yaw_rad,
    veh_relative_yaw_rad, veh_tilt_for_mutual_boresight,
)


def build_scenario(a):
    # --- 道路断面 ---
    near_ys, far_ys = lane_layout(a.lanes_near, a.lanes_far, a.lane_width)
    rsu_y = (rsu_y_from_section(a.lanes_near, a.lane_width, a.shoulder_m, a.sidewalk_m)
             if a.rsu_y == 'auto' else float(a.rsu_y))

    veh_tilt = (veh_tilt_for_mutual_boresight(a.rsu_tilt_deg)
                if a.veh_tilt_deg == 'auto' else float(a.veh_tilt_deg))

    # --- RSU 列 ---
    rsu_xs = rsu_positions(a.num_rsu, a.rsu_spacing)
    tilts = rsu_tilts_deg(a.num_rsu, a.rsu_tilt_deg, a.rsu_tilt_pattern)

    entities = []
    for i, (x, tilt) in enumerate(zip(rsu_xs, tilts)):
        entities.append({
            'name': f"rsu_{i}",
            'model': 'antenna',
            'role': 'rx',
            'static': True,
            'pose': [round(x, 3), round(rsu_y, 3), 0.0,
                     0.0, 0.0, round(rsu_yaw_rad(tilt), 6)],
            'antennas': [{'name': f"rsu_{i}",
                          'offset': [0.0, 0.0, float(a.rsu_h)],
                          'relative_rpy': [0.0, 0.0, 0.0]}],
        })

    # --- 交通が意味を持つ区間 ---
    c0 = rsu_xs[0] - a.corridor_margin_m
    c1 = rsu_xs[-1] + a.corridor_margin_m

    # 走行間固定シャドウ場はコリドー + 余裕をカバーすれば足りる
    # (待機中の車はゲートされ通信しないため、場を staging 区間まで広げない)
    u_min, u_max = c0 - 20.0, c1 + 20.0

    lanes = []
    for y in near_ys:
        lanes.append({'y': round(y, 3), 'direction': 1, 'speed_mps': a.speed,
                      'headway_s': {'dist': 'lognormal', 'mean': a.headway_mean_s,
                                    'sigma': 0.6, 'min': 1.0}})
    for y in far_ys:
        lanes.append({'y': round(y, 3), 'direction': -1, 'speed_mps': a.speed,
                      'headway_s': {'dist': 'lognormal', 'mean': a.headway_mean_s,
                                    'sigma': 0.6, 'min': 1.0}})

    # 対象車のアンテナ諸元。traffic_gen は幾何を知らず、これを引き写すだけ (§2.1)
    target_template = {
        'model': 'Car',
        'antenna_offset': [0.0, 0.0, float(a.ant_h)],
        'relative_yaw_by_direction': {
            '1': round(veh_relative_yaw_rad(1, veh_tilt), 6),
            '-1': round(veh_relative_yaw_rad(-1, veh_tilt), 6),
        },
    }

    scenario = {'scenario': {
        'name': a.name,
        'description': (
            f"都市部 片側{a.lanes_near}車線×{a.lanes_far}車線・"
            f"対象車と一般車の混在交通 (target_ratio で分離)。"
            f"RSU {a.num_rsu}台 {a.rsu_spacing}m間隔 y={rsu_y:.2f} (歩道外側端)、"
            f"傾き ±{a.rsu_tilt_deg}° ({a.rsu_tilt_pattern}) / 車 ±{veh_tilt}°。"
            f"δ_rsu+δ_veh={a.rsu_tilt_deg + veh_tilt:.0f}° "
            f"({'相互ボアサイト成立' if abs(a.rsu_tilt_deg + veh_tilt - 90) < 1e-6 else '対向不成立'})。"
            "tools/gen_urban_scenario.py により生成"),
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
                    # 既定は提案手法。arm ごとの上書きは sweep の cases が行う
                    'scheduling_policy': 'external_schedule',
                    'control_plane': 'kkf_mpc',
                    'schedule_topic': '/comms/ho_schedule',
                    'ff_max_pairs': 1,
                    # assoc_hold (802.15.3e 準拠・HOなし) のリンク監視回復待機時間 [s]。
                    # 単一値だが sweep の cases で掃引可能
                    'assoc_recover_timeout_s': a.assoc_recover_timeout_s,
                    'export_rssi_profile': True,
                    'kkf_use_prior': False,
                    'kkf_assigner': 'hungarian',
                    'kkf_switch_bonus_db': 3.0,
                    # 再計画はレポート周期 (0.05s) に合わせる。接続窓が ~1s しか
                    # ないため 0.2s では窓の 2 割を取りこぼす。推定 (KF更新+
                    # クリギング) はレポート毎に走るので周期とは独立
                    'kkf_replan_period_s': a.kkf_replan_period_s,
                    # 先読み: K=1 で瞬時最適 (先読みなし)、K>1 で窓の出入りを見る。
                    # arm (kkf_k0 / kkf_mpc) は sweep がこの値を上書きして作る
                    'kkf_lookahead_stages': a.kkf_lookahead_stages,
                    'kkf_lookahead_discount': 1.0,
                    # 連続交通流ではコリドーを出た車が計画に残り続けるため必須
                    'kkf_stale_report_s': 1.0,
                    # REM を分ける方向の集合 (道路の車線構成から決まる環境の性質)。
                    # 対象車は確率生成なので run によっては片方向しか出ないことが
                    # あり、車両から推定すると run ごとに状態キーが変わって学習が
                    # 蓄積しない。ここで明示して固定する
                    'kkf_directions': sorted({lane['direction'] for lane in lanes}),
                    'kkf_horizon_s': 4.0,
                    'kkf_plan_dt_s': a.kkf_plan_dt_s,
                    'kkf_kappa': 1.0,
                    'kkf_idle_lcb_db': -110.0,
                    'kkf_basis': 'rbf',
                    'kkf_rbf_num_bases': 20,
                    'kkf_rbf_width_m': 0.0,
                    'kkf_sigma_nu_db': 4.0,
                    'kkf_corr_length_s_m': 20.0,
                    'kkf_corr_length_t_s': 5.0,
                    'kkf_varmap_enabled': True,
                    'kkf_varmap_grid_m': 2.0,
                    'kkf_varmap_prior_var_db2': 0.0,
                    'kkf_state_dir': '',
                    'kkf_state_save': False,
                    'kkf_q_forget': 0.05,
                    'kkf_state_gamma': 0.997,
                    'kkf_tracker_enabled': False,
                    'tskf_process_noise_q': 25.0,
                    'tskf_prior_mean_dbm': -120.0,
                    'tskf_prior_var': 400.0,
                },
            },
            'comms_simulator_node': {
                'ros__parameters': {
                    'tx_power': a.tx_power,
                    'comms_update_period_s': a.comms_update_period_s,
                    # コリドーゲーティング (0 = 無効)。連続交通流では待機中の車も
                    # 全 tick で全 RSU 分を評価するため、圏外確定のペアを落とす。
                    # 閾値 -68.5dBm の到達距離 (両端ボアサイトで ~65m) より十分
                    # 大きく取ること (でないと情報上界 oracle_inst が壊れる)
                    'link_eval_radius_m': a.link_eval_radius_m,
                    # snr_min_db は接続閾値そのもの (仕様 §3.1)。
                    #   rssi_min = noise_floor + snr_min_db = -95 + 26.5 = -68.5 dBm
                    # = MCS テーブル下限と等価。greedy_fcfs の占有時間を決める最重要定数
                    'rate_model': {
                        'type': 'shannon',
                        'bandwidth_hz': 100.0e6,
                        'noise_floor_dbm': -95.0,
                        'efficiency': 1.0,
                        'snr_min_db': a.snr_min_db,
                        'snr_cap_db': 50.0,
                    },
                    'channel': {
                        'seed': 42,  # run 毎に注入
                        'blockage': {'enabled': True, 'max_total_loss_db': 60.0},
                        'shadowing': {'enabled': True, 'type': 'frozen',
                                      'sigma_db': 4.0, 'corr_length_m': 6.0,
                                      'environment_seed': a.environment_seed,
                                      'grid_m': 1.5, 'axis': 'x',
                                      'u_min': round(u_min, 1),
                                      'u_max': round(u_max, 1)},
                        'fading': {'enabled': False},
                    },
                    'measurement_report': {
                        'enabled': True,
                        'period_s': 0.05,
                        'noise_std_db': 2.0,
                        'observe_all_pairs': False,  # arm が sweep で上書き
                        'seed': 123,
                    },
                },
            },
        },
        'model_catalog': {
            'antenna': {
                'sdf_path': 'models://antenna',
                'description': '沿道RSUアンテナ (歩道外側端のポール)',
                'default_antenna_offset': [0.0, 0.0, float(a.rsu_h)],
            },
            # 対象車。collision なし (GhostBox) — 混在交通では台数が増えるため
            # 接触ソルバに載せない。blockage 属性があるので他車の視線は遮る
            'Car': {
                'sdf_path': 'models://GhostBox',
                'description': '配信対象の乗用車 (車高1.5m)',
                'default_antenna_offset': [0.0, 0.0, float(a.ant_h)],
                'blockage': {'size': [4.5, 1.8, 1.5], 'loss_db': 8.0},
            },
            # 一般車 (仕様 §2.2)。乗用車は視線 (1.74〜2.05m) の下を通り遮蔽しない
            'SedanBlocker': {
                'sdf_path': 'models://GhostBox',
                'description': '乗用車 (車高1.5m、視線下=遮蔽せず)',
                'blockage': {'size': [4.5, 1.8, 1.5], 'loss_db': 8.0},
            },
            # ミニバン/SUV: 既定 ratio 0。yaml の比率変更だけで仕様 §1.2 の (b) 案へ移れる
            'MinivanBlocker': {
                'sdf_path': 'models://GhostBox',
                'description': 'ミニバン・SUV (車高1.9m、視線を遮る境界クラス)',
                'blockage': {'size': [4.8, 1.8, 1.9], 'loss_db': 12.0},
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
        'traffic': {
            'seed': 0,
            'window_s': [0.0, a.window_s],
            'warmup_s': a.warmup_s,
            'prefill': 'auto',       # auto = (コリドー長 + margin) / speed
            # 先着順ポリシー (greedy_fcfs) の「先着」は gz-sim のエンティティ
            # 実行順で決まるため、車線ごとに積むと常に近車線が勝つ (仕様 §2.1)
            'interleave_arrivals': True,
            'corridor': [round(c0, 1), round(c1, 1)],
            'margin_m': 30.0,
            'max_vehicles': 400,
            'target_ratio': a.target_ratio,
            'target_template': target_template,
            'tx_entry_jitter_s': 0.0,
            'vehicle_mix': [
                {'model': 'SedanBlocker', 'ratio': 0.80, 'can_be_target': True},
                {'model': 'MinivanBlocker', 'ratio': 0.00, 'can_be_target': True},
                {'model': 'TruckBlocker', 'ratio': 0.15, 'can_be_target': False},
                {'model': 'BusBlocker', 'ratio': 0.05, 'can_be_target': False},
            ],
            'lanes': lanes,
        },
        'entities': entities,
    }}
    return scenario


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', required=True)
    ap.add_argument('--name', default='road_urban_2lane')
    # 道路断面 (仕様 §1.1)
    ap.add_argument('--lanes-near', type=int, default=1, help='RSU側 (下り) の車線数')
    ap.add_argument('--lanes-far', type=int, default=1, help='対向 (上り) の車線数')
    ap.add_argument('--lane-width', type=float, default=3.5)
    ap.add_argument('--shoulder-m', type=float, default=0.5)
    ap.add_argument('--sidewalk-m', type=float, default=2.0)
    ap.add_argument('--rsu-y', default='auto',
                    help="RSU 横位置 [m]。auto = 断面から積み上げ。"
                         "車線数を掃引する際は固定値を明示指定すること (交絡回避)")
    ap.add_argument('--rsu-h', type=float, default=2.5,
                    help='RSU アンテナ高 [m]【実測】。仕様 §1.2 で 3.5〜4.0 を推奨')
    # RSU 配置と角度 (仕様 §1.3)
    ap.add_argument('--num-rsu', type=int, default=4)
    ap.add_argument('--rsu-spacing', type=float, default=10.0)
    ap.add_argument('--rsu-tilt-deg', type=float, default=45.0)
    ap.add_argument('--rsu-tilt-pattern', default='alternate',
                    choices=['alternate', 'grouped'])
    ap.add_argument('--veh-tilt-deg', default='auto',
                    help='auto = 90 − rsu_tilt (相互ボアサイト対向)')
    ap.add_argument('--ant-h', type=float, default=1.35, help='車載アンテナ高 [m]【実測】')
    ap.add_argument('--corridor-margin-m', type=float, default=40.0)
    # 交通 (仕様 §2)
    ap.add_argument('--speed', type=float, default=16.7, help='車速 [m/s] (60km/h)')
    ap.add_argument('--headway-mean-s', type=float, default=5.0)
    ap.add_argument('--target-ratio', type=float, default=0.30)
    ap.add_argument('--window-s', type=float, default=60.0, help='計測窓 [s]')
    ap.add_argument('--warmup-s', type=float, default=5.0)
    # 無線 (仕様 §3)
    ap.add_argument('--tx-power', type=float, default=-7.0)
    ap.add_argument('--snr-min-db', type=float, default=26.5,
                    help='rssi_min = -95 + これ。既定 26.5 → -68.5 dBm (MCS表下限相当)')
    ap.add_argument('--environment-seed', type=int, default=1)
    ap.add_argument('--rtf', type=float, default=2.0)
    ap.add_argument('--comms-update-period-s', type=float, default=0.005)
    ap.add_argument('--link-eval-radius-m', type=float, default=100.0,
                    help='この距離を超えるリンクは評価しない (0 = 無効)。'
                         '閾値の到達距離 (~65m) より十分大きく取ること')
    ap.add_argument('--assoc-recover-timeout-s', type=float, default=1.0,
                    help='assoc_hold のリンク監視回復待機時間 [s] (掃引可)')
    ap.add_argument('--kkf-replan-period-s', type=float, default=0.05,
                    help='KKF 再計画周期 [s] (レポート周期に合わせるのが上限)')
    ap.add_argument('--kkf-lookahead-stages', type=int, default=1,
                    help='先読み段数 K (1 = 瞬時最適)。arm ごとに sweep が上書き')
    ap.add_argument('--kkf-plan-dt-s', type=float, default=0.1,
                    help='先読みのステージ幅 [s] (K×これ = ホライズン長)')
    a = ap.parse_args()

    scenario = build_scenario(a)
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    header = (
        "# 都市部2車線シナリオ (仕様 docs/urban_2lane_scenario_spec.md)。\n"
        "# tools/gen_urban_scenario.py により生成 — 手編集より再生成を推奨:\n"
        f"#   python3 tools/gen_urban_scenario.py --out {a.out}\n"
        "# 角度規約: RSU yaw = -90°+δ_rsu (下流へ) / 車 relative yaw = ±δ_veh (走行方向で反転)\n"
        "#   δ_rsu < 0 (上流へ振る) = 下り車を担当 / δ_rsu > 0 = 上り車を担当\n"
        "# traffic: は run 毎に tools/lib/traffic_gen.py が対象車(tx)と一般車(blocker)へ展開する\n"
    )
    with open(a.out, 'w', encoding='utf-8') as f:
        f.write(header)
        yaml.dump(scenario, f, default_flow_style=None, sort_keys=False,
                  allow_unicode=True, width=100)
    print(f"generated: {a.out}")


if __name__ == '__main__':
    main()
