#!/usr/bin/env python3
"""tools/lib/road_geometry.py の単体テスト (仕様 §1)。

  python3 tools/tests/road_geometry_test.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from lib.road_geometry import (  # noqa: E402
    blocks_los, boresight_meet_dx, lane_layout, los_height_at_y,
    mutual_boresight_pose, rsu_boresight_xy, rsu_positions, rsu_tilts_deg,
    rsu_y_from_section, serving_direction, veh_boresight_xy,
    veh_relative_yaw_rad, veh_tilt_for_mutual_boresight,
)

failures = []


def check(name, cond, detail=""):
    if cond:
        print(f"  OK   {name}")
    else:
        print(f"  FAIL {name} {detail}")
        failures.append(name)


def close(a, b, tol=1e-9):
    return abs(a - b) <= tol


# --- 1. 道路断面 ------------------------------------------------------------
print("[1] 道路断面")

near, far = lane_layout(1, 1, 3.5)
check("1+1 の車線 y", near == [1.75] and far == [-1.75], f"{near} {far}")

near2, far2 = lane_layout(2, 2, 3.5)
check("2+2 の車線 y", near2 == [1.75, 5.25] and far2 == [-1.75, -5.25],
      f"{near2} {far2}")

check("RSU y (1+1) = 6.0", close(rsu_y_from_section(1, 3.5, 0.5, 2.0), 6.0))
check("RSU y (2+2) = 9.5", close(rsu_y_from_section(2, 3.5, 0.5, 2.0), 9.5))

check("RSU x 列 (4台10m間隔)", rsu_positions(4, 10.0) == [-15.0, -5.0, 5.0, 15.0],
      str(rsu_positions(4, 10.0)))

check("alternate tilt", rsu_tilts_deg(4, 45.0, 'alternate') == [-45.0, 45.0, -45.0, 45.0])
check("grouped tilt", rsu_tilts_deg(4, 45.0, 'grouped') == [-45.0, -45.0, 45.0, 45.0])


# --- 2. 相互ボアサイト対向 --------------------------------------------------
print("\n[2] 相互ボアサイト対向 (δ_rsu + δ_veh = 90°)")

check("δ_veh = 90 − δ_rsu", close(veh_tilt_for_mutual_boresight(45.0), 45.0))
check("δ_rsu=30 → δ_veh=60", close(veh_tilt_for_mutual_boresight(30.0), 60.0))

RSU_Y, RSU_H = 6.0, 2.5
VEH_H = 1.35


def boresight_error_deg(rsu_x, rsu_tilt, veh_x, veh_y, veh_tilt, direction):
    """RSU→車 / 車→RSU の水平ベクトルと、それぞれのボアサイトの角度差 [deg]。"""
    dx, dy = veh_x - rsu_x, veh_y - RSU_Y
    bx, by = rsu_boresight_xy(rsu_tilt)
    err_rsu = abs(math.degrees(math.atan2(dy, dx) - math.atan2(by, bx)))

    vx, vy = rsu_x - veh_x, RSU_Y - veh_y
    cx, cy = veh_boresight_xy(direction, veh_tilt)
    err_veh = abs(math.degrees(math.atan2(vy, vx) - math.atan2(cy, cx)))
    return err_rsu, err_veh


for tilt in (30.0, 45.0, 52.0, 60.0):
    for lane_y in (-1.75, 1.75):
        veh_x, veh_tilt, direction = mutual_boresight_pose(0.0, RSU_Y,
                                                           tilt if lane_y < 0 else -tilt,
                                                           lane_y)
        e_rsu, e_veh = boresight_error_deg(
            0.0, tilt if lane_y < 0 else -tilt, veh_x, lane_y, veh_tilt, direction)
        lane = "奥" if lane_y < 0 else "近"
        check(f"δ={tilt}° {lane}車線 で双方が対向 (誤差<1e-6°)",
              e_rsu < 1e-6 and e_veh < 1e-6, f"rsu={e_rsu:.2e} veh={e_veh:.2e}")

# 対向位置の大きさ
check("Δx = L·tan δ (奥車線 L=7.75, δ=45)",
      close(boresight_meet_dx(7.75, 45.0), 7.75, 1e-9))
check("Δx = L·tan δ (近車線 L=4.25, δ=45)",
      close(boresight_meet_dx(4.25, 45.0), 4.25, 1e-9))

# tilt 符号が「どちらの走行方向に対向を与えるか」を決める
check("上流へ振った RSU (−δ) は下り車 (+1) を捕まえる", serving_direction(-45.0) == 1)
check("下流へ振った RSU (+δ) は上り車 (−1) を捕まえる", serving_direction(45.0) == -1)

# alternate パターンで両方向が同数のRSUに割り当たる
tilts = rsu_tilts_deg(4, 45.0, 'alternate')
dirs = [serving_direction(t) for t in tilts]
check("4台の alternate で上り下りが2台ずつ",
      dirs.count(1) == 2 and dirs.count(-1) == 2, str(dirs))


# --- 3. 車載アンテナの向き (方向別の符号反転) --------------------------------
print("\n[3] 車載アンテナ relative yaw の符号")

check("下り車は +δ", close(veh_relative_yaw_rad(1, 45.0), math.radians(45.0)))
check("上り車は −δ", close(veh_relative_yaw_rad(-1, 45.0), math.radians(-45.0)))

# world 座標では両方向とも RSU 側 (+y) を向く
for direction in (1, -1):
    _, by = veh_boresight_xy(direction, 45.0)
    check(f"direction={direction:+d} のボアサイトが world +y (RSU側) を向く", by > 0,
          f"by={by:.3f}")


# --- 4. 視線高さ (仕様 §1.2 の表) -------------------------------------------
print("\n[4] 視線高さと遮蔽判定 (RSU y=6.0 h=2.5, 車 y=−1.75 h=1.35)")

FAR_Y = -1.75
h_at = lambda y: los_height_at_y(y, RSU_Y, RSU_H, FAR_Y, VEH_H)

check("y=3.0 で 2.05 m", close(h_at(3.0), 1.35 + (4.75 / 7.75) * 1.15, 1e-12),
      f"{h_at(3.0):.4f}")
check("y=1.75 で 1.87 m", close(h_at(1.75), 1.35 + (3.50 / 7.75) * 1.15, 1e-12),
      f"{h_at(1.75):.4f}")
check("y=0.5 で 1.68 m", close(h_at(0.5), 1.35 + (2.25 / 7.75) * 1.15, 1e-12),
      f"{h_at(0.5):.4f}")
check("視線高は単調 (RSU に近いほど高い)", h_at(3.0) > h_at(1.75) > h_at(0.5))

# 近車線 (y=+1.75) の遮蔽車が奥車線リンクを遮るか
NEAR_Y, W_SMALL, W_LARGE = 1.75, 1.8, 2.5
sedan = blocks_los(1.5, NEAR_Y, W_SMALL, RSU_Y, RSU_H, FAR_Y, VEH_H)
minivan = blocks_los(1.9, NEAR_Y, W_SMALL, RSU_Y, RSU_H, FAR_Y, VEH_H)
bus = blocks_los(3.2, NEAR_Y, W_LARGE, RSU_Y, RSU_H, FAR_Y, VEH_H)
truck = blocks_los(3.8, NEAR_Y, W_LARGE, RSU_Y, RSU_H, FAR_Y, VEH_H)

check("乗用車 1.5m は遮らない", not sedan)
check("バス 3.2m は遮る", bus)
check("トラック 3.8m は遮る", truck)
check("ミニバン 1.9m は遮る (= 乗用車クラスに含めると性格が変わる)", minivan)

# 余裕の薄さを明示的に固定する (仕様 §1.2 の警告)
margin = h_at(NEAR_Y - W_SMALL / 2.0) - 1.5
check("乗用車の余裕が 0.3 m 未満であること (境界すれすれ)", margin < 0.3,
      f"margin={margin:.3f} m")

# RSU を 4.0 m に上げると余裕が十分になる
h_at_high = los_height_at_y(NEAR_Y - W_SMALL / 2.0, RSU_Y, 4.0, FAR_Y, VEH_H)
check("RSU 高 4.0m なら乗用車もミニバンも通す", h_at_high > 1.9,
      f"h={h_at_high:.3f} m")
check("RSU 高 4.0m でもバスは遮る",
      blocks_los(3.2, NEAR_Y, W_LARGE, RSU_Y, 4.0, FAR_Y, VEH_H))


# --- 5. 生成器の出力 --------------------------------------------------------
print("\n[5] gen_urban_scenario.build_scenario の出力")

import argparse  # noqa: E402
from gen_urban_scenario import build_scenario  # noqa: E402

defaults = argparse.Namespace(
    name='t', lanes_near=1, lanes_far=1, lane_width=3.5, shoulder_m=0.5,
    sidewalk_m=2.0, rsu_y='auto', rsu_h=2.5, num_rsu=4, rsu_spacing=10.0,
    rsu_tilt_deg=45.0, rsu_tilt_pattern='alternate', veh_tilt_deg='auto',
    ant_h=1.35, corridor_margin_m=40.0, speed=16.7, headway_mean_s=5.0,
    target_ratio=0.30, window_s=60.0, warmup_s=5.0, tx_power=-7.0,
    snr_min_db=26.5, environment_seed=1, rtf=2.0, comms_update_period_s=0.005,
    link_eval_radius_m=100.0)

sc = build_scenario(defaults)['scenario']
rsus = sc['entities']

check("RSU が 4 台", len(rsus) == 4)
check("RSU x 列", [r['pose'][0] for r in rsus] == [-15.0, -5.0, 5.0, 15.0])
check("RSU y = 6.0", all(close(r['pose'][1], 6.0) for r in rsus))
check("RSU yaw = -135/-45 の交互",
      [round(math.degrees(r['pose'][5])) for r in rsus] == [-135, -45, -135, -45],
      str([round(math.degrees(r['pose'][5])) for r in rsus]))

tr = sc['traffic']
check("車線 y と方向",
      [(l['y'], l['direction']) for l in tr['lanes']] == [(1.75, 1), (-1.75, -1)],
      str([(l['y'], l['direction']) for l in tr['lanes']]))
check("コリドー = RSU列 ± 40m", tr['corridor'] == [-55.0, 55.0], str(tr['corridor']))

tmpl = tr['target_template']['relative_yaw_by_direction']
check("対象車 yaw が方向で符号反転",
      tmpl['1'] > 0 and tmpl['-1'] < 0 and close(tmpl['1'], -tmpl['-1'], 1e-9),
      str(tmpl))
check("対象車 yaw = ±45°", close(math.degrees(tmpl['1']), 45.0, 1e-4))

check("対象車比率が traffic に載る", close(tr['target_ratio'], 0.30))
check("Car に blockage 属性がある (対象車も遮蔽体)",
      'blockage' in sc['model_catalog']['Car'])
check("ミニバンは既定 ratio 0",
      close([m for m in tr['vehicle_mix'] if m['model'] == 'MinivanBlocker'][0]['ratio'], 0.0))
check("大型車は通信対象になれない",
      all(not m['can_be_target'] for m in tr['vehicle_mix']
          if m['model'] in ('TruckBlocker', 'BusBlocker')))

rate = sc['config_overrides']['comms_simulator_node']['ros__parameters']['rate_model']
check("snr_min_db = 26.5 → rssi_min = -68.5 dBm",
      close(rate['noise_floor_dbm'] + rate['snr_min_db'], -68.5, 1e-9),
      f"{rate['noise_floor_dbm'] + rate['snr_min_db']}")

# 車線数を変えたときの断面追従
sc22 = build_scenario(argparse.Namespace(**{**vars(defaults),
                                            'lanes_near': 2, 'lanes_far': 2}))['scenario']
check("2+2 で RSU y = 9.5", close(sc22['entities'][0]['pose'][1], 9.5),
      str(sc22['entities'][0]['pose'][1]))
check("2+2 で車線が 4 本", len(sc22['traffic']['lanes']) == 4)

# 明示指定は auto を上書きする
sc_fixed = build_scenario(argparse.Namespace(**{**vars(defaults),
                                                'lanes_near': 2, 'rsu_y': '8.0'}))['scenario']
check("--rsu-y 明示指定が auto を上書き",
      close(sc_fixed['entities'][0]['pose'][1], 8.0))


print("\n" + "=" * 60)
if failures:
    print(f"FAILED: {len(failures)} 件 — {failures}")
    sys.exit(1)
print("ALL PASSED")
