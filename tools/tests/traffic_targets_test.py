#!/usr/bin/env python3
"""
交通流への対象車混在 (都市部2車線仕様 §2.1) の単体テスト。
既存の traffic_gen_test.py は後方互換 (対象車なし) を守る役割で、こちらは新機能を見る。

  python3 tools/tests/traffic_targets_test.py
"""
import copy
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

import yaml  # noqa: E402
from lib import traffic_gen  # noqa: E402
from lib.scenario_loader import generate_sim_params  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SCENARIO = os.path.join(REPO, 'config', 'scenarios', 'road_urban_2lane.yaml')

failures = []


def check(name, cond, detail=""):
    if cond:
        print(f"  OK   {name}")
    else:
        print(f"  FAIL {name} {detail}")
        failures.append(name)


def scen():
    with open(SCENARIO, encoding='utf-8') as f:
        return yaml.safe_load(f)


def by_role(s, role):
    return [e for e in s['scenario']['entities'] if e.get('role') == role]


# --- 1. 決定論性 (CRN) ------------------------------------------------------
print("[1] 決定論性")

a, b = scen(), scen()
na, nb = traffic_gen.expand_traffic(a, 4242), traffic_gen.expand_traffic(b, 4242)
check("同一シードで台数一致", na == nb, f"{na} vs {nb}")
check("同一シードでエンティティ列が完全一致",
      a['scenario']['entities'] == b['scenario']['entities'])

c = scen()
traffic_gen.expand_traffic(c, 4243)
check("シードが違えば実現が変わる",
      a['scenario']['entities'] != c['scenario']['entities'])

check("対象車が生成されている", len(by_role(a, 'tx')) > 0, str(len(by_role(a, 'tx'))))
check("一般車が生成されている", len(by_role(a, 'blocker')) > 0)


# --- 2. 到着時刻インターリーブ (先着順バイアスの除去) -------------------------
print("\n[2] エンティティ列が到着時刻順")


def arrival_of(e, traffic):
    """配置式 x_start = c0 − v·t (下り) / c1 + v·t (上り) の逆算"""
    c0, c1 = traffic['corridor']
    y = e['pose'][1]
    lane = next(l for l in traffic['lanes'] if abs(l['y'] - y) < 1e-9)
    v = lane['speed_mps']
    if lane['direction'] > 0:
        return (c0 - e['pose'][0]) / v
    return (e['pose'][0] - c1) / v


tr = a['scenario']['traffic']
gen_entities = [e for e in a['scenario']['entities']
                if e.get('role') in ('tx', 'blocker')]
arrivals = [arrival_of(e, tr) for e in gen_entities]
check("到着時刻が単調非減少 (interleave_arrivals)",
      all(arrivals[i] <= arrivals[i + 1] + 1e-9 for i in range(len(arrivals) - 1)))

# 両車線の車が交ざっていること (片方に固まっていない)
first_half = {e['pose'][1] > 0 for e in gen_entities[:len(gen_entities) // 2]}
check("列の前半に両車線の車が混在", len(first_half) == 2, str(first_half))


# --- 3. prefill (t=0 でコリドーが埋まっている) -------------------------------
print("\n[3] prefill")

c0, c1 = tr['corridor']
inside = [e for e in gen_entities if c0 <= e['pose'][0] <= c1]
check("t=0 でコリドー内に車がいる (定常状態から開始)", len(inside) > 0, str(len(inside)))
check("負の到着時刻が存在する", min(arrivals) < 0.0, f"min={min(arrivals):.2f}")
check("コリドー出口を越えた車は生成されない",
      all(c0 <= e['pose'][0] <= c1 + 1e-6 or arrival_of(e, tr) >= 0
          for e in gen_entities))

# prefill を切ると t=0 のコリドーは空になる (従来挙動)
d = scen()
d['scenario']['traffic']['prefill'] = 0.0
traffic_gen.expand_traffic(d, 4242)
d_ent = [e for e in d['scenario']['entities'] if e.get('role') in ('tx', 'blocker')]
check("prefill=0 では t=0 のコリドーが空 (従来挙動)",
      not any(c0 <= e['pose'][0] <= c1 for e in d_ent))


# --- 4. 車種構成と対象車比率 ------------------------------------------------
print("\n[4] 構成比 (多数シードの集計)")

counts = {}
n_target = n_total = 0
for seed in range(120):
    s = scen()
    traffic_gen.expand_traffic(s, seed)
    for e in [x for x in s['scenario']['entities'] if x.get('role') in ('tx', 'blocker')]:
        n_total += 1
        if e.get('role') == 'tx':
            n_target += 1
            counts['_target_sedan'] = counts.get('_target_sedan', 0) + 1
        else:
            counts[e['model']] = counts.get(e['model'], 0) + 1

target_frac = n_target / n_total
print(f"  n={n_total}, 対象車 {target_frac:.3f} (設定 0.30)")
check("対象車比率が target_ratio に収束", abs(target_frac - 0.30) < 0.03,
      f"{target_frac:.3f}")

# 全交通に占める大型車比率 (対象車は乗用車なので分母に入る)
truck = counts.get('TruckBlocker', 0) / n_total
bus = counts.get('BusBlocker', 0) / n_total
print(f"  トラック {truck:.3f} (0.15) / バス {bus:.3f} (0.05)")
check("トラック比率が構成比どおり", abs(truck - 0.15) < 0.02, f"{truck:.3f}")
check("バス比率が構成比どおり", abs(bus - 0.05) < 0.02, f"{bus:.3f}")
check("大型車は対象車にならない",
      all(e['model'] == 'Car' for s_ in [a] for e in by_role(s_, 'tx')))

# target_ratio が対象可能車種の総量を超えたらエラー
e_scen = scen()
e_scen['scenario']['traffic']['target_ratio'] = 0.95
try:
    traffic_gen.expand_traffic(e_scen, 1)
    check("対象可能車種を超える target_ratio がエラー", False, "例外が出ない")
except ValueError:
    check("対象可能車種を超える target_ratio がエラー", True)


# --- 5. 上り車線の幾何 ------------------------------------------------------
print("\n[5] 上り車線の幾何")

up = [e for e in by_role(a, 'tx') if e['pose'][1] < 0]
down = [e for e in by_role(a, 'tx') if e['pose'][1] > 0]
check("両車線に対象車がいる", len(up) > 0 and len(down) > 0,
      f"up={len(up)} down={len(down)}")

check("上り車の車体 yaw = π", all(abs(e['pose'][5] - math.pi) < 1e-9 for e in up))
check("下り車の車体 yaw = 0", all(abs(e['pose'][5]) < 1e-9 for e in down))
check("上り車は −x へ進む", all(e['waypoints'][-1][0] < e['waypoints'][0][0] for e in up))
check("下り車は +x へ進む", all(e['waypoints'][-1][0] > e['waypoints'][0][0] for e in down))

check("上り車のアンテナ relative yaw が負",
      all(e['antennas'][0]['relative_rpy'][2] < 0 for e in up))
check("下り車のアンテナ relative yaw が正",
      all(e['antennas'][0]['relative_rpy'][2] > 0 for e in down))

# world 座標のボアサイトが両方向とも RSU 側 (+y) を向く
for e in by_role(a, 'tx'):
    world_yaw = e['pose'][5] + e['antennas'][0]['relative_rpy'][2]
    if math.sin(world_yaw) <= 0:
        check(f"{e['name']} のボアサイトが RSU 側を向く", False,
              f"world_yaw={math.degrees(world_yaw):.1f}°")
        break
else:
    check("全対象車のボアサイトが world +y (RSU側) を向く", True)


# --- 6. scenario_loader との接続 --------------------------------------------
print("\n[6] scenario_loader (対象車が vehicles と blockers の両方に載る)")

loaded_src = scen()
traffic_gen.expand_traffic(loaded_src, 4242)
cfg = generate_sim_params(copy.deepcopy(loaded_src))

veh_names = {v['name'] for v in cfg.get('vehicles', [])}
blk_names = set(cfg.get('blockers', {}).keys())
tx_names = {e['name'] for e in by_role(loaded_src, 'tx')}
bl_names = {e['name'] for e in by_role(loaded_src, 'blocker')}

check("対象車が vehicles に載る", tx_names <= veh_names,
      str(sorted(tx_names - veh_names)[:3]))
check("対象車が blockers にも載る", tx_names <= blk_names,
      str(sorted(tx_names - blk_names)[:3]))
check("一般車は blockers に載る", bl_names <= blk_names)
check("一般車は vehicles に載らない", not (bl_names & veh_names))
check("RSU は blockers に載らない", not ({'rsu_0', 'rsu_1'} & blk_names))

tx_geo = cfg['blockers'][sorted(tx_names)[0]]
check("対象車の遮蔽サイズが catalog どおり (乗用車 1.5m)",
      abs(tx_geo['size'][2] - 1.5) < 1e-9, str(tx_geo))

# 無効化した対象車は blockers からも消える
disabled = copy.deepcopy(loaded_src)
victim = sorted(tx_names)[0]
for e in disabled['scenario']['entities']:
    if e['name'] == victim:
        e['enabled'] = False
cfg2 = generate_sim_params(disabled)
check("enabled: false の対象車は blockers からも消える",
      victim not in cfg2.get('blockers', {}))


# --- 7. 後方互換 (road_10car は対象車を生成しない) ---------------------------
print("\n[7] 後方互換")

old_path = os.path.join(REPO, 'config', 'scenarios', 'road_10car.yaml')
with open(old_path, encoding='utf-8') as f:
    old = yaml.safe_load(f)
n_old_tx_before = len(by_role(old, 'tx'))
traffic_gen.expand_traffic(old, 4242)
check("road_10car の tx は固定10台のまま (交通から生成されない)",
      len(by_role(old, 'tx')) == n_old_tx_before == 10,
      f"{len(by_role(old, 'tx'))}")
check("road_10car の生成物は blocker のみ",
      all(e['name'].startswith('tf') for e in by_role(old, 'blocker')))


print("\n" + "=" * 60)
if failures:
    print(f"FAILED: {len(failures)} 件 — {failures}")
    sys.exit(1)
print("ALL PASSED")
