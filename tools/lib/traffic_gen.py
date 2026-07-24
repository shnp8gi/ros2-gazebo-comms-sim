"""
traffic: セクションの決定論展開 (本番仕様 §3.3)。

シナリオ yaml の `traffic:` 定義 (車線ごとの流量・車種構成) と seed から、
遮蔽車 (role: blocker) のエンティティ列を決定論生成して entities に追記する。
seed は run_idx のみから導出されるため、同一 run なら方式・条件によらず
同一の交通実現になる (CRN)。走行ごとに交通が変わることが σ_ν²(s) 学習の
前提条件 (遮蔽はランダム、リスクの場所は持続)。

責務: 展開のみ。移動は既存 WaypointMoverPlugin、遮蔽計算は既存
BlockageEnvironment の責務であり、ランタイムは本モジュールを知らない。
ホスト直実行 (sweep_sim) のため stdlib のみに依存する。

スキーマ:
  traffic:
    seed: 0                    # derive_run_seeds: false のときに使う固定シード
    window_s: [0.0, 30.0]      # この時間窓にコリドーへ到着する車のみ生成
    corridor: [-180.0, 180.0]  # 遮蔽が意味を持つ道路区間 [m] (x 軸)
    margin_m: 30.0             # コリドー外側の走り抜け余裕
    max_vehicles: 200          # 暴走防止の上限
    tx_entry_jitter_s: 0.0     # 対象車 (role: tx) の進入タイミングジッタ σ [s]
    vehicle_mix:               # 車種構成 (model_catalog に blockage 属性必須)
      - { model: SedanBlocker, ratio: 0.75 }
      - { model: TruckBlocker, ratio: 0.15 }
      - { model: BusBlocker,   ratio: 0.10 }
    lanes:
      - y: 2.5                 # 車線の横位置 [m]
        direction: 1           # +1 = +x (並走) / -1 = -x (対向)
        speed_mps: 5.0
        headway_s: { dist: lognormal, mean: 4.0, sigma: 0.6, min: 1.5 }
"""
import math
import random


def _draw_headway(rng, spec):
    """車頭時間 [s] を1つ引く。dist: exponential | lognormal | uniform | fixed"""
    dist = spec.get('dist', 'exponential')
    mean = float(spec.get('mean', 6.0))
    if dist == 'exponential':
        h = rng.expovariate(1.0 / mean) if mean > 0 else 0.0
    elif dist == 'lognormal':
        sigma = float(spec.get('sigma', 0.6))
        mu = math.log(mean) - 0.5 * sigma * sigma  # E[h] = mean になる正規化
        h = rng.lognormvariate(mu, sigma)
    elif dist == 'uniform':
        h = rng.uniform(float(spec.get('min', 1.0)), float(spec.get('max', mean * 2)))
    elif dist == 'fixed':
        h = float(spec.get('value', mean))
    else:
        raise ValueError(f"traffic: 未知の headway 分布 '{dist}'")
    return max(h, float(spec.get('min', 0.0)))


def _pick_model(rng, mix):
    """車種構成から1台の model 名を引く (比率は正規化して扱う)。"""
    total = sum(float(m.get('ratio', 0.0)) for m in mix)
    x = rng.uniform(0.0, total)
    acc = 0.0
    for m in mix:
        acc += float(m.get('ratio', 0.0))
        if x <= acc:
            return m['model']
    return mix[-1]['model']


def expand_traffic(scenario, seed=None):
    """scenario 辞書 (トップに 'scenario' キーがあってもよい) を in-place 展開する。

    Args:
        scenario: load_scenario() の戻り値。
        seed: 展開シード (run_idx 由来)。None なら traffic.seed (既定 0) を使う。
    Returns:
        追加した blocker エンティティ数 (traffic: が無ければ 0)。
    """
    sc = scenario.get('scenario', scenario)
    traffic = sc.get('traffic')
    if not traffic:
        return 0

    rng = random.Random(int(seed) if seed is not None else int(traffic.get('seed', 0)))
    t0, t1 = [float(v) for v in traffic.get('window_s', [0.0, 30.0])]
    c0, c1 = [float(v) for v in traffic.get('corridor', [-180.0, 180.0])]
    margin = float(traffic.get('margin_m', 30.0))
    max_vehicles = int(traffic.get('max_vehicles', 200))
    mix = traffic.get('vehicle_mix', [])
    if not mix:
        raise ValueError("traffic: vehicle_mix が空")

    entities = sc.setdefault('entities', [])
    existing = {e.get('name') for e in entities}
    catalog = sc.get('model_catalog', {})
    for m in mix:
        if m['model'] not in catalog:
            raise ValueError(f"traffic: model_catalog に {m['model']} が無い")
        if 'blockage' not in catalog[m['model']]:
            raise ValueError(f"traffic: {m['model']} に blockage 属性が無い "
                             f"(遮蔽体として機能しない)")

    added = 0
    for li, lane in enumerate(traffic.get('lanes', [])):
        y = float(lane['y'])
        v = float(lane['speed_mps'])
        direction = 1 if int(lane.get('direction', 1)) >= 0 else -1
        hw = lane.get('headway_s', {'dist': 'exponential', 'mean': 6.0})

        # 到着時刻列 (コリドー入口の通過時刻)。初回はランダム位相
        t = t0 + _draw_headway(rng, hw)
        k = 0
        while t <= t1 and added < max_vehicles:
            model = _pick_model(rng, mix)
            name = f"tf{li}_{k:03d}"
            if name in existing:
                raise ValueError(f"traffic: エンティティ名衝突 {name}")
            # 到着時刻 t を「sim 開始時に入口の v·t 手前に置く」ことで実現する
            # (スポーン時刻の機構は不要。既存 WaypointMoverPlugin が等速で運ぶ)
            if direction > 0:
                x_start, x_end, yaw = c0 - v * t, c1 + margin, 0.0
            else:
                x_start, x_end, yaw = c1 + v * t, c0 - margin, math.pi
            entities.append({
                'name': name,
                'model': model,
                'role': 'blocker',
                'pose': [x_start, y, 0.0, 0.0, 0.0, yaw],
                'waypoints': [[x_start, y, 0.0, v], [x_end, y, 0.0, v]],
            })
            existing.add(name)
            added += 1
            k += 1
            t += _draw_headway(rng, hw)

    # 対象車 (role: tx) の進入タイミングジッタ: 開始位置を進行方向に沿って
    # ±v·N(0, σ) ずらす (等速なので位置ずれ = 時間ずれ)
    jitter_s = float(traffic.get('tx_entry_jitter_s', 0.0))
    if jitter_s > 0.0:
        for e in sc.get('entities', []):
            wps = e.get('waypoints')
            if e.get('role') != 'tx' or not wps or len(wps) < 2:
                continue
            v0 = float(wps[0][3]) if len(wps[0]) >= 4 else 0.0
            dirx = 1.0 if float(wps[-1][0]) >= float(wps[0][0]) else -1.0
            delta = -dirx * v0 * rng.gauss(0.0, jitter_s)
            if 'pose' in e:
                e['pose'][0] = float(e['pose'][0]) + delta
            for wp in wps:
                wp[0] = float(wp[0]) + delta

    return added
