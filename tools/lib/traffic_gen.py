"""
traffic: セクションの決定論展開 (本番仕様 §3.3 / 都市部2車線仕様 §2.1)。

シナリオ yaml の `traffic:` 定義 (車線ごとの流量・車種構成) と seed から、
交通流を決定論生成して entities に追記する。seed は run_idx のみから導出される
ため、同一 run なら方式・条件によらず同一の交通実現になる (CRN)。

生成するもの:
  - 一般車 (role: blocker) — 遮蔽体としてのみ振る舞う
  - 対象車 (role: tx)      — `target_ratio` を指定したときのみ。一般車と同じ
                             到着列から抽出されるため「交通に紛れた通信対象」になる

責務: 展開のみ。移動は既存 WaypointMoverPlugin / TxControllerPlugin、遮蔽計算は
既存 BlockageEnvironment の責務であり、ランタイムは本モジュールを知らない。
**幾何の知識も持たない** — 対象車のアンテナ諸元は `target_template` として
シナリオ生成側 (gen_urban_scenario.py) が書き込んだものを引き写すだけ。
ホスト直実行 (sweep_sim) のため stdlib のみに依存する。

後方互換: `target_ratio` / `prefill` / `interleave_arrivals` を指定しない
シナリオ (road_10car 等) では**乱数列の消費順が従来と完全に一致する**ため、
過去の交通実現がビット単位で再現される。

スキーマ:
  traffic:
    seed: 0                    # derive_run_seeds: false のときに使う固定シード
    window_s: [0.0, 30.0]      # この時間窓にコリドーへ到着する車を生成
    corridor: [-180.0, 180.0]  # 遮蔽が意味を持つ道路区間 [m] (x 軸)
    margin_m: 30.0             # コリドー外側の走り抜け余裕
    max_vehicles: 200          # 暴走防止の上限
    prefill: auto              # 省略/0 = 従来通り (t=0 のコリドーは空)。
                               # auto = (コリドー長 + margin) / speed 手前から到着列を
                               # 始め、t=0 で既にコリドーが埋まった定常状態にする
    interleave_arrivals: false # true = エンティティ列を到着時刻順に並べ替える。
                               # 先着順ポリシー (greedy_fcfs) の「先着」は gz-sim の
                               # エンティティ実行順で決まるため、車線ごとに積むと
                               # 常に先の車線が勝つ系統バイアスが出る
    target_ratio: 0.0          # 全交通に占める通信対象の割合 (0 = 対象車を生成しない)
    target_template:           # target_ratio > 0 のとき必須
      model: Car
      antenna_offset: [0.0, 0.0, 1.35]
      relative_yaw_by_direction: { "1": 0.785398, "-1": -0.785398 }
    tx_entry_jitter_s: 0.0     # 既存 role: tx の進入タイミングジッタ σ [s]
    vehicle_mix:               # 車種構成 (model_catalog に blockage 属性必須)
      - { model: SedanBlocker, ratio: 0.80, can_be_target: true }
      - { model: TruckBlocker, ratio: 0.15, can_be_target: false }
    lanes:
      - y: 2.5                 # 車線の横位置 [m]
        direction: 1           # +1 = +x / -1 = -x
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


def target_probability(mix, target_ratio):
    """「全交通の target_ratio」を「対象になれる車種の中での条件付き確率」に変換する。

        P(target | eligible) = target_ratio × Σratio / Σratio_eligible

    対象になれる車種の総量が target_ratio に足りない場合はエラー
    (黙って比率を切り詰めると構成比の主張が崩れるため)。
    """
    if target_ratio <= 0.0:
        return 0.0
    total = sum(float(m.get('ratio', 0.0)) for m in mix)
    elig = sum(float(m.get('ratio', 0.0)) for m in mix if m.get('can_be_target'))
    if elig <= 0.0:
        raise ValueError("traffic: target_ratio > 0 だが can_be_target の車種が無い")
    p = target_ratio * total / elig
    if p > 1.0 + 1e-9:
        raise ValueError(
            f"traffic: target_ratio {target_ratio} が対象可能車種の構成比 "
            f"{elig / total:.3f} を超えている (全車を対象にしても足りない)")
    return min(p, 1.0)


def _prefill_seconds(spec, corridor_len, margin_m, speed_mps):
    """prefill: auto | <秒> | 省略 を秒に解決する。"""
    if spec is None:
        return 0.0
    if isinstance(spec, str):
        if spec != 'auto':
            raise ValueError(f"traffic: 未知の prefill 指定 '{spec}'")
        return (corridor_len + margin_m) / speed_mps if speed_mps > 0 else 0.0
    return float(spec)


def expand_traffic(scenario, seed=None):
    """scenario 辞書 (トップに 'scenario' キーがあってもよい) を in-place 展開する。

    Args:
        scenario: load_scenario() の戻り値。
        seed: 展開シード (run_idx 由来)。None なら traffic.seed (既定 0) を使う。
    Returns:
        追加したエンティティ数 (対象車 + 一般車。traffic: が無ければ 0)。
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
        # 遮蔽属性が要るのは「通信対象になりえない車両」だけ。そうした車両は
        # 遮蔽体として存在する意味しかないため、属性が無ければ設定ミスである。
        # 一方、通信対象になりうる車両は需要を作るために存在するので、
        # 遮蔽属性が無くてもよい (固定遮蔽のみを使う構成ではこちらになる)
        if not m.get('can_be_target', False) and 'blockage' not in catalog[m['model']]:
            raise ValueError(f"traffic: {m['model']} は通信対象になりえないのに "
                             f"blockage 属性が無い (存在する意味が無い)")

    # --- 対象車 (role: tx) の設定 ---
    target_ratio = float(traffic.get('target_ratio', 0.0))
    template = traffic.get('target_template')
    if target_ratio > 0.0 and not template:
        raise ValueError("traffic: target_ratio > 0 には target_template が必要")
    if template and template.get('model') not in catalog:
        raise ValueError(f"traffic: model_catalog に "
                         f"{template.get('model')} が無い (target_template)")
    p_target = target_probability(mix, target_ratio)
    eligible = {m['model'] for m in mix if m.get('can_be_target')}

    interleave = bool(traffic.get('interleave_arrivals', False))

    # --- 到着列の生成 ---
    # (t, lane_index, k, model, is_target) を集めてから entities に積む。
    # p_target == 0 のときは is_target 判定の乱数を引かないため、
    # 乱数列の消費順が従来と一致する (road_10car の再現性を保つ)
    draws = []
    n_drawn = 0
    for li, lane in enumerate(traffic.get('lanes', [])):
        v = float(lane['speed_mps'])
        hw = lane.get('headway_s', {'dist': 'exponential', 'mean': 6.0})
        prefill = _prefill_seconds(traffic.get('prefill'), c1 - c0, margin, v)

        t = t0 - prefill + _draw_headway(rng, hw)
        k = 0
        while t <= t1 and n_drawn < max_vehicles:
            model = _pick_model(rng, mix)
            is_target = (p_target > 0.0 and model in eligible
                         and rng.random() < p_target)
            draws.append((t, li, k, model, is_target))
            n_drawn += 1
            k += 1
            t += _draw_headway(rng, hw)

    if interleave:
        # 先着順ポリシーの系統バイアス除去 (仕様 §2.1)。
        # 同時刻は車線順で決定論的にほどく
        draws.sort(key=lambda d: (d[0], d[1], d[2]))

    # --- エンティティ化 ---
    added = 0
    for t, li, k, model, is_target in draws:
        lane = traffic['lanes'][li]
        y = float(lane['y'])
        v = float(lane['speed_mps'])
        direction = 1 if int(lane.get('direction', 1)) >= 0 else -1

        # 到着時刻 t を「sim 開始時に入口の v·t 手前に置く」ことで実現する
        # (スポーン時刻の機構は不要。既存の移動プラグインが等速で運ぶ)。
        # t < 0 (prefill) では入口より先 = コリドー内から始まる
        if direction > 0:
            x_start, x_end, yaw = c0 - v * t, c1 + margin, 0.0
            if x_start > c1:
                continue                      # 開始時点で既に通過済み
        else:
            x_start, x_end, yaw = c1 + v * t, c0 - margin, math.pi
            if x_start < c0:
                continue

        prefix = 'tx' if is_target else 'tf'
        name = f"{prefix}{li}_{k:03d}"
        if name in existing:
            raise ValueError(f"traffic: エンティティ名衝突 {name}")

        entity = {
            'name': name,
            'model': template['model'] if is_target else model,
            'role': 'tx' if is_target else 'blocker',
            'pose': [x_start, y, 0.0, 0.0, 0.0, yaw],
            'waypoints': [[x_start, y, 0.0, v], [x_end, y, 0.0, v]],
        }
        if is_target:
            # アンテナ諸元はシナリオ生成側が書いた target_template を引き写すだけ
            # (traffic_gen は幾何を知らない)。
            #   antennas: [{suffix, relative_yaw_by_direction}] があればそれを使い、
            #   無ければ従来の単一アンテナ (relative_yaw_by_direction) にフォールバック。
            # 前向き 1 本だけだと「接近中の RSU」しか捉えられず、通過後の RSU が
            # 使えない。後ろ向きを足すと窓が 13m → 53m に広がる (RSU 2 基構成の実測)
            specs = template.get('antennas')
            if not specs:
                specs = [{'suffix': 'ant',
                          'relative_yaw_by_direction':
                              template.get('relative_yaw_by_direction', {})}]
            ants = []
            for spec in specs:
                rel = spec.get('relative_yaw_by_direction', {})
                rel_yaw = rel.get(str(direction), rel.get(direction, 0.0))
                ants.append({
                    'name': f"{name}_{spec.get('suffix', 'ant')}",
                    'offset': [float(o) for o in
                               spec.get('offset', template['antenna_offset'])],
                    'relative_rpy': [0.0, 0.0, float(rel_yaw)],
                })
            entity['antennas'] = ants
        entities.append(entity)
        existing.add(name)
        added += 1

    # 対象車 (role: tx) の進入タイミングジッタ: 開始位置を進行方向に沿って
    # ±v·N(0, σ) ずらす (等速なので位置ずれ = 時間ずれ)。
    # 注: traffic が生成した tx は到着時刻が既に確率的なので、両方を使うと
    # 二重にばらつく。生成 tx を使う構成では 0.0 のままにすること
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
