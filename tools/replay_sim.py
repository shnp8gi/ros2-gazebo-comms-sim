#!/usr/bin/env python3
"""
記録した軌跡と全ペア RSSI から、通信とスケジューリングを事後に再計算する。

なぜ事後計算にするか
--------------------
通信計算は車両の運動に影響しない (VehicleMotionController は運動状態と dt しか
受け取らない)。したがって物理は「手法」に依存せず交通実現にのみ依存する。
実行時評価では制御プレーンを別プロセスの ROS ノードとして非同期に動かして
いたため、計算機の混み具合で計画が効き始めるシム時刻が変わり、外部プロセスで
動く手法だけが負荷で不利になっていた (実測: KKF ノード4個同時で grant -4.6%、
プラグイン内で完結する assoc_hold は +2.6%)。手法間の差 5-14% と同程度の交絡で、
同じ実験を2回やると結論が変わる状態だった。

事後計算なら単一プロセスで同期実行できるので、
  - 完全に決定的 (非同期のメッセージングが無い)
  - 全手法が同一のチャネル実現を共有する = 共通乱数の理想形
  - Gazebo 実行が「手法 × シード」から「シードのみ」に減る

チャネル物理は記録済みなので、ここで実装するのは**判断に依存する部分だけ**
(リンク状態機械・排他制御・スループット積算・スケジューラ)。

適用限界 (重要)
---------------
**この方式は「通信が運動に影響しない」前提の上にのみ成立する。**
リアルタイム車両制御 (隊列走行・接続維持のための速度調整・遠隔運転・協調合流)
を入れると軌跡が手法に依存するようになり、記録した軌跡を使い回せなくなる。

前提が破れているかは tools/check_motion_independence.py で検査できる。
**モデルに制御則を足したら必ず走らせること。** 走らせないと、再生の結果は
失敗ではなく静かな誤りになる。代替案は docs/replay_architecture.md を参照。

入力
----
  <rec>/poses.csv                全モデルの姿勢 (t_s,model,x,y,z,roll,pitch,yaw)
  <rec>/pairs/<vehicle>_pairs.csv 全ペア RSSI (t_s,ant,bs,rssi_dBm,los)
  <rec>/config/sim_params_*.yaml  実効設定 (レート・閾値・T_est 等)

  python3 tools/replay_sim.py <rec_dir> --arm assoc_hold --out <dir>
"""
import argparse
import glob
import math
import os
import sys

import numpy as np
import pandas as pd
import yaml


# --------------------------------------------------------------- 設定

class ReplayConfig:
    """実効 sim_params から、再計算に要る値だけを取り出す。"""

    def __init__(self, params):
        comms = params.get('comms_simulator_node', {}).get('ros__parameters', {})
        link = params.get('link_controller_node', {}).get('ros__parameters', {})
        rate = comms.get('rate_model', {}) or {}

        self.bandwidth_hz = float(rate.get('bandwidth_hz', 1e8))
        self.noise_floor_dbm = float(rate.get('noise_floor_dbm', -95.0))
        self.efficiency = float(rate.get('efficiency', 1.0))
        self.snr_min_db = float(rate.get('snr_min_db', 0.0))
        self.snr_cap_db = float(rate.get('snr_cap_db', 50.0))
        # 接続閾値: ShannonRateModel::MinRssiDbm と同一
        self.rssi_min_dbm = self.noise_floor_dbm + self.snr_min_db

        self.t_est_ms = float(comms.get('link_establishment_time_ms', 2.0))
        self.comms_period_s = float(comms.get('comms_update_period_s', 0.005))
        self.data_limit_mb = float(comms.get('comm_data_limit_mb', 0.0))
        self.assoc_recover_timeout_s = float(link.get('assoc_recover_timeout_s', 0.2))

    def rate_gbps(self, rssi_dbm):
        """ShannonRateModel::RateGbps の移植 (配列対応)。"""
        snr_db = np.asarray(rssi_dbm, dtype=float) - self.noise_floor_dbm
        ok = snr_db >= self.snr_min_db
        snr_db = np.minimum(snr_db, self.snr_cap_db)
        snr = np.power(10.0, snr_db / 10.0)
        r = self.efficiency * self.bandwidth_hz * np.log2(1.0 + snr) / 1.0e9
        return np.where(ok, r, 0.0)


# --------------------------------------------------------------- 排他

class BsOccupancy:
    """BsOccupancyRegistry の移植 (1 BS = 1 車両)。"""

    def __init__(self):
        self.owner_by_bs = {}

    def available(self, bs, owner):
        o = self.owner_by_bs.get(bs)
        return o is None or o == owner

    def sync(self, owner, desired_bs):
        for b in [b for b, o in self.owner_by_bs.items()
                  if o == owner and b != desired_bs]:
            del self.owner_by_bs[b]
        if desired_bs is None or desired_bs < 0:
            return False
        o = self.owner_by_bs.get(desired_bs)
        if o is None:
            self.owner_by_bs[desired_bs] = owner
            return True
        return o == owner


# --------------------------------------------------------------- 車両状態

class AntennaState:
    __slots__ = ('assigned_bs', 'outage_start', 'link_state', 'est_steps',
                 'last_rssi', 'total_mb', 'comm_active', 'grant_s',
                 'connected_s', 'connectable_s')

    def __init__(self):
        self.assigned_bs = -1
        self.outage_start = -1.0
        self.link_state = 'DISCONNECTED'
        self.est_steps = 0
        self.last_rssi = -999.0
        self.total_mb = 0.0
        self.comm_active = True
        self.grant_s = 0.0
        self.connected_s = 0.0
        self.connectable_s = 0.0


class VehicleReplay:
    """1車両分の再計算。RSSI 表は (時刻 × アンテナ × BS) の密行列で持つ。"""

    def __init__(self, name, times, rssi, los, cfg):
        self.name = name
        self.times = times            # (T,)
        self.rssi = rssi              # (T, n_ant, n_bs)
        self.los = los                # (T, n_ant, n_bs)
        self.cfg = cfg
        self.n_ant = rssi.shape[1]
        self.n_bs = rssi.shape[2]
        self.ants = [AntennaState() for _ in range(self.n_ant)]
        self.active_ant = -1
        self.idx_by_t = {round(float(t), 6): i for i, t in enumerate(times)}

    def row(self, t):
        return self.idx_by_t.get(round(float(t), 6))


# --------------------------------------------------------------- 方策

def policy_assoc_hold(veh, k, t, occ, cfg):
    """802.15.3e 準拠の受動接続 (TxControllerPlugin の assoc_hold を移植)。

    最初に閾値を超えた空き RSU にアソシし、閾値以上の間は保持する。閾値割れは
    即断とせず回復待機に入り、T_recover 以内に回復すれば同じ RSU で再開、
    超えたら断を宣言して次の圏内 RSU へ再アソシする。より良い RSU へ能動的には
    移らない (= HO しないことの代償が指標に出る)。
    """
    rmin = cfg.rssi_min_dbm
    for i, ant in enumerate(veh.ants):
        r = veh.rssi[k, i]
        cur = ant.assigned_bs
        if 0 <= cur < veh.n_bs:
            if r[cur] >= rmin:
                ant.outage_start = -1.0
            else:
                if ant.outage_start < 0.0:
                    ant.outage_start = t
                if t - ant.outage_start > cfg.assoc_recover_timeout_s:
                    ant.assigned_bs = -1
                    ant.outage_start = -1.0
        if ant.assigned_bs < 0:
            # 最良ではなく先頭 (index 順) から採るため能動選択にならない
            for b in range(veh.n_bs):
                if not occ.available(b, veh.name):
                    continue
                if r[b] >= rmin:
                    ant.assigned_bs = b
                    ant.outage_start = -1.0
                    break
        if ant.assigned_bs >= 0:
            veh.active_ant = i


# --------------------------------------------------------------- KKF

class _CapturedSchedule:
    """MpcScheduler が発行する HoSchedule を配信せず受け取るだけの器。"""

    def __init__(self):
        self.entries = []          # [(t_start, ant, bs, vehicle)]

    def publish(self, sched):
        self.entries = [(e.t_start, e.ant, e.bs, e.vehicle) for e in sched.plan]


def make_kkf_scheduler(sim_params_path, state_dir=None, overrides=None):
    """既存の MpcScheduler を、通信層だけ差し替えて再生から呼べるようにする。

    計画ロジック (クリギング・LCB・割当) には一切手を触れない。実行時と同じ
    コードが同じ入力に対して同じ判断を返すことが要件なので、再実装はしない。
    """
    sys.path.insert(0, os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        '..', 'src', 'comms_sim_pkg', 'comms_sim_pkg'))
    import kkf_scheduler_node as ksn

    cfg = ksn.SchedulerConfig(sim_params_path)
    # 記録走行は制御プレーンを使わない構成 (control_plane: none) で回すため、
    # 保存された設定にはその値が残っている。再生では KKF の予測器が要る
    cfg.control_plane = 'kkf_mpc'
    for k, v in (overrides or {}).items():
        setattr(cfg, k, v)
    # 学習済み地図は明示したときだけ読む。設定に残った値を暗黙に拾うと
    # cold のつもりが warm になる (実測で kkf_cold が kkf_conv と同値になった)
    cfg.state_dir = state_dir or ''

    class ReplayScheduler(ksn.MpcScheduler):
        def _setup_transport(self):
            self.pub = _CapturedSchedule()   # 配信せず捕捉するだけ

    return ReplayScheduler(cfg), ksn


class Report:
    """MeasurementReport の最小の代役 (_process_report が触る属性のみ)。"""

    class _E:
        __slots__ = ('ant', 'bs', 'rssi_dbm')

        def __init__(self, ant, bs, rssi):
            self.ant, self.bs, self.rssi_dbm = ant, bs, rssi

    class _P:
        __slots__ = ('x', 'y', 'z')

        def __init__(self, p):
            self.x, self.y, self.z = float(p[0]), float(p[1]), float(p[2])

    def __init__(self, t, vehicle, pos, entries):
        self.t_sim = t
        self.vehicle = vehicle
        self.vehicle_pos = Report._P(pos)
        self.reports = [Report._E(a, b, r) for a, b, r in entries]


class ExternalSchedule:
    """ExternalScheduleStrategy の移植 (計画の区間解決と再確立)。"""

    def __init__(self):
        self.cur_ant = -1
        self.cur_bs = -1
        self.entries = []
        self.valid_until = -1.0

    def set_plan(self, entries, valid_until):
        # 空の計画は「今は言うことがない」なので現割当に触れない
        if not entries:
            return
        self.entries = sorted(entries, key=lambda e: e[0])
        self.valid_until = valid_until

    def resolve(self, t):
        """(ant, bs, 切替が起きたか) を返す。"""
        tgt_a, tgt_b = self.cur_ant, self.cur_bs
        if self.entries and t <= self.valid_until:
            for t_start, a, b in self.entries:
                if t_start <= t:
                    tgt_a, tgt_b = a, b
                else:
                    break
        changed = (tgt_a != self.cur_ant) or (tgt_b != self.cur_bs)
        self.cur_ant, self.cur_bs = tgt_a, tgt_b
        return tgt_a, tgt_b, changed


POLICIES = {'assoc_hold': policy_assoc_hold}


# --------------------------------------------------------------- 本体

def step_vehicle(veh, k, t, dt, occ, cfg, policy, grant_by_state):
    policy(veh, k, t, occ, cfg)

    rmin = cfg.rssi_min_dbm
    any_grant_synced = False
    for i, ant in enumerate(veh.ants):
        r = veh.rssi[k, i]
        b = ant.assigned_bs
        # last_rssi: 割当済みならそのBS、未割当なら最良 (SchedulingSupport と同一)
        ant.last_rssi = r[b] if 0 <= b < veh.n_bs else float(np.max(r))
        # 会計に使う指標も割当BS、未割当なら最良BS
        m_bs = b if 0 <= b < veh.n_bs else int(np.argmax(r))

        has_grant = grant_by_state(veh, i)
        if has_grant and b >= 0:
            has_grant = occ.sync(veh.name, b)
            any_grant_synced = True

        if not has_grant or not ant.comm_active:
            if ant.link_state != 'DISCONNECTED':
                ant.link_state = 'DISCONNECTED'
                ant.est_steps = 0
            link_ready = False
        else:
            required = int(math.ceil((cfg.t_est_ms / 1000.0) / dt))
            link_ready = False
            if ant.link_state == 'DISCONNECTED' and ant.last_rssi > rmin:
                ant.link_state = 'ESTABLISHING'
                ant.est_steps = 1
            elif ant.link_state == 'ESTABLISHING':
                if ant.last_rssi <= rmin:
                    ant.link_state = 'DISCONNECTED'
                else:
                    ant.est_steps += 1
                    if ant.est_steps >= required:
                        ant.link_state = 'CONNECTED'
                        link_ready = True
            elif ant.link_state == 'CONNECTED':
                if ant.last_rssi <= rmin:
                    ant.link_state = 'DISCONNECTED'
                else:
                    link_ready = True

        if link_ready and ant.comm_active:
            thr = float(cfg.rate_gbps(r[m_bs]))
            ant.total_mb += thr * 1000.0 / 8.0 * dt
            if cfg.data_limit_mb > 0 and ant.total_mb >= cfg.data_limit_mb:
                ant.comm_active = False
            ant.connected_s += dt
        if has_grant:
            ant.grant_s += dt
        if float(np.max(r)) >= rmin:
            ant.connectable_s += dt

    # どのアンテナも grant を同期しなかったら占有を全解放する。
    # これを落とすと、一度掴んだ車が圏外に出ても RSU を握ったままになり、
    # 後続の車が永久に締め出される (実測: 12台中10台が配信ゼロになった)
    if not any_grant_synced:
        occ.sync(veh.name, -1)


def grant_simple(veh, i):
    """assigned_bs>=0 をそのまま grant とする方策群 (assoc_hold 等)。"""
    return veh.ants[i].assigned_bs >= 0


def load_poses(path):
    """poses.csv を {model: (times, xyz)} に整える (スケジューラの位置入力)。"""
    d = pd.read_csv(path)
    d['t_s'] = d['t_s'].round(6)
    out = {}
    for name, g in d.groupby('model'):
        g = g.sort_values('t_s')
        out[name] = (g['t_s'].to_numpy(),
                     g[['x', 'y', 'z']].to_numpy(),
                     {round(float(t), 6): i for i, t in enumerate(g['t_s'])})
    return out


def load_pairs(pairs_dir, cfg):
    """<vehicle>_pairs.csv を (時刻 × アンテナ × BS) の密行列に整える。"""
    out = {}
    for f in sorted(glob.glob(os.path.join(pairs_dir, '*_pairs.csv'))):
        name = os.path.basename(f)[:-len('_pairs.csv')]
        d = pd.read_csv(f)
        if d.empty:
            continue
        d['t_s'] = d['t_s'].round(6)
        times = np.sort(d['t_s'].unique())
        n_ant = int(d['ant'].max()) + 1
        n_bs = int(d['bs'].max()) + 1
        ti = {t: i for i, t in enumerate(times)}
        rssi = np.full((len(times), n_ant, n_bs), -999.0)
        los = np.zeros((len(times), n_ant, n_bs), dtype=bool)
        ri = d['t_s'].map(ti).to_numpy()
        rssi[ri, d['ant'].to_numpy(), d['bs'].to_numpy()] = d['rssi_dBm'].to_numpy()
        los[ri, d['ant'].to_numpy(), d['bs'].to_numpy()] = d['los'].to_numpy() > 0
        out[name] = VehicleReplay(name, times, rssi, los, cfg)
    return out


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('rec_dir', help='記録ディレクトリ (pairs/ と config/ を含む)')
    ap.add_argument('--arm', default='assoc_hold',
                    choices=sorted(list(POLICIES) + ['kkf']))
    ap.add_argument('--state-dir', default=None,
                    help='学習済み REM のディレクトリ (kkf 用、省略で cold)')
    ap.add_argument('--kkf-set', action='append', default=[], metavar='KEY=VAL',
                    help='SchedulerConfig の属性を上書き (例 kappa=0.0)。'
                         'アブレーションで LCB や分散地図を切るのに使う')
    ap.add_argument('--observe-all-pairs', action='store_true',
                    help='観測を grant 中のペアに限らず全ペアにする。'
                         '「地図を持つ」価値と「事前に学習する」価値を分ける'
                         'kkf_cold_probe 用 (規格忠実ではない)')
    ap.add_argument('--out', required=True, help='出力ディレクトリ')
    ap.add_argument('--config', default=None, help='実効 sim_params (省略時は rec_dir から探す)')
    a = ap.parse_args()

    cfg_path = a.config
    if cfg_path is None:
        cand = glob.glob(os.path.join(a.rec_dir, 'config', 'sim_params*.yaml'))
        if not cand:
            sys.exit(f"実効設定が見つかりません: {a.rec_dir}/config/sim_params*.yaml")
        cfg_path = sorted(cand)[0]
    with open(cfg_path, 'r', encoding='utf-8') as f:
        params = yaml.safe_load(f) or {}
    cfg = ReplayConfig(params)

    pairs_dir = os.path.join(a.rec_dir, 'pairs')
    if not os.path.isdir(pairs_dir):
        pairs_dir = a.rec_dir
    vehicles = load_pairs(pairs_dir, cfg)
    if not vehicles:
        sys.exit(f"RSSI 表が見つかりません: {pairs_dir}")

    # 全車共通の時刻グリッド。プラグインは全車が同一グリッドで更新するので、
    # 再生でもグリッドを共有する (位相がずれると排他の順序が変わる)
    grid = np.unique(np.concatenate([v.times for v in vehicles.values()]))
    dt = cfg.comms_period_s

    # 排他の解決順 = 「先着」。gz-sim は物理ステップごとに全プラグインを単一
    # スレッドで逐次実行するので、実行時はエンティティ順で決まっていた。
    # 再生では車両名の昇順に固定する (決定的であることが要件で、順序自体は
    # 手法によらず共通なので比較の公平性は保たれる)
    order = sorted(vehicles)
    occ = BsOccupancy()

    if a.arm == 'kkf':
        poses_path = os.path.join(a.rec_dir, 'poses.csv')
        if not os.path.exists(poses_path):
            sys.exit(f"KKF には姿勢が要ります: {poses_path}")
        poses = load_poses(poses_path)
        ov = {}
        for kv in a.kkf_set:
            k, _, v = kv.partition('=')
            try:
                ov[k.strip()] = yaml.safe_load(v)
            except Exception:
                ov[k.strip()] = v
        sched, _ksn = make_kkf_scheduler(cfg_path, state_dir=a.state_dir,
                                         overrides=ov)
        ext = {n: ExternalSchedule() for n in order}
        report_period = float(params.get('comms_simulator_node', {})
                              .get('ros__parameters', {})
                              .get('measurement_report', {})
                              .get('period_s', 0.05))
        replan_period = float(sched.cfg.replan_period_s)
        next_report_t, next_plan_t = 0.0, 0.0

        def grant_kkf(veh, i):
            # 単一ペアネット: 計画された (アンテナ, BS) のみ grant
            return i == veh.active_ant and veh.ants[i].assigned_bs >= 0

        for t in grid:
            tf = float(t)
            # 1. 観測 (規格忠実: grant 中のペアのみ。学習相と違い全ペアではない)
            if tf + 1e-9 >= next_report_t:
                next_report_t = (math.floor(tf / report_period) + 1.0) * report_period
                for name in order:
                    v = vehicles[name]
                    k = v.row(tf)
                    if k is None or name not in poses:
                        continue
                    pt, pxyz, pidx = poses[name]
                    j = pidx.get(round(tf, 6))
                    if j is None:
                        continue
                    ent = []
                    if a.observe_all_pairs:
                        for i in range(v.n_ant):
                            for b in range(v.n_bs):
                                ent.append((i, b, float(v.rssi[k, i, b])))
                    else:
                        # 規格忠実: grant 中のペアしか観測できない
                        for i, ant in enumerate(v.ants):
                            b = ant.assigned_bs
                            if b >= 0:
                                ent.append((i, b, float(v.rssi[k, i, b])))
                    sched._process_report(Report(tf, name, pxyz[j], ent))
            # 2. 再計画 (実行時と同じ周期・同じロジック)
            if tf + 1e-9 >= next_plan_t:
                next_plan_t = (math.floor(tf / replan_period) + 1.0) * replan_period
                sched._replan(tf)
                byveh = {}
                for t_start, ant, bs, vid in sched.pub.entries:
                    byveh.setdefault(vid, []).append((t_start, ant, bs))
                for name in order:
                    if name in byveh:
                        ext[name].set_plan(byveh[name], tf + 1.0)
            # 3. 計画の適用とデータ会計
            for name in order:
                v = vehicles[name]
                k = v.row(tf)
                if k is None:
                    continue
                ta, tb, changed = ext[name].resolve(tf)
                for i, ant in enumerate(v.ants):
                    ant.assigned_bs = tb if (i == ta and tb is not None and tb >= 0) else -1
                v.active_ant = ta if 0 <= ta < v.n_ant else -1
                if changed:      # ペア変更はペアネット再確立を課す
                    for ant in v.ants:
                        ant.link_state = 'DISCONNECTED'
                        ant.est_steps = 0
                step_vehicle(v, k, tf, dt, occ, cfg, lambda *_: None, grant_kkf)
    else:
        policy = POLICIES[a.arm]
        for t in grid:
            for name in order:
                v = vehicles[name]
                k = v.row(t)
                if k is None:
                    continue      # その時刻には未スポーン/退場済み
                step_vehicle(v, k, float(t), dt, occ, cfg, policy, grant_simple)

    os.makedirs(a.out, exist_ok=True)
    rows = []
    for name in order:
        v = vehicles[name]
        for i, ant in enumerate(v.ants):
            rows.append({'vehicle_name': f'{name}_ant{i}', 'method': a.arm,
                         'total_data_MB': ant.total_mb,
                         'connected_time_s': ant.connected_s,
                         'grant_time_s': ant.grant_s,
                         'connectable_time_s': ant.connectable_s})
        rows.append({'vehicle_name': f'{name}_total', 'method': a.arm,
                     'total_data_MB': sum(x.total_mb for x in v.ants),
                     'connected_time_s': sum(x.connected_s for x in v.ants),
                     'grant_time_s': sum(x.grant_s for x in v.ants),
                     'connectable_time_s': sum(x.connectable_s for x in v.ants)})
    df = pd.DataFrame(rows)
    out_csv = os.path.join(a.out, 'replay_summary.csv')
    df.to_csv(out_csv, index=False)

    tot = df[df.vehicle_name.str.endswith('_total')]
    print(f"[replay] arm={a.arm} 車両 {len(tot)} 台 / 時刻 {len(grid):,} 点")
    print(f"[replay] 総配信 {tot.total_data_MB.sum():.3f} MB / "
          f"接続 {tot.connected_time_s.sum():.3f} s")
    print(f"[replay] wrote {out_csv}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
