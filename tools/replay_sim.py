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
    ap.add_argument('--arm', default='assoc_hold', choices=sorted(POLICIES))
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
    policy = POLICIES[a.arm]

    for t in grid:
        for name in order:
            v = vehicles[name]
            k = v.row(t)
            if k is None:
                continue          # その時刻には未スポーン/退場済み
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
