#!/usr/bin/env python3
"""
RSU 配置から接続窓を解析的に求め、「選択の余地」がどれだけあるかを測る。

なぜ要るか
----------
予測型スケジューリングの利得は、**どの RSU を選ぶかに意味がある区間**でしか
生まれない。ある時点で接続可能な RSU が1基しかなければ、どんな予測器でも
反応型と同じ選択をするしかない。したがって情報上界は「2基以上が同時に
接続可能な区間の割合」で上から押さえられる。

現行構成 (RSU 間隔 10 m・傾き ±70°) はこの重なりがほぼ無く、実測でも情報
上界は assoc_hold 比 +4.5% しか無かった。本ツールは Gazebo を回さずに配置を
掃引し、重なりが生まれる構成を先に見つけるためのもの。

物理は TxControllerPlugin と同じ (AntennaPatternParser の E/H 面パターン合成 +
自由空間経路損失)。遮蔽とシャドウイングは含まない — 接続窓の上側の輪郭を
見るのが目的で、遮蔽はそれを削る方向にしか働かないため。

  # 記録された真値と突き合わせて物理の実装を検証する
  python3 tools/analyze_connection_windows.py --validate sim_results/rec_fixblk

  # 配置を掃引する
  python3 tools/analyze_connection_windows.py --sweep
"""
import argparse
import glob
import os
import sys

import numpy as np
import pandas as pd
import yaml

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


# ------------------------------------------------------------ アンテナ

class Pattern:
    """E/H 面パターンの合成 (antenna_pattern_parser.cpp の移植)。"""

    def __init__(self, e_csv, h_csv, max_atten_db=30.0):
        e = np.loadtxt(e_csv, delimiter=',')
        h = np.loadtxt(h_csv, delimiter=',')
        self.ea, self.eg = e[:, 0], e[:, 1]
        self.ha, self.hg = h[:, 0], h[:, 1]
        self.peak = max(self.eg.max(), self.hg.max())
        self.max_atten = float(max_atten_db)

    def gain(self, el_deg, az_deg):
        e = np.interp(el_deg, self.ea, self.eg, left=self.eg[0], right=self.eg[-1])
        h = np.interp(az_deg, self.ha, self.hg, left=self.hg[0], right=self.hg[-1])
        e_at = np.minimum(self.peak - e, self.max_atten)
        h_at = np.minimum(self.peak - h, self.max_atten)
        return np.maximum(self.peak - e_at - h_at, self.peak - self.max_atten)


def rotz(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def frame_angles(ant_pos, tgt_pos, yaw):
    """アンテナ座標系での (仰角, 方位角) [deg]。tgt_pos は (N,3) 可。"""
    v = np.atleast_2d(tgt_pos) - np.asarray(ant_pos)
    n = np.linalg.norm(v, axis=1, keepdims=True)
    n[n < 1e-12] = 1e-12
    v = v / n
    va = v @ rotz(yaw)            # R^T v = v @ R
    az = np.arctan2(va[:, 1], va[:, 0])
    el = np.arcsin(np.clip(va[:, 2], -1.0, 1.0))
    return np.degrees(el), np.degrees(az)


# ------------------------------------------------------------ リンク

class LinkModel:
    def __init__(self, params, pattern):
        comms = params['comms_simulator_node']['ros__parameters']
        pl = comms['path_loss']
        self.tx_power = float(comms['tx_power'])
        self.pl_d0 = float(pl['pl_d0'])
        self.d0 = float(pl['d0'])
        self.n = float(pl['exponent'])
        rate = comms['rate_model']
        self.rssi_min = float(rate['noise_floor_dbm']) + float(rate['snr_min_db'])
        self.pat = pattern

    def rssi(self, tx_pos, tx_yaw, rx_pos, rx_yaw):
        """tx_pos: (N,3) 車載アンテナ / rx_pos: (3,) RSU アンテナ。"""
        tx_pos = np.atleast_2d(tx_pos)
        d = np.linalg.norm(tx_pos - np.asarray(rx_pos), axis=1)
        d = np.maximum(d, 1e-6)
        pl = self.pl_d0 + 10.0 * self.n * np.log10(d / self.d0)
        el_t, az_t = frame_angles(tx_pos, np.tile(rx_pos, (len(tx_pos), 1)), tx_yaw)
        # RSU 側は 1 点なので、各車位置に対して都度角度を出す
        v = tx_pos - np.asarray(rx_pos)
        nv = np.linalg.norm(v, axis=1, keepdims=True)
        nv[nv < 1e-12] = 1e-12
        va = (v / nv) @ rotz(rx_yaw)
        az_r = np.degrees(np.arctan2(va[:, 1], va[:, 0]))
        el_r = np.degrees(np.arcsin(np.clip(va[:, 2], -1.0, 1.0)))
        return self.tx_power - pl + self.pat.gain(el_t, az_t) + self.pat.gain(el_r, az_r)


# ------------------------------------------------------------ 構成

def build_geometry(rsu_sep_m, rsu_tilt_deg, veh_tilt_deg, rsu_y=6.0, rsu_z=2.5,
                   lane_y=(1.75, -1.75), veh_z=1.35, same_dir=False,
                   dual=False, num_poles=2):
    """RSU (位置, ヨー) と、車線ごとの車載アンテナのヨーを返す。

    既定 (same_dir=False) は config/scenarios/road_urban_2lane.yaml と同じで、
    2基が互いに逆を向く (rsu_0 が上流、rsu_1 が下流)。この構成では2基の
    接続窓が原理的に交わらないため、間隔をどう振っても選択の余地が生まれない。

    same_dir=True は2基を同じ向きに揃える。窓が間隔より長ければ重なる。
    """
    t = np.radians(rsu_tilt_deg)
    if dual:
        # tools/lib/road_geometry.py の rsu_tilt_pattern='dual' と同じ。
        # 各ポールに上流面と下流面を載せるので、論理 RSU はポール数の2倍。
        # 同じ向きの面どうしがポール間隔だけ離れて並ぶため、間隔が窓長より
        # 短ければ担当領域が重なり「選べる RSU が2基以上」の区間が生まれる
        rsus = []
        for p in range(num_poles):
            x = (p - (num_poles - 1) / 2.0) * rsu_sep_m
            for face, yaw in (('u', -(np.pi - t)), ('d', -t)):
                rsus.append({'name': f'rsu_{p}{face}',
                             'pos': np.array([x, rsu_y, rsu_z]), 'yaw': yaw})
    else:
        yaws = (-t, -t) if same_dir else (-(np.pi - t), -t)
        rsus = [
            {'name': 'rsu_0', 'pos': np.array([-rsu_sep_m / 2, rsu_y, rsu_z]),
             'yaw': yaws[0]},
            {'name': 'rsu_1', 'pos': np.array([+rsu_sep_m / 2, rsu_y, rsu_z]),
             'yaw': yaws[1]},
        ]
    v = np.radians(veh_tilt_deg)
    lanes = [
        {'name': 'tx0', 'y': lane_y[0], 'heading': 0.0,
         'ant_yaw': [v, np.pi - v], 'z': veh_z},
        {'name': 'tx1', 'y': lane_y[1], 'heading': np.pi,
         'ant_yaw': [-v, -(np.pi - v)], 'z': veh_z},
    ]
    return rsus, lanes


def windows(model, rsus, lanes, x_range=(-105.0, 105.0), dx=0.25):
    """各 (車線, アンテナ, RSU) の RSSI(x) と、接続窓の論理値を返す。"""
    xs = np.arange(x_range[0], x_range[1] + dx, dx)
    out = {}
    for ln in lanes:
        pos = np.stack([xs, np.full_like(xs, ln['y']),
                        np.full_like(xs, ln['z'])], axis=1)
        for ai, ayaw in enumerate(ln['ant_yaw']):
            world_yaw = ln['heading'] + ayaw
            for bi, r in enumerate(rsus):
                out[(ln['name'], ai, bi)] = model.rssi(pos, world_yaw,
                                                       r['pos'], r['yaw'])
    return xs, out


def choice_metrics(xs, w, lanes, n_rsu, rssi_min):
    """車線ごとに『何基が同時に接続可能か』を数え、選択の余地を測る。"""
    dx = xs[1] - xs[0]
    rows = []
    for ln in lanes:
        # その位置で接続可能な RSU 集合 (アンテナはどれか1本使えればよい)
        ok = np.zeros((n_rsu, len(xs)), dtype=bool)
        for bi in range(n_rsu):
            for ai in range(2):
                ok[bi] |= w[(ln['name'], ai, bi)] >= rssi_min
        n_ok = ok.sum(axis=0)
        any_len = float((n_ok >= 1).sum() * dx)
        multi_len = float((n_ok >= 2).sum() * dx)
        rows.append({
            'lane': ln['name'],
            '接続可能長_m': any_len,
            '複数可能長_m': multi_len,
            '選択余地率': (multi_len / any_len) if any_len > 0 else 0.0,
        })
    return pd.DataFrame(rows)


# ------------------------------------------------------------ 検証

def validate(rec_root, model):
    """記録された真値と解析計算を突き合わせる (LOS 標本のみ)。"""
    seeds = sorted(glob.glob(os.path.join(rec_root, 'seed_*')))
    seeds = [s for s in seeds if os.path.isdir(os.path.join(s, 'pairs'))]
    if not seeds:
        sys.exit(f"記録がありません: {rec_root}")
    params = yaml.safe_load(open(os.path.join(seeds[0], 'effective_sim_params.yaml')))

    ents = params.get('spawn_entities') or {}
    rsus = []
    for name in sorted(ents):
        e = ents[name]
        p = e['pose']
        off = e.get('antenna_offset', [0, 0, 0])
        rsus.append({'name': name,
                     'pos': np.array([p[0] + off[0], p[1] + off[1], p[2] + off[2]]),
                     'yaw': p[5]})
    print(f"RSU: " + ", ".join(f"{r['name']} pos={np.round(r['pos'],2)} "
                               f"yaw={np.degrees(r['yaw']):.1f}°" for r in rsus))

    vmap = {v['name']: v for v in params.get('vehicles', [])}
    rows = []
    for sd in seeds:
        poses = pd.read_csv(os.path.join(sd, 'poses.csv'))
        poses['t_s'] = poses['t_s'].round(6)
        pos_by = {(r.model, r.t_s): (r.x, r.y, r.z, r.yaw) for r in poses.itertuples()}
        for f in sorted(glob.glob(os.path.join(sd, 'pairs', '*_pairs.csv')))[:3]:
            name = os.path.basename(f)[:-len('_pairs.csv')]
            veh = vmap.get(name)
            if veh is None:
                continue
            d = pd.read_csv(f)
            d['t_s'] = d['t_s'].round(6)
            d = d[(d.rssi_dBm > -200) & (d.los > 0)]
            if d.empty:
                continue
            d = d.iloc[::37]                      # 間引き
            for r in d.itertuples():
                key = (name, r.t_s)
                if key not in pos_by:
                    continue
                x, y, z, yaw = pos_by[key]
                ants = veh.get('antennas', [])
                if int(r.ant) >= len(ants):
                    continue
                a = ants[int(r.ant)]
                off = a.get('offset', [0, 0, 0])
                apos = np.array([[x + off[0], y + off[1], z + off[2]]])
                ayaw = yaw + a.get('relative_rpy', [0, 0, 0])[2]
                for bi, rs in enumerate(rsus):
                    rows.append({'bs_rec': int(r.bs), 'rsu_idx': bi,
                                 'truth': r.rssi_dBm,
                                 'calc': float(model.rssi(apos, ayaw, rs['pos'], rs['yaw'])[0])})
    v = pd.DataFrame(rows)
    if v.empty:
        sys.exit("突き合わせできる標本がありません")
    print(f"\n突き合わせ標本 {len(v):,} (LOS のみ、遮蔽・シャドウイングは計算に含まない)")
    print("\n記録の bs 列と解析計算の RSU の対応 (残差 = 真値 - 計算):")
    print(f"{'bs列':>5} {'RSU':>5} {'n':>7} {'残差平均':>10} {'残差std':>9}")
    for (b, ri), g in v.groupby(['bs_rec', 'rsu_idx']):
        res = g.truth - g.calc
        print(f"{b:>5} {ri:>5} {len(g):>7,} {res.mean():>10.2f} {res.std():>9.2f}")
    print("\n残差の平均が0付近・stdが小さい組み合わせが正しい対応。")
    print("シャドウイング (σ=4dB) と遮蔽の取りこぼしの分だけ残差が残るのは正常。")


# ------------------------------------------------------------ 掃引

def sweep(model, seps, tilts, veh_tilt, same_dir=False, dual=False, num_poles=2):
    rows = []
    for sep in seps:
        for tilt in tilts:
            rsus, lanes = build_geometry(sep, tilt, veh_tilt, same_dir=same_dir,
                                         dual=dual, num_poles=num_poles)
            xs, w = windows(model, rsus, lanes)
            m = choice_metrics(xs, w, lanes, len(rsus), model.rssi_min)
            rows.append({'ポール間隔_m' if dual else 'RSU間隔_m': sep,
                         'RSU傾き_deg': tilt,
                         '接続可能長_m': m['接続可能長_m'].mean(),
                         '複数可能長_m': m['複数可能長_m'].mean(),
                         '選択余地率': m['選択余地率'].mean()})
    t = pd.DataFrame(rows)
    print(f"\n接続閾値 {model.rssi_min:.1f} dBm / 車載傾き {veh_tilt}° / 2車線平均"
          f" / 配置 {'dual (1ポール2面)' if dual else ('同方向' if same_dir else '逆方向 (現行)')}"
          f"{f' / ポール {num_poles} 本 = 論理RSU {2*num_poles} 基' if dual else ''}")
    print("選択余地率 = 2基以上が同時に接続可能な区間 / 1基以上が接続可能な区間")
    sep_col = 'ポール間隔_m' if dual else 'RSU間隔_m'
    piv = t.pivot(index=sep_col, columns='RSU傾き_deg', values='選択余地率')
    print("\n=== 選択余地率 ===")
    print(piv.round(3).to_string())
    piv2 = t.pivot(index=sep_col, columns='RSU傾き_deg', values='接続可能長_m')
    print("\n=== 接続可能長 [m] (1基以上) ===")
    print(piv2.round(1).to_string())
    return t


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--validate', metavar='REC_ROOT',
                    help='記録された真値と物理計算を突き合わせる')
    ap.add_argument('--sweep', action='store_true', help='配置を掃引する')
    ap.add_argument('--params', default=None,
                    help='sim_params (既定は --validate 先の記録、なければ既定設定)')
    ap.add_argument('--veh-tilt', type=float, default=20.0, help='車載アンテナ傾き [deg]')
    ap.add_argument('--seps', type=float, nargs='*',
                    default=[10, 20, 40, 60, 80, 120])
    ap.add_argument('--tilts', type=float, nargs='*',
                    default=[20, 35, 50, 70, 85])
    ap.add_argument('--dual', action='store_true',
                    help='1ポールに上流/下流の2面を載せる '
                         '(road_geometry.py の rsu_tilt_pattern=dual と同じ)')
    ap.add_argument('--num-poles', type=int, default=2, help='--dual のポール本数')
    ap.add_argument('--same-dir', action='store_true',
                    help='2基を同じ向きに揃える (窓を重ねられるか見る)')
    ap.add_argument('--snr-min', type=float, default=None,
                    help='接続に要る SNR [dB] を上書きする (既定は設定値)。'
                         'MCS 段階的低下を模擬したときの効き方を見る')
    ap.add_argument('--out', default=None, help='掃引結果 CSV の書き出し先')
    a = ap.parse_args()

    pp = a.params
    if pp is None and a.validate:
        c = sorted(glob.glob(os.path.join(a.validate, 'seed_*',
                                          'effective_sim_params.yaml')))
        pp = c[0] if c else None
    if pp is None:
        sys.exit("--params か --validate のいずれかで sim_params を与えてください")
    params = yaml.safe_load(open(pp))
    comms = params['comms_simulator_node']['ros__parameters']
    pat = Pattern(os.path.join(REPO, 'src/comms_sim_pkg/config/e_plane.csv'),
                  os.path.join(REPO, 'src/comms_sim_pkg/config/h_plane.csv'),
                  comms.get('max_antenna_attenuation', 30.0))
    model = LinkModel(params, pat)
    if a.snr_min is not None:
        nf = float(params['comms_simulator_node']['ros__parameters']
                   ['rate_model']['noise_floor_dbm'])
        model.rssi_min = nf + a.snr_min
    print(f"パターン peak {pat.peak:.2f} dBi / 最大減衰 {pat.max_atten} dB / "
          f"tx {model.tx_power} dBm / 接続閾値 {model.rssi_min:.1f} dBm")

    if a.validate:
        validate(a.validate, model)
    if a.sweep:
        t = sweep(model, a.seps, a.tilts, a.veh_tilt, same_dir=a.same_dir,
                  dual=a.dual, num_poles=a.num_poles)
        if a.out:
            t.to_csv(a.out, index=False)
            print(f"\n[windows] wrote {a.out}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
