#!/usr/bin/env python3
"""
記録の bs 索引が、走行をまたいで同じ RSU を指しているかを検査する。

なぜ要るか
----------
bs 索引はスケジューリング・BS排他・pairs 記録・制御プレーンの地図キーの
すべてで共有される。この対応が走行ごとに変わると、位置を鍵にした地図
(REM) は「互いに別の RSU」の観測を1つの地図に混ぜて学ぶことになり、
平均場の山が消えて接続閾値に届かなくなる。

**この壊れ方は静かである。** 例外も警告も出ず、性能だけが落ちる。しかも
地図を持つ手法だけを狙い撃ちで壊し、実測 RSSI に反応するだけの
ベースライン (assoc_hold 等) は無傷なので、「提案手法が弱い」という
誤った結論を生む。

実際に起きた: 2026-08-06、19走行中3走行 (16%) で RSU0/RSU1 が反転していた。
原因は TxControllerPlugin が base station を ECM の走査順に集めていたこと
(走査順は動的スポーンの生成順に依存する)。設定順に組み立てるよう修正済み。

検査の考え方
------------
チャネルは位置の決定論的関数なので、同じ RSU・同じ位置なら走行が違っても
同じ RSSI になるはずである。各走行の bs 別プロファイル (位置 → RSSI 平均)
を取り、基準走行のどの bs と最も相関するかを見る。

**低信号での誤検出を避けること。** 接続機会が乏しい構成では bs ごとの
プロファイルがほぼ空になり、相関の argmax が無意味な値を拾う (実測:
δ_rsu=40° の構成で相関 0.39〜0.56 のまま、全単射ですらない対応が出た)。
そこで (a) 相関が min_corr 以上、(b) 最良と次点の差が min_gap 以上、
(c) 対応が全単射、の3条件を満たしたものだけを「規約」として数え、
満たさない走行は「判定不能」として NG の根拠にしない。索引の入れ替わりは
必ず置換なので、全単射でない対応は入れ替わりではなく測定の失敗である。

  python3 tools/check_bs_index_stability.py sim_results/<rec_root>
"""
import argparse
import glob
import os
import sys

import numpy as np
import pandas as pd


def profile(rec_dir, bin_m=5.0):
    """1走行の bs 別プロファイル {bs: Series(位置ビン -> RSSI平均)} を作る。

    位置は pairs に無いので poses から引く。弧長ではなく x をそのまま使う
    (直線路の前提。曲線路なら road 射影に置き換える)。
    """
    poses_path = os.path.join(rec_dir, 'poses.csv')
    pairs_dir = os.path.join(rec_dir, 'pairs')
    if not (os.path.exists(poses_path) and os.path.isdir(pairs_dir)):
        return None
    poses = pd.read_csv(poses_path)
    poses['t_s'] = poses['t_s'].round(6)
    xs = {(r.model, r.t_s): r.x for r in poses.itertuples()}

    rows = []
    for f in sorted(glob.glob(os.path.join(pairs_dir, '*_pairs.csv'))):
        name = os.path.basename(f)[:-len('_pairs.csv')]
        d = pd.read_csv(f)
        if d.empty:
            continue
        d['t_s'] = d['t_s'].round(6)
        d['x'] = [xs.get((name, t), np.nan) for t in d['t_s']]
        d = d[np.isfinite(d['x'])]
        # 進行方向で分けないと上り下りが混ざる。名前の接頭辞を代理に使う
        d['lane'] = name[:3]
        rows.append(d[['bs', 'x', 'rssi_dBm', 'lane']])
    if not rows:
        return None
    d = pd.concat(rows, ignore_index=True)
    d = d[d.rssi_dBm > -200]          # 未記録のセンチネルを除く
    d['xb'] = (d.x / bin_m).round().astype(int)
    out = {}
    for bs, g in d.groupby('bs'):
        out[int(bs)] = g.groupby(['lane', 'xb']).rssi_dBm.mean()
    return out


# ------------------------------------------------------------ 設定を基準にした同定

def anchor_to_config(rec_dir, max_rows=4000):
    """記録の各 bs が、設定上のどの RSU かを幾何から直接同定する。

    走行どうしの相関に頼る方法は、接続機会が乏しい構成でプロファイルが
    ほぼ空になり判定不能になる。こちらは解析リンクモデル
    (tools/analyze_connection_windows.py、実測との残差 +0.10 dB で検証済み)
    で各 RSU の RSSI を計算し、記録値との残差が最小の RSU に割り当てる。
    接続可能かどうかに関係なく全標本が使えるので、低信号でも判定できる。

    戻り値は (対応, 残差RMSE) で、対応[k] = 記録の bs=k に対応する設定 RSU の番号。
    """
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import yaml
    from analyze_connection_windows import LinkModel, Pattern

    cfg_path = os.path.join(rec_dir, 'effective_sim_params.yaml')
    if not os.path.exists(cfg_path):
        return None, None
    params = yaml.safe_load(open(cfg_path, encoding='utf-8'))
    comms = params['comms_simulator_node']['ros__parameters']
    repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    pat = Pattern(os.path.join(repo, 'src/comms_sim_pkg/config/e_plane.csv'),
                  os.path.join(repo, 'src/comms_sim_pkg/config/h_plane.csv'),
                  comms.get('max_antenna_attenuation', 30.0))
    model = LinkModel(params, pat)

    ents = params.get('spawn_entities') or {}
    rsus = []
    for name in sorted(ents):
        e = ents[name]
        pz = e['pose']
        off = e.get('antenna_offset', [0, 0, 0])
        rsus.append((name,
                     np.array([pz[0] + off[0], pz[1] + off[1], pz[2] + off[2]]),
                     pz[5]))
    if not rsus:
        return None, None

    vmap = {v['name']: v for v in params.get('vehicles', [])}
    poses = pd.read_csv(os.path.join(rec_dir, 'poses.csv'))
    poses['t_s'] = poses['t_s'].round(6)
    pos_by = {(r.model, r.t_s): (r.x, r.y, r.z, r.yaw) for r in poses.itertuples()}

    rows = []
    for f in sorted(glob.glob(os.path.join(rec_dir, 'pairs', '*_pairs.csv'))):
        vname = os.path.basename(f)[:-len('_pairs.csv')]
        veh = vmap.get(vname)
        if veh is None:
            continue
        d = pd.read_csv(f)
        d = d[d.rssi_dBm > -200]
        if d.empty:
            continue
        d['t_s'] = d['t_s'].round(6)
        d = d.iloc[::max(1, len(d) // 400)]
        ants = veh.get('antennas', [])
        for r in d.itertuples():
            key = (vname, r.t_s)
            if key not in pos_by or int(r.ant) >= len(ants):
                continue
            x, y, z, yaw = pos_by[key]
            an = ants[int(r.ant)]
            off = an.get('offset', [0, 0, 0])
            apos = np.array([[x + off[0], y + off[1], z + off[2]]])
            ayaw = yaw + an.get('relative_rpy', [0, 0, 0])[2]
            calc = [float(model.rssi(apos, ayaw, rp, ry)[0]) for _, rp, ry in rsus]
            rows.append([int(r.bs), float(r.rssi_dBm)] + calc)
        if len(rows) > max_rows:
            break
    if not rows:
        return None, None

    n = len(rsus)
    arr = np.array(rows)
    cost = np.full((n, n), np.inf)
    for b in range(n):
        m = arr[:, 0] == b
        if not m.any():
            continue
        for j in range(n):
            cost[b, j] = np.sqrt(np.mean((arr[m, 1] - arr[m, 2 + j]) ** 2))
    if not np.isfinite(cost).any():
        return None, None
    from scipy.optimize import linear_sum_assignment
    c = np.where(np.isfinite(cost), cost, 1e6)
    ri, ci = linear_sum_assignment(c)
    mapping = tuple(int(ci[list(ri).index(b)]) for b in range(n))
    rmse = float(np.mean([cost[b, mapping[b]] for b in range(n)
                          if np.isfinite(cost[b, mapping[b]])]))
    return mapping, rmse


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('rec_root', help='seed_* を含む記録ディレクトリ')
    ap.add_argument('--bin-m', type=float, default=5.0, help='位置ビン幅 [m]')
    ap.add_argument('--min-bins', type=int, default=5,
                    help='判定に要る共通ビン数の下限')
    ap.add_argument('--min-corr', type=float, default=0.90,
                    help='対応を確定するのに要る相関の下限')
    ap.add_argument('--min-gap', type=float, default=0.10,
                    help='最良と次点の相関差の下限 (曖昧な対応を弾く)')
    ap.add_argument('--no-anchor', action='store_true',
                    help='設定幾何を基準にした同定を使わず、走行間の相関だけで判定する')
    a = ap.parse_args()

    recs = sorted(d for d in glob.glob(os.path.join(a.rec_root, 'seed_*'))
                  if os.path.isdir(os.path.join(d, 'pairs')))
    if len(recs) < 2:
        sys.exit('記録が2走行以上必要です: %s/seed_*' % a.rec_root)

    # まず設定幾何を基準に同定する (接続機会の多寡に依存しない)
    if not a.no_anchor:
        anchored, failed = {}, []
        for r in recs:
            try:
                m, rmse = anchor_to_config(r)
            except Exception as exc:      # 幾何が引けない記録は相関法に回す
                m, rmse = None, None
                failed.append((os.path.basename(r), str(exc)[:80]))
            if m is None:
                failed.append((os.path.basename(r), '幾何を引けない'))
            else:
                anchored[os.path.basename(r)] = (m, rmse)
        if anchored:
            ident = tuple(range(len(next(iter(anchored.values()))[0])))
            groups = {}
            for n, (m, rmse) in sorted(anchored.items()):
                groups.setdefault(m, []).append(n)
            print('設定幾何を基準にした同定 (%d 走行)' % len(anchored))
            for n, (m, rmse) in sorted(anchored.items()):
                tag = 'OK' if m == ident else '**設定と食い違う**'
                print('  %-14s 対応 %-28s 残差RMSE %6.2f dB  %s'
                      % (n, str(m), rmse, tag))
            if failed:
                print('  (同定できず %d 走行)' % len(failed))
            print()
            if len(groups) > 1:
                # アンカー方式は「どの RSU か」を当てる必要があるため、向きが同じで
                # 近接した RSU (dual 配置の同一傾き面など) を判別できない。
                # 食い違いが出ても入れ替わりの証拠にはならないので、ここでは
                # 判定せず、走行間の一貫性を直接見る相関方式に委ねる。
                # (実測: 8基構成で規約が4通りに割れたが、相関は 1.00 で一貫していた。
                #  本物の入れ替わりは rec_fixblk のように2通りで安定する)
                print('設定幾何での同定は曖昧 (%d 通りに割れた)。' % len(groups))
                print('向きが同じで近接した RSU を含む構成では判別できないため、')
                print('走行間の一貫性を相関で直接見る方式に切り替える\n')
                profs_reset = True
            else:
                only = next(iter(groups))
                if only != ident:
                    print('注意: 全走行で一貫しているが設定順とは異なる (%s)。'
                          % str(only))
                    print('      走行間で一貫していれば RBF 地図は壊れないが、'
                          'bs_positions を幾何的に使う設定では要注意')
                print('OK: %d 走行すべてで bs 索引の規約が一致 (対応 %s)'
                      % (len(anchored), str(only)))
                return 0
        else:
            print('設定幾何での同定ができなかったので、走行間の相関で判定する\n')

    profs = {}
    for r in recs:
        p = profile(r, a.bin_m)
        if p:
            profs[os.path.basename(r)] = p
    if len(profs) < 2:
        sys.exit('プロファイルを作れた走行が足りません')

    names = sorted(profs)
    bs_ids = sorted(profs[names[0]])

    def mapping_to(ref_name, name):
        """name の各 bs が ref_name のどの bs に対応するか。

        戻り値は (対応, 相関, 確信できたか)。相関が min_corr 未満、または
        最良と次点の差が min_gap 未満なら確信なしとする。
        """
        ref, cur = profs[ref_name], profs[name]
        out, corr, conf = [], [], []
        for b in bs_ids:
            if b not in cur:
                out.append(None)
                corr.append(float('nan'))
                conf.append(False)
                continue
            cs = {}
            for rb in bs_ids:
                if rb not in ref:
                    continue
                idx = cur[b].index.intersection(ref[rb].index)
                if len(idx) < a.min_bins:
                    continue
                c = np.corrcoef(cur[b][idx], ref[rb][idx])[0, 1]
                if np.isfinite(c):
                    cs[rb] = c
            if not cs:
                out.append(None)
                corr.append(float('nan'))
                conf.append(False)
                continue
            ranked = sorted(cs.items(), key=lambda kv: -kv[1])
            best, best_c = ranked[0]
            second = ranked[1][1] if len(ranked) > 1 else -2.0
            out.append(best)
            corr.append(best_c)
            conf.append(best_c >= a.min_corr and (best_c - second) >= a.min_gap)
        return tuple(out), corr, conf

    groups = {}          # 規約 (対応の組) -> [走行名]
    details = {}
    unsure = []
    for n in names:
        key, corr, conf = mapping_to(names[0], n)
        details[n] = (key, corr, conf)
        # 全 bs を確信を持って判定でき、かつ対応が全単射のときだけ規約に数える
        if all(conf) and None not in key and len(set(key)) == len(key):
            groups.setdefault(key, []).append(n)
        else:
            unsure.append(n)

    if not groups:
        print('判定できる走行がありません '
              '(接続機会が乏しく、相関で bs の対応を決められない)')
        print('  min_corr=%s / min_gap=%s を緩めるか、接続機会の多い構成で'
              '検査すること' % (a.min_corr, a.min_gap))
        return 2

    major = max(groups, key=lambda k: len(groups[k]))
    print('%14s ' % '走行' + ''.join('%10s' % ('bs%d→' % b) for b in bs_ids)
          + '   判定')
    for n in names:
        key, corr, conf = details[n]
        cells = ''.join('%10s' % (('%d(%.2f)' % (k, c)) if k is not None else '?')
                        for k, c in zip(key, corr))
        if n in unsure:
            verdict = '判定不能'
        elif key == major:
            verdict = 'OK'
        else:
            verdict = '**規約が違う**'
        print('%14s %s   %s' % (n, cells, verdict))

    if unsure:
        print('\n判定不能 %d/%d 走行 (相関 < %s か、対応が全単射でない = 信号不足)'
              % (len(unsure), len(names), a.min_corr))

    print()
    if len(groups) > 1:
        print('NG: bs 索引の規約が %d 通りある (1 通りでなければならない)'
              % len(groups))
        for key, members in sorted(groups.items(), key=lambda kv: -len(kv[1])):
            tag = '多数派' if key == major else '少数派'
            print('    %s %3d 走行  対応 %s: %s' % (tag, len(members), key, members))
        print('    この記録で位置を鍵にした地図を学習・評価してはいけない。')
        print('    プラグインが base station を設定順に並べているか確認すること')
        return 1
    n_ok = sum(len(v) for v in groups.values())
    print('OK: 判定できた %d 走行すべてで bs 索引の規約が一致 (対応 %s)'
          % (n_ok, major))
    return 0


if __name__ == '__main__':
    sys.exit(main())
