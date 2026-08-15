#!/usr/bin/env python3
"""
学習した REM の予測が実測 RSSI とどれだけ合っているかを測る (較正監査)。

なぜ要るか
----------
アブレーション (sim_results/ablation_fixblk) で、
  - 不確実性機構 (kappa=0 / varmap 無効) を抜いても結果が変わらない
  - 学習した地図を持つ kkf_conv が、事前学習を持たない kkf_cold_probe に劣る
という状態になっている。原因は次のどちらかで、対処がまったく違う:

  (a) 地図の予測が偏っている (較正バグ) → 直せば結果が変わる
  (b) 地図の予測は正しいが、正しくても勝てない (地図が不要な問題設定)
      → 環境か問題設定を変えるしかない

このツールは記録済みの真値 RSSI を答え合わせに使って (a)/(b) を切り分ける。
再生 (tools/replay_sim.py) と同じ経路で予測器を構成するので、監査している
のは評価で実際に使われている予測そのものである。

測るもの
--------
  bias / RMSE      μ − 真値。系統的な偏りがあれば較正バグ
  ゲートの取り違え  スケジューラは μ ≥ idle_lcb_db で「繋がる見込み」を判定し、
                   満たさないペアを割当から外す (_replan_hungarian)。
                   真値が接続閾値 (noise_floor + snr_min) を超えているのに
                   μ が閾値を下回る = **繋がる機会の取りこぼし**
  σ の情報量        |真値 − μ| と σ の順位相関、および z=(真値−μ)/σ の標準偏差。
                   相関がなく z の広がりが 1 から外れていれば、σ は誤差の
                   大きさを何も語っていない = LCB が効かないことの直接の説明

モード
------
  frozen  学習した地図を凍結して予測のみ (走行内観測を使わない)。
          「事前に学習した地図そのものの予測力」を測る
  online  各時刻で予測 → その後に全ペアを取り込む (一段先予測の逐次監査)。
          走行内観測を足すとどこまで良くなるか = 学習済み地図の上積み分

  python3 tools/audit_rem_calibration.py sim_results/rec_fixblk \
      --state-dir /workspace/sim_results/learn_fixblk/rem_state \
      --out sim_results/rem_audit_fixblk --mode frozen
"""
import argparse
import glob
import os
import sys

import numpy as np
import pandas as pd

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TOOLS_DIR)

import yaml  # noqa: E402
import replay_sim  # noqa: E402  (記録の読み込みと予測器構成を共有する)


def audit_seed(rec_dir, state_dir, mode, report_period=None, overrides=None):
    """1シード分の (予測, 真値) の対を全ペア・全時刻について集める。"""
    cfg_path = os.path.join(rec_dir, 'effective_sim_params.yaml')
    import yaml
    with open(cfg_path, 'r', encoding='utf-8') as f:
        params = yaml.safe_load(f) or {}
    rcfg = replay_sim.ReplayConfig(params)

    vehicles = replay_sim.load_pairs(os.path.join(rec_dir, 'pairs'), rcfg)
    poses = replay_sim.load_poses(os.path.join(rec_dir, 'poses.csv'))
    if not vehicles:
        return None, None

    # frozen は地図を凍結して予測のみ。online は取り込むので凍結しない
    ov = {'kkf_freeze': True} if mode == 'frozen' else {}
    ov.update(overrides or {})
    sched, _ = replay_sim.make_kkf_scheduler(cfg_path, state_dir=state_dir,
                                             overrides=ov)
    kappa = float(sched.cfg.kappa) or 1.0
    n_bs = sched.num_bs

    if report_period is None:
        report_period = float(params.get('comms_simulator_node', {})
                              .get('ros__parameters', {})
                              .get('measurement_report', {})
                              .get('period_s', 0.05))

    grid = np.unique(np.concatenate([v.times for v in vehicles.values()]))
    order = sorted(vehicles)
    rows = []
    step = max(1, int(round(report_period / rcfg.comms_period_s)))

    for t in grid[::step]:
        tf = float(t)
        active = []
        for name in order:
            v = vehicles[name]
            k = v.row(tf)
            if k is None or name not in poses:
                continue
            _pt, pxyz, pidx = poses[name]
            j = pidx.get(round(tf, 6))
            if j is None:
                continue
            active.append((name, v, k, pxyz[j]))
        if not active:
            continue

        # --- 予測 (取り込みより前に行う = 一段先予測) ---
        queries, qidx = [], []
        for i, (name, v, k, _pos) in enumerate(active):
            vs = sched.vstates.get(name)
            if vs is None:
                continue
            ant_s = vs.antenna_s_at(tf)
            if ant_s is None:
                continue          # まだ1度も報告が無い (初回ステップ)
            for a, s0 in enumerate(ant_s[:v.n_ant]):
                queries.append((name, a, np.array([s0]), np.array([tf])))
                qidx.append((i, a, float(s0)))
        if queries:
            for b in range(n_bs):
                vals = sched.predictor.lcb_multi(queries, b, kappa)
                for q, (i, a, s0) in enumerate(qidx):
                    lcb_q, mu_q = vals[q]
                    mu = float(np.asarray(mu_q).ravel()[0])
                    lcb = float(np.asarray(lcb_q).ravel()[0])
                    name, v, k, _pos = active[i]
                    if b >= v.n_bs:
                        continue
                    rows.append({
                        't_s': tf, 'vehicle': name, 'ant': a, 'bs': b, 's_m': s0,
                        'mu_dbm': mu, 'sigma_db': (mu - lcb) / kappa,
                        'truth_dbm': float(v.rssi[k, a, b]),
                        'los': bool(v.los[k, a, b]),
                    })

        # --- 取り込み (online のみ実質的な効果がある) ---
        for name, v, k, pos in active:
            ent = []
            if mode == 'online':
                for i2 in range(v.n_ant):
                    for b2 in range(v.n_bs):
                        ent.append((i2, b2, float(v.rssi[k, i2, b2])))
            sched._process_report(replay_sim.Report(tf, name, pos, ent))

    meta = {
        'rssi_min_dbm': rcfg.rssi_min_dbm,
        'idle_lcb_db': float(sched.cfg.idle_lcb_db),
        'kappa': kappa,
    }
    return pd.DataFrame(rows), meta


def summarize(df, meta):
    """監査表から、方針判断に効く指標だけを取り出す。"""
    err = df['mu_dbm'].to_numpy() - df['truth_dbm'].to_numpy()
    sig = df['sigma_db'].to_numpy()
    truth_ok = df['truth_dbm'].to_numpy() >= meta['rssi_min_dbm']
    pred_ok = df['mu_dbm'].to_numpy() >= meta['idle_lcb_db']

    from scipy import stats
    finite = np.isfinite(err) & np.isfinite(sig) & (sig > 0)
    rho = (float(stats.spearmanr(sig[finite], np.abs(err[finite]))[0])
           if finite.sum() > 10 else float('nan'))
    z = err[finite] / sig[finite]

    n = len(df)
    out = {
        'n_samples': n,
        'bias_db': float(err.mean()),
        'rmse_db': float(np.sqrt((err ** 2).mean())),
        'mae_db': float(np.abs(err).mean()),
        'corr_mu_truth': float(np.corrcoef(df['mu_dbm'], df['truth_dbm'])[0, 1]),
        'sigma_mean_db': float(sig.mean()),
        'z_std': float(z.std()) if len(z) else float('nan'),
        'spearman_sigma_abserr': float(rho),
        'truth_connectable_pct': 100.0 * truth_ok.mean(),
        'pred_connectable_pct': 100.0 * pred_ok.mean(),
        # 取りこぼし: 真に繋がるのに地図が「繋がらない」と言った割合
        'miss_pct': 100.0 * float((truth_ok & ~pred_ok).sum()) / max(1, truth_ok.sum()),
        # 幻: 真に繋がらないのに地図が「繋がる」と言った割合
        'false_alarm_pct': 100.0 * float((~truth_ok & pred_ok).sum()) / max(1, (~truth_ok).sum()),
    }
    if truth_ok.any():
        out['bias_db_on_connectable'] = float(err[truth_ok].mean())
        out['rmse_db_on_connectable'] = float(np.sqrt((err[truth_ok] ** 2).mean()))
    return out


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('rec_root', help='record_runs.py の出力 (seed_* を含む)')
    ap.add_argument('--state-dir', required=True, help='学習済み REM')
    ap.add_argument('--out', required=True)
    ap.add_argument('--mode', default='frozen', choices=['frozen', 'online'])
    ap.add_argument('--seeds', type=int, default=0, help='先頭 N シードのみ (0=全部)')
    ap.add_argument('--kkf-set', action='append', default=[], metavar='KEY=VAL',
                    help='SchedulerConfig の上書き。学習時と基底設定が違う記録を'
                         '監査するときに使う (例 rbf_num_bases=20)')
    a = ap.parse_args()

    recs = sorted(d for d in glob.glob(os.path.join(a.rec_root, 'seed_*'))
                  if os.path.isdir(os.path.join(d, 'pairs')))
    if a.seeds > 0:
        recs = recs[:a.seeds]
    if not recs:
        sys.exit(f"記録が見つかりません: {a.rec_root}/seed_*/pairs")
    print(f"[audit] {len(recs)} シード / mode={a.mode}")

    frames, meta = [], None
    for i, r in enumerate(recs, 1):
        ov = {}
        for kv in a.kkf_set:
            k, _, v = kv.partition('=')
            ov[k.strip()] = yaml.safe_load(v)
        df, m = audit_seed(r, a.state_dir, a.mode, overrides=ov)
        if df is None or df.empty:
            print(f"  [{i}/{len(recs)}] {os.path.basename(r)}: 空")
            continue
        df['seed'] = os.path.basename(r)
        frames.append(df)
        meta = m
        print(f"  [{i}/{len(recs)}] {os.path.basename(r)}: {len(df):,} 標本", flush=True)

    if not frames:
        sys.exit("監査対象が空です")
    all_df = pd.concat(frames, ignore_index=True)
    os.makedirs(a.out, exist_ok=True)
    all_df.to_parquet(os.path.join(a.out, f'samples_{a.mode}.parquet'), index=False)

    s = summarize(all_df, meta)
    per_seed = pd.DataFrame([
        dict(seed=k, **summarize(g, meta)) for k, g in all_df.groupby('seed')])
    per_seed.to_csv(os.path.join(a.out, f'per_seed_{a.mode}.csv'), index=False)

    print(f"\n=== REM 較正監査 (mode={a.mode}, N={len(frames)} シード) ===")
    print(f"接続閾値 {meta['rssi_min_dbm']:.1f} dBm / "
          f"ゲート idle_lcb {meta['idle_lcb_db']:.1f} dBm / kappa {meta['kappa']}")
    for k, v in s.items():
        print(f"  {k:28s} {v:,.3f}" if isinstance(v, float) else f"  {k:28s} {v:,}")
    pd.DataFrame([s]).to_csv(os.path.join(a.out, f'summary_{a.mode}.csv'), index=False)
    print(f"\n[audit] wrote {a.out}/summary_{a.mode}.csv")
    return 0


if __name__ == '__main__':
    sys.exit(main())
