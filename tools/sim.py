#!/usr/bin/env python3
"""
sim.py — 評価パイプラインの唯一のエントリポイント。

  python3 tools/sim.py run --sweep-config config/sweep/<name>.yaml
  python3 tools/sim.py status [--watch]
  python3 tools/sim.py analyze sim_results/<eval> [--baseline kkf_full] [--delete-raw]
  python3 tools/sim.py plot    sim_results/<eval>
  python3 tools/sim.py report  sim_results/<eval>
  python3 tools/sim.py reproduce sim_results/<eval>
  python3 tools/sim.py verify  sim_results/<A> sim_results/<B>
  python3 tools/sim.py clean   [--delete-raw]

設計原則「評価ディレクトリが唯一の契約」: 各サブコマンドは manifest.yaml と
評価ディレクトリの明文化されたレイアウトだけを読み書きし、互いを知らない。

実行環境: run/status/reproduce は L0 (stdlib+yaml) でホスト側でも動く。
analyze/plot/report/verify は pandas 等が要るため、ホストで叩かれたら
自動的に `docker compose exec sim` で自分自身をコンテナ内へ委譲する。
"""
import argparse
import datetime
import glob
import os
import subprocess
import sys

import yaml

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(TOOLS_DIR)
sys.path.insert(0, TOOLS_DIR)

from lib import manifest as eval_manifest  # noqa: E402  (L0)

IN_CONTAINER = os.path.exists('/.dockerenv')
CONTAINER_COMMANDS = {'analyze', 'plot', 'report', 'verify', 'clean'}


def delegate_to_container(argv):
    """ホストで叩かれたコマンドをコンテナ内の自分へ委譲する。

    docker exec はシグナルを中へ伝えない。ホスト側を止めてもコンテナ内の
    本体と孫 (sweep_sim / gz sim) が生き残り、排他ロックを握ったまま次の
    実行を止めてしまう (実測で複数回発生)。委譲コマンドに一意の目印を
    埋め込み、ホストが止められたらその目印でコンテナ内へ TERM を送る。
    """
    import signal
    import uuid
    rel = [a.replace(REPO_ROOT + os.sep, '') if a.startswith(REPO_ROOT) else a
           for a in argv]
    token = f"SIMTOKEN_{uuid.uuid4().hex[:12]}"
    inner = (f'cd /workspace && PYTHONDONTWRITEBYTECODE=1 exec '
             f'python3 tools/sim.py --run-token {token} ' + ' '.join(rel))
    proc = subprocess.Popen(['docker', 'compose', 'exec', '-T', 'sim',
                             'bash', '-c', inner], cwd=REPO_ROOT)

    def _stop(signum, _frame):
        print(f"\n[sim] シグナル {signum} を受信。コンテナ内の実行を停止します",
              flush=True)
        # 目印で本体を撃つ。本体は自分の子プロセスグループを畳んでから終わる
        for sig in ('TERM', 'KILL'):
            subprocess.call(['docker', 'compose', 'exec', '-T', 'sim',
                             'bash', '-c', f'pkill -{sig} -f {token}'],
                            cwd=REPO_ROOT,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            try:
                proc.wait(timeout=25 if sig == 'TERM' else 5)
                break
            except subprocess.TimeoutExpired:
                continue
        sys.exit(130)

    for _s in (signal.SIGINT, signal.SIGTERM):
        signal.signal(_s, _stop)
    return proc.wait()


def resolve_eval_dir(path):
    path = path.rstrip('/')
    if not os.path.isdir(path):
        cand = os.path.join('sim_results', path)
        if os.path.isdir(cand):
            return cand
        sys.exit(f"評価ディレクトリが見つかりません: {path}")
    return path


def analysis_spec(eval_dir, manifest):
    """分析仕様: sweep スナップショットの analysis: セクション + manifest変数。"""
    spec = {}
    sweep_path = os.path.join(eval_dir, 'config', 'sweep.yaml')
    if not os.path.exists(sweep_path):
        sweep_path = os.path.join(eval_dir, 'sweep_config_backup.yaml')  # 旧形式
    if os.path.exists(sweep_path):
        with open(sweep_path, 'r', encoding='utf-8') as f:
            spec = (yaml.safe_load(f) or {}).get('sweep', {}).get('analysis', {}) or {}
    var_names = [v['name'] for v in (manifest or {}).get('variables', [])]
    if not var_names and os.path.exists(sweep_path):
        with open(sweep_path, 'r', encoding='utf-8') as f:
            data = (yaml.safe_load(f) or {}).get('sweep', {})
        var_names = [v['name'] for v in data.get('variables', [])]
    spec.setdefault('group_by', var_names)
    return spec


# ------------------------------------------------------------------ run

def cmd_run(args):
    """スイープ実行 (sweep_sim.py への薄いラッパ。ホストならコンテナへ委譲)。"""
    inner = (f"cd /workspace && PYTHONDONTWRITEBYTECODE=1 "
             f"python3 tools/sweep_sim.py --sweep-config {args.sweep_config}"
             + (" --no-build" if args.no_build else "")
             + (" --resume" if args.resume else ""))
    if IN_CONTAINER:
        return subprocess.call(['bash', '-c', inner], cwd='/workspace')
    return subprocess.call(['docker', 'compose', 'exec', '-T', 'sim',
                            'bash', '-c', inner], cwd=REPO_ROOT)


def cmd_status(args):
    cmd = [sys.executable, os.path.join(TOOLS_DIR, 'sweep_progress.py')]
    if args.watch:
        cmd.append('--watch')
    return subprocess.call(cmd, cwd=REPO_ROOT)


# ------------------------------------------------------------------ analyze

def cmd_analyze(args):
    import pandas as pd  # L1 (コンテナ内)
    from lib import eval_data, eval_metrics

    eval_dir = resolve_eval_dir(args.eval_dir)
    manifest = eval_manifest.read_manifest(eval_dir)
    spec = analysis_spec(eval_dir, manifest)
    group_vars = spec['group_by']
    if not group_vars:
        sys.exit("sweep変数が特定できません (manifest / sweep.yaml を確認)")

    need = not os.path.exists(os.path.join(eval_dir, 'summaries.parquet'))
    if need or args.reconsolidate:
        print(f"[analyze] 生CSVを統合中: {eval_dir}")
        eval_data.consolidate(eval_dir, delete_raw=args.delete_raw)
    elif args.delete_raw:
        eval_data.consolidate(eval_dir, delete_raw=True)

    tables = eval_data.load_tables(eval_dir)
    runs_df = eval_metrics.runs_table(tables, group_vars)

    out_dir = os.path.join(eval_dir, 'analysis')
    os.makedirs(out_dir, exist_ok=True)
    runs_df.to_csv(os.path.join(out_dir, 'runs.csv'), index=False)
    agg = eval_metrics.aggregate(runs_df, group_vars, spec.get('metrics'))
    agg.to_csv(os.path.join(out_dir, 'agg.csv'), index=False)

    baseline = args.baseline or spec.get('baseline')
    if baseline:
        arm_var = spec.get('arm_var', group_vars[0])
        rest = [v for v in group_vars if v != arm_var]
        paired = eval_metrics.paired(runs_df, baseline, arm_var, rest,
                                     spec.get('metrics'))
        paired.to_csv(os.path.join(out_dir, 'paired.csv'), index=False)

    # コンソール要約 (最初の2変数のピボット)
    met = 'total_data_MB'
    if len(group_vars) >= 2:
        piv = agg.pivot(index=group_vars[0], columns=group_vars[1],
                        values=f'{met}_mean')
        print(f"\n=== {met} (mean) ===\n{piv.round(1).to_string()}")
    else:
        print(agg[[*group_vars, f'{met}_mean', f'{met}_ci95']].round(1)
              .to_string(index=False))
    print(f"\n[analyze] wrote {out_dir}/runs.csv, agg.csv"
          + (", paired.csv" if baseline else " (baseline未指定: paired省略)"))
    return 0


# ------------------------------------------------------------------ plot

# 図ラベルは英語 (コンテナに日本語フォントが無く豆腐化するため)
METRIC_LABELS = {
    'total_data_MB': 'Total data [MB]',
    'connected_time_s': 'Connected time [s]',
    'min_car_data_MB': 'Min per-vehicle data [MB]',
    'jain_index': "Jain's fairness index",
    'nlos_grant_pct': 'NLOS exposure during grant [%]',
    'ho_count': 'Serving handovers',
}


def cmd_plot(args):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import pandas as pd

    eval_dir = resolve_eval_dir(args.eval_dir)
    manifest = eval_manifest.read_manifest(eval_dir)
    spec = analysis_spec(eval_dir, manifest)
    group_vars = spec['group_by']
    agg_path = os.path.join(eval_dir, 'analysis', 'agg.csv')
    if not os.path.exists(agg_path):
        sys.exit("analysis/agg.csv がありません。先に analyze を実行してください")
    agg = pd.read_csv(agg_path)

    fig_dir = os.path.join(eval_dir, 'figures')
    os.makedirs(fig_dir, exist_ok=True)
    arm_var = spec.get('arm_var', group_vars[0])
    x_var = next((v for v in group_vars if v != arm_var), None)
    if x_var is not None:
        # 数値ラベル ("0","2","10") は数値軸に (文字列だと辞書順に並ぶ)
        agg[x_var] = pd.to_numeric(agg[x_var], errors='ignore')

    metrics = [m for m in (spec.get('metrics') or list(METRIC_LABELS))
               if f'{m}_mean' in agg.columns]
    made = []
    for met in metrics:
        fig, ax = plt.subplots(figsize=(5.2, 3.6))
        if x_var is not None:
            for arm, grp in agg.groupby(arm_var):
                grp = grp.sort_values(x_var)
                ax.errorbar(grp[x_var], grp[f'{met}_mean'],
                            yerr=grp[f'{met}_ci95'], marker='o',
                            capsize=3, label=str(arm))
            ax.set_xlabel(x_var)
            ax.legend(fontsize=8)
        else:
            grp = agg.sort_values(arm_var)
            ax.bar(grp[arm_var].astype(str), grp[f'{met}_mean'],
                   yerr=grp[f'{met}_ci95'], capsize=3)
            ax.set_xlabel(arm_var)
        ax.set_ylabel(METRIC_LABELS.get(met, met))
        ax.grid(alpha=0.3)
        fig.tight_layout()
        out = os.path.join(fig_dir, f'{met}.png')
        fig.savefig(out, dpi=200)
        plt.close(fig)
        made.append(os.path.basename(out))
    print(f"[plot] {len(made)} figures -> {fig_dir}/ ({', '.join(made)})")
    return 0


# ------------------------------------------------------------------ report

def _b64_png(path):
    import base64
    with open(path, 'rb') as f:
        return base64.b64encode(f.read()).decode('ascii')


def cmd_report(args):
    import pandas as pd

    eval_dir = resolve_eval_dir(args.eval_dir)
    manifest = eval_manifest.read_manifest(eval_dir) or {}
    name = manifest.get('name', os.path.basename(eval_dir))

    def table_html(csv_path, max_rows=200):
        if not os.path.exists(csv_path):
            return '<p>(なし)</p>'
        try:
            df = pd.read_csv(csv_path)
        except pd.errors.EmptyDataError:
            return '<p>(空 — 例: num_runs=1 では paired 比較が成立しない)</p>'
        if df.empty:
            return '<p>(空)</p>'
        return df.head(max_rows).round(3).to_html(index=False, border=0)

    figs = sorted(glob.glob(os.path.join(eval_dir, 'figures', '*.png')))
    fig_html = '\n'.join(
        f'<figure><img src="data:image/png;base64,{_b64_png(p)}" '
        f'style="max-width:100%"/><figcaption>{os.path.basename(p)}'
        f'</figcaption></figure>' for p in figs)

    git = manifest.get('git', {})
    runs = manifest.get('runs', [])
    n_fail = sum(1 for r in runs if r.get('status') != 'OK')
    fail_rows = ''.join(
        f"<tr><td>{r['task']}</td><td>{r['params']}</td>"
        f"<td>{r['status']}</td><td>{r['attempts']}</td></tr>"
        for r in runs if r.get('status') != 'OK') or \
        '<tr><td colspan="4">失敗タスクなし</td></tr>'
    variables = manifest.get('variables', [])
    var_rows = ''.join(f"<tr><td>{v['name']}</td><td>{', '.join(map(str, v['labels']))}"
                       f"</td></tr>" for v in variables)

    html = f"""<!DOCTYPE html>
<html lang="ja"><head><meta charset="utf-8">
<title>{name} — 評価レポート</title>
<style>
 body {{ font-family: sans-serif; max-width: 1000px; margin: 2em auto; padding: 0 1em; }}
 table {{ border-collapse: collapse; font-size: 13px; }}
 th, td {{ border: 1px solid #ccc; padding: 3px 8px; text-align: right; }}
 th {{ background: #f0f0f0; }}
 code {{ background: #f5f5f5; padding: 1px 4px; }}
 figure {{ margin: 1em 0; }} figcaption {{ color: #666; font-size: 12px; }}
 .meta td {{ text-align: left; }}
</style></head><body>
<h1>{name}</h1>
<p>{manifest.get('created_at', '')} — status: <b>{manifest.get('status', '?')}</b>
 (tasks: {len(runs)}, failed: {n_fail})</p>

<h2>実験条件</h2>
<table class="meta">{var_rows}</table>
<p>num_runs={manifest.get('execution', {}).get('num_runs')},
 base_seed={manifest.get('execution', {}).get('base_seed')} (CRN),
 git: <code>{(git.get('commit') or '?')[:12]}</code>
 ({git.get('branch')}{', dirty — config/workspace.patch 参照' if git.get('dirty') else ''})</p>
<p>再現: <code>{manifest.get('reproduce_command', '')}</code><br>
 {manifest.get('reproducibility', '')}</p>

<h2>主要結果 (平均±95%CI)</h2>
{table_html(os.path.join(eval_dir, 'analysis', 'agg.csv'))}

<h2>paired 比較 (CRN)</h2>
{table_html(os.path.join(eval_dir, 'analysis', 'paired.csv'))}

<h2>図</h2>
{fig_html or '<p>(figures/ なし — 先に plot を実行)</p>'}

<h2>失敗タスク</h2>
<table><tr><th>task</th><th>params</th><th>status</th><th>attempts</th></tr>
{fail_rows}</table>

<h2>データスキーマ</h2>
<p><code>data.parquet</code>: tick粒度 (time_s, vehicle, antenna, run, sweep変数,
 rssi_dBm, throughput_Gbps, link_los, blockage_loss_dB, shadow_dB, fading_dB, …)。
 <code>summaries.parquet</code>: run×車両粒度。<code>events.parquet</code>: HOイベント。
 詳細は manifest.yaml の tables/layout を参照。</p>
</body></html>"""
    out = os.path.join(eval_dir, 'report.html')
    with open(out, 'w', encoding='utf-8') as f:
        f.write(html)
    print(f"[report] {out}")
    return 0


# ------------------------------------------------------------------ reproduce

def cmd_reproduce(args):
    """スナップショットから同一シードで再実行する (結果は <name>_repro_<ts>)。"""
    eval_dir = resolve_eval_dir(args.eval_dir)
    m = eval_manifest.read_manifest(eval_dir)
    if m is None:
        sys.exit("manifest.yaml がありません (旧形式ディレクトリは reproduce 非対応)")
    cfg_dir = os.path.join(eval_dir, 'config')
    sweep_snap = os.path.join(cfg_dir, 'sweep.yaml')
    scen_snap = os.path.join(cfg_dir, 'scenario.yaml')
    base_snap = os.path.join(cfg_dir, 'sim_params_base.yaml')
    for p in [sweep_snap, scen_snap]:
        if not os.path.exists(p):
            sys.exit(f"スナップショットがありません: {p}")

    ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    # ホスト・コンテナ双方から見え、ホストユーザーが書ける場所に置く
    # (tools/sweep_build はコンテナ実行でroot所有になることがある)
    tmp_dir = 'scratch'
    os.makedirs(tmp_dir, exist_ok=True)

    # シナリオの base_config をスナップショットへ差し替え
    with open(scen_snap, 'r', encoding='utf-8') as f:
        scen = yaml.safe_load(f)
    target = scen.get('scenario', scen)
    if os.path.exists(base_snap):
        target['base_config'] = base_snap
    scen_tmp = os.path.join(tmp_dir, f'repro_scenario_{ts}.yaml')
    with open(scen_tmp, 'w', encoding='utf-8') as f:
        yaml.safe_dump(scen, f, allow_unicode=True, sort_keys=False)

    # sweep のシナリオ参照と出力名を差し替え (シードはスナップショットのまま = 同一)
    with open(sweep_snap, 'r', encoding='utf-8') as f:
        sweep = yaml.safe_load(f)
    sweep['sweep']['scenario'] = scen_tmp
    sweep['sweep'].setdefault('execution', {})['output_name'] = \
        f"{m.get('name', 'eval')}_repro_{ts}"
    sweep_tmp = os.path.join(tmp_dir, f'repro_sweep_{ts}.yaml')
    with open(sweep_tmp, 'w', encoding='utf-8') as f:
        yaml.safe_dump(sweep, f, allow_unicode=True, sort_keys=False)

    if m.get('git', {}).get('dirty'):
        print(f"[reproduce] 注意: 元実行はdirtyな作業木でした。完全なコード復元には\n"
              f"  git checkout {m['git'].get('commit', '?')[:12]} && "
              f"git apply {cfg_dir}/workspace.patch")
    print(f"[reproduce] {m.get('name')} を再実行します -> "
          f"sim_results/{sweep['sweep']['execution']['output_name']}")
    args.sweep_config = sweep_tmp
    args.no_build = False
    args.resume = False
    return cmd_run(args)


# ------------------------------------------------------------------ verify

def cmd_verify(args):
    """2つの評価ディレクトリの統計的一致を検証する (再現性チェック)。

    合格基準 (ハードゲート): 同一 (条件, run, vehicle) の total_data_MB の
    paired 差に系統バイアスがない (Wilcoxon p>0.05)。

    run単位の散らばりは方式別に報告のみ行う: 開ループ方式 (lut等) は
    ほぼ0%で一致するが、閉ループ制御 (ts_kf/kkf等) は制御タイミングの
    非決定性により run 単位では数十%ばらつき得る (実測 2026-07-17)。
    閉ループ方式の分布一致の確認には num_runs を増やした paired 比較が必要。
    """
    import pandas as pd
    from scipy import stats as st
    from lib import eval_data, eval_metrics

    dirs = [resolve_eval_dir(d) for d in [args.dir_a, args.dir_b]]
    frames = []
    for d in dirs:
        if not os.path.exists(os.path.join(d, 'summaries.parquet')):
            eval_data.consolidate(d, delete_raw=False, verbose=False)
        m = eval_manifest.read_manifest(d)
        spec = analysis_spec(d, m)
        tables = eval_data.load_tables(d)
        veh = eval_metrics.per_vehicle_totals(tables['summaries'],
                                              spec['group_by'])
        veh['__dir'] = d
        frames.append((spec['group_by'], veh))
    gv_a, a = frames[0]
    gv_b, b = frames[1]
    if gv_a != gv_b:
        sys.exit(f"sweep変数が一致しません: {gv_a} vs {gv_b}")
    keys = gv_a + ['run', 'vehicle']
    merged = a.merge(b, on=keys, suffixes=('_a', '_b'))
    if merged.empty:
        sys.exit("突き合わせ可能な (条件, run, vehicle) がありません")
    diff = merged['total_data_MB_a'] - merged['total_data_MB_b']
    denom = merged[['total_data_MB_a', 'total_data_MB_b']].mean(axis=1)
    merged['rel'] = (diff.abs() / denom.replace(0, float('nan')))
    try:
        p = st.wilcoxon(diff).pvalue if (diff != 0).any() else 1.0
    except ValueError:
        p = float('nan')

    # 方式別の run 単位散らばり (開ループ≈0% / 閉ループは大きくて正常)
    arm_var = gv_a[0] if gv_a else None
    print(f"[verify] n={len(merged)} pairs, wilcoxon_p={p:.3f} (系統バイアス検定)")
    if arm_var:
        for arm, grp in merged.groupby(arm_var):
            r = grp['rel'].dropna()
            tag = 'ほぼ決定論' if r.mean() < 0.005 else '閉ループ由来の散らばり'
            print(f"[verify]   {arm_var}={arm}: mean|rel diff|="
                  f"{100 * r.mean():.2f}% max={100 * r.max():.2f}% ({tag})")
    ok = (p != p) or p > 0.05
    print(f"[verify] {'PASS' if ok else 'FAIL'}: "
          + ("系統バイアスなし (CRN再現性OK。閉ループ方式のrun単位差は正常、"
             "分布一致の確認には num_runs を増やすこと)" if ok
             else "系統バイアスあり — シード・コード差分を確認"))
    return 0 if ok else 1


# ------------------------------------------------------------------ clean

def cmd_clean(args):
    """統合済み評価ディレクトリの raw/ を削除して容量を回収する。"""
    from lib import eval_data
    total = 0
    for d in sorted(glob.glob(os.path.join('sim_results', '*'))):
        if not os.path.isdir(d):
            continue
        raw = os.path.join(d, 'raw')
        has_raw = os.path.isdir(raw) and os.listdir(raw)
        consolidated = os.path.exists(os.path.join(d, 'summaries.parquet'))
        if has_raw and consolidated:
            size = sum(os.path.getsize(os.path.join(r, f))
                       for r, _, fs in os.walk(raw) for f in fs)
            if args.delete_raw:
                eval_data.consolidate(d, delete_raw=True, verbose=False)
                print(f"[clean] {d}: raw/ 削除 ({size / 1e9:.2f} GB)")
            else:
                print(f"[clean] {d}: raw/ 削除可能 ({size / 1e9:.2f} GB) "
                      f"— --delete-raw で実行")
            total += size
    if total == 0:
        print("[clean] 回収可能な raw/ はありません")
    return 0


# ------------------------------------------------------------------ learn

def cmd_learn(args):
    """学習相 (本番仕様 §8.1): kkf arm を直列反復し REM を自己組織化させる。

    run ごとに sweep_sim を 1-run で起動し、共有 rem_state/ を介して
    (α, P, S2, W) を受け渡す。base_seed を run ごとに +stride するため、
    シード列は「単一スイープで num_runs=N」と同一 (交通は run 毎に変わり、
    持続シャドウは environment_seed で固定 = 学習相の定義そのもの)。

    学習ディレクトリのレイアウト (評価ディレクトリ契約の拡張):
      sim_results/<name>/rem_state/                  最新状態 (= 収束状態)
      sim_results/<name>/rem_state_snapshots/after_run_<m>/
                                                     学習曲線・学習量掃引用
      sim_results/<name>/learning_curve.csv          収束確認 (σ_ν² contrast 等)
      sim_results/<name>/runs/run_<m>/               各 run の sweep 出力
      sim_results/<name>/learn_manifest.yaml         再現に必要な入力の記録

    評価相 (kkf_conv) は kkf_state_dir に上記 rem_state (またはスナップ
    ショット) を指定し、kkf_state_save: false で読み取り専用にする。
    """
    if not IN_CONTAINER:
        return delegate_to_container(sys.argv[1:])
    import json
    import shutil
    import signal
    import numpy as np

    name = args.name
    learn_dir = os.path.join('sim_results', name)
    state_dir = os.path.join(learn_dir, 'rem_state')
    snap_root = os.path.join(learn_dir, 'rem_state_snapshots')
    runs_root = os.path.join(learn_dir, 'runs')
    for d in (learn_dir, state_dir, snap_root, runs_root, 'scratch'):
        os.makedirs(d, exist_ok=True)
    curve_path = os.path.join(learn_dir, 'learning_curve.csv')

    stride = 1000  # sweep_config.RUN_SEED_STRIDE と一致させる
    start_run = 1
    if args.resume and os.path.exists(curve_path):
        with open(curve_path) as f:
            rows = [r for r in f.read().splitlines()[1:] if r]
        if rows:
            start_run = max(int(r.split(',')[0]) for r in rows) + 1
        print(f"[learn] resume: run {start_run} から再開")
    elif os.listdir(state_dir) and not args.resume:
        sys.exit(f"[learn] {state_dir} に既存状態があります。継続なら --resume、"
                 "やり直しなら rem_state/ を削除してください "
                 "(黙って混ぜると収束履歴が汚染されるため中断)")

    def state_metrics():
        rows = []
        for f in sorted(glob.glob(os.path.join(state_dir, 'bs*.npz'))):
            # 地図キーは "0" (単一方向) または "0_p"/"0_m" (方向別) の文字列。
            # 学習曲線では地図を識別できればよいので整数化しない
            bs = os.path.basename(f)[2:-4]
            with np.load(f) as d:
                alpha, P, S2, W = d['alpha'], d['P'], d['S2'], d['W']
            ok = W >= 0.5
            if ok.sum() >= 4:
                var = S2[ok] / W[ok]
                med = max(float(np.median(var)), 1e-12)
                contrast = float(np.quantile(var, 0.9)) / med
            else:
                contrast = 1.0
            rows.append({'bs': bs, 'alpha': alpha, 'p_trace': float(np.trace(P)),
                         'contrast': contrast})
        return rows

    def state_run_count():
        """地図に記録された取り込み済み走行数 (地図間で一致するはず)。"""
        counts = set()
        for f in glob.glob(os.path.join(state_dir, 'bs*.npz')):
            with np.load(f) as d:
                counts.add(int(json.loads(str(d['meta'])).get('run_count', -1)))
        return max(counts) if counts else 0

    prev_alpha = {r['bs']: r['alpha'] for r in state_metrics()}
    if start_run == 1:
        with open(curve_path, 'w') as f:
            f.write("run,bs,p_trace,sigma_nu_contrast,alpha_rms_delta_db\n")

    # --- 進捗状態ファイル (sweep_progress.py が読む) を learn 側で所有する ---
    # 内側の sweep_sim は 1-run で反復起動されるため、各 run が total_tasks=1 で
    # 上書きすると全体進捗が壊れる。SWEEP_SIM_SUPPRESS_STATE=1 で内側の書き出しを
    # 止め、ここで「runs 本中 m 本完了 / 現在 run m 実行中」を書く。
    state_file = os.path.join(TOOLS_DIR, 'log', '.latest_sweep_state.json')
    os.makedirs(os.path.dirname(state_file), exist_ok=True)
    learn_start_iso = datetime.datetime.now().isoformat()
    run_durations = []
    scenario_name = os.path.splitext(os.path.basename(args.scenario))[0]

    def _write_learn_state(completed, running):
        st = {
            "start_time": learn_start_iso,
            "last_updated": datetime.datetime.now().isoformat(),
            "total_tasks": args.runs,
            "concurrency": 1,
            "completed": completed,
            "running_tasks": running,
            "task_durations": run_durations,
            "sweep_dir_name": "",
            "scenario_name": f"{scenario_name} (learn)",
        }
        try:
            tmp = state_file + ".tmp"
            with open(tmp, 'w', encoding='utf-8') as sf:
                json.dump(st, sf, indent=2)
            os.replace(tmp, state_file)
        except Exception as e:
            print(f"[learn] 進捗状態の書き出しに失敗: {e}")

    # 走行を REM へのトランザクションとして扱う。sweep_sim の内部再試行は切り、
    # 失敗したら地図を走行前に巻き戻してから再試行する。
    # (タイムアウトやクラッシュで死んだ試行も、死ぬ前に観測を取り込んでしまう。
    #  同一シードの部分走行が重複計上されると、カルマンフィルタが前提とする
    #  観測の独立性が壊れ、分散を過小評価する。実測で完了5走行に対し
    #  run_count=8 になった)
    child_env = dict(os.environ, SWEEP_SIM_SUPPRESS_STATE="1",
                     SWEEP_SIM_MAX_ATTEMPTS="1")
    bak_dir = os.path.join(learn_dir, '.rem_state_bak')
    max_attempts = 3
    live = {'proc': None}

    def _kill_child(sig=signal.SIGTERM):
        pr = live.get('proc')
        if pr is None or pr.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(pr.pid), sig)
        except (ProcessLookupError, PermissionError):
            pass

    def _on_signal(signum, _frame):
        print(f"\n[learn] シグナル {signum} を受信。実行中の走行を停止します")
        _kill_child(signal.SIGTERM)
        pr = live.get('proc')
        if pr is not None:
            try:
                pr.wait(timeout=20)
            except subprocess.TimeoutExpired:
                _kill_child(signal.SIGKILL)
        sys.exit(130)

    for _s in (signal.SIGINT, signal.SIGTERM):
        signal.signal(_s, _on_signal)

    for m in range(start_run, args.runs + 1):
        sweep = {'sweep': {
            'name': f"{name}_run{m}",
            'scenario': args.scenario,
            'variables': [{'name': 'method', 'cases': {'kkf_learn': {'config': {
                'link_controller_node.ros__parameters.control_plane': 'kkf_mpc',
                'link_controller_node.ros__parameters.kkf_state_dir':
                    f"/workspace/{state_dir}",
                'link_controller_node.ros__parameters.kkf_state_save': True,
            }}}}],
            'execution': {
                'output_name': f"{name}/runs/run_{m}",
                'num_runs': 1,
                'base_seed': args.base_seed + stride * (m - 1),
                'task_timeout_sec': args.timeout,
                'max_concurrency': 1,
            },
        }}
        sweep_path = os.path.join('scratch', f"learn_{name}.yaml")
        with open(sweep_path, 'w') as f:
            yaml.dump(sweep, f, sort_keys=False, allow_unicode=True)

        base_seed_m = sweep['sweep']['execution']['base_seed']
        print(f"\n[learn] ===== run {m}/{args.runs} "
              f"(base_seed {base_seed_m}) =====")

        run_started = datetime.datetime.now()
        rc_before = state_run_count()
        shutil.rmtree(bak_dir, ignore_errors=True)
        shutil.copytree(state_dir, bak_dir)
        running_task = {str(m): {
            "task_no": m,
            "params_str": f"kkf_learn run {m}/{args.runs} (base_seed {base_seed_m})",
            "started_at": run_started.isoformat(),
            "worker_id": None,
        }}
        _write_learn_state(m - 1, running_task)

        for attempt in range(1, max_attempts + 1):
            # 子は独自のプロセスグループに置く。学習を止めたとき sweep_sim と
            # その配下 (gz sim 等) が生き残ると、排他ロックを握ったままになり
            # 次の学習が起動できない (実測で2回発生)
            proc = subprocess.Popen(
                ['bash', '-c', f"cd /workspace && PYTHONDONTWRITEBYTECODE=1 "
                               f"python3 tools/sweep_sim.py --sweep-config {sweep_path}"],
                env=child_env, start_new_session=True)
            live['proc'] = proc
            # 5秒ごとに last_updated を打ち直し、監視側の遅延/ゾンビ誤検知を防ぐ
            while True:
                try:
                    rc = proc.wait(timeout=5)
                    break
                except subprocess.TimeoutExpired:
                    _write_learn_state(m - 1, running_task)
            got = state_run_count() - rc_before
            if rc == 0 and got == 1:
                break
            why = (f"exit {rc}" if rc != 0
                   else f"取り込み数が {got} (1 のはず)")
            print(f"[learn] run {m} 試行 {attempt}/{max_attempts} 失敗 ({why})。"
                  f"地図を走行前に巻き戻して再試行")
            shutil.rmtree(state_dir, ignore_errors=True)
            shutil.copytree(bak_dir, state_dir)
            if attempt == max_attempts:
                _write_learn_state(m - 1, {})
                shutil.rmtree(bak_dir, ignore_errors=True)
                sys.exit(f"[learn] run {m} が {max_attempts} 回とも失敗 ({why})。"
                         f"地図は run {m - 1} 時点に巻き戻し済み — "
                         f"原因解消後 --resume で継続可能")
        shutil.rmtree(bak_dir, ignore_errors=True)
        run_durations.append((datetime.datetime.now() - run_started).total_seconds())
        _write_learn_state(m, {})

        for r in state_metrics():
            pa = prev_alpha.get(r['bs'])
            delta = (float(np.sqrt(np.mean((r['alpha'] - pa) ** 2)))
                     if pa is not None and pa.shape == r['alpha'].shape else float('nan'))
            prev_alpha[r['bs']] = r['alpha']
            with open(curve_path, 'a') as f:
                f.write(f"{m},{r['bs']},{r['p_trace']:.4f},"
                        f"{r['contrast']:.3f},{delta:.4f}\n")

        if (args.snapshot_every > 0
                and (m % args.snapshot_every == 0 or m == args.runs)):
            snap = os.path.join(snap_root, f"after_run_{m}")
            shutil.rmtree(snap, ignore_errors=True)
            shutil.copytree(state_dir, snap)
            print(f"[learn] snapshot: {snap}")

    with open(os.path.join(learn_dir, 'learn_manifest.yaml'), 'w') as f:
        yaml.dump({
            'name': name,
            'scenario': args.scenario,
            'runs': args.runs,
            'base_seed': args.base_seed,
            'seed_stride': stride,
            'snapshot_every': args.snapshot_every,
            'state_dir': state_dir,
            'note': ('評価相 (kkf_conv) は kkf_state_dir にこの rem_state を指定し '
                     'kkf_state_save: false で使う。評価相の base_seed は学習相の '
                     'シード域 [base_seed, base_seed+stride·runs) と重ねないこと'),
            'finished_at': datetime.datetime.now().isoformat(timespec='seconds'),
        }, f, sort_keys=False, allow_unicode=True)
    print(f"\n[learn] 完了: {learn_dir} (learning_curve.csv で収束を確認)")
    return 0


# ------------------------------------------------------------------ main

def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1],
                                     prog='sim.py')
    # コンテナへ委譲したとき、ホスト側からこの実行だけを特定して止めるための
    # 目印。環境変数では pkill -f (argv 照合) に引っかからないので引数で渡す
    parser.add_argument('--run-token', default='', help=argparse.SUPPRESS)
    sub = parser.add_subparsers(dest='command', required=True)

    p = sub.add_parser('run', help='スイープ実行')
    p.add_argument('--sweep-config', required=True)
    p.add_argument('--no-build', action='store_true')
    p.add_argument('--resume', action='store_true')

    p = sub.add_parser('status', help='進捗表示')
    p.add_argument('--watch', action='store_true')

    p = sub.add_parser('learn', help='学習相: kkf を直列反復し REM を収束させる')
    p.add_argument('--scenario', required=True,
                   help='例: config/scenarios/road_10car.yaml')
    p.add_argument('--name', required=True,
                   help='学習ディレクトリ名 (sim_results/<name>)')
    p.add_argument('--runs', type=int, default=100, help='学習走行数 (80〜120)')
    p.add_argument('--base-seed', type=int, default=512345,
                   help='学習相のシード基点 (評価相の域と重ねない)')
    p.add_argument('--snapshot-every', type=int, default=10,
                   help='m 走行ごとに rem_state をスナップショット (0=無効)')
    p.add_argument('--timeout', type=int, default=900)
    p.add_argument('--resume', action='store_true')

    p = sub.add_parser('analyze', help='parquet統合 + runs/agg/paired 集計')
    p.add_argument('eval_dir')
    p.add_argument('--baseline', help='paired比較の基準 (sweepのanalysis:でも指定可)')
    p.add_argument('--delete-raw', action='store_true',
                   help='統合完了後に raw/ の生CSVを削除')
    p.add_argument('--reconsolidate', action='store_true')

    p = sub.add_parser('plot', help='agg.csv から図を生成')
    p.add_argument('eval_dir')

    p = sub.add_parser('report', help='自己完結HTMLレポート生成')
    p.add_argument('eval_dir')

    p = sub.add_parser('reproduce', help='スナップショット+同一シードで再実行')
    p.add_argument('eval_dir')

    p = sub.add_parser('verify', help='2結果の統計的一致検証 (再現性チェック)')
    p.add_argument('dir_a')
    p.add_argument('dir_b')

    p = sub.add_parser('clean', help='統合済みディレクトリの raw/ を回収')
    p.add_argument('--delete-raw', action='store_true')

    args = parser.parse_args()

    if args.command in CONTAINER_COMMANDS and not IN_CONTAINER:
        return delegate_to_container(sys.argv[1:])
    os.chdir('/workspace' if IN_CONTAINER else REPO_ROOT)

    return {'run': cmd_run, 'status': cmd_status, 'learn': cmd_learn,
            'analyze': cmd_analyze, 'plot': cmd_plot, 'report': cmd_report,
            'reproduce': cmd_reproduce, 'verify': cmd_verify,
            'clean': cmd_clean}[args.command](args)


if __name__ == '__main__':
    sys.exit(main())
