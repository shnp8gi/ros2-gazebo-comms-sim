"""
eval_data.py
------------
評価ディレクトリの生CSV群を tidy な parquet に統合する (L1: +pandas/pyarrow)。

生成物 (評価ディレクトリ直下):
  data.parquet       detailed_logs の全tick。メタデータ (run, sweep変数,
                     vehicle, antenna) はファイル名ではなく「列」として持つ
  events.parquet     ハンドオーバーイベント (run, sweep変数, vehicle 列付き)
  summaries.parquet  run別サマリ (sweep_summary_run*.csv の縦結合)
  controls.parquet   制御ログ (存在する場合のみ)

設計原則: 以後の分析 (eval_metrics / plot / report) はこれら parquet と
manifest.yaml だけから動く。ファイル名の正規表現解読はここで根絶する。

レイアウト互換: 新形式 (raw/ 配下) と旧形式 (直下に detailed_logs/ 等) の
両方を読める。sweep変数名は manifest.yaml か、無ければ旧 backup yaml から得る。
"""
import glob
import os
import re
import shutil

import pandas as pd
import yaml

from . import manifest as eval_manifest


# ---------------------------------------------------------------- 入力の解決

def raw_dir(eval_dir):
    """生CSVのルート (新形式 raw/、旧形式は評価ディレクトリ直下)。"""
    cand = os.path.join(eval_dir, 'raw')
    return cand if os.path.isdir(cand) else eval_dir


def sweep_variable_names(eval_dir):
    """sweep変数名のリスト (ファイル名サフィックス解読に使用)。"""
    m = eval_manifest.read_manifest(eval_dir)
    if m and m.get('variables'):
        return [v['name'] for v in m['variables']]
    for name in ['config/sweep.yaml', 'sweep_config_backup.yaml']:
        path = os.path.join(eval_dir, name)
        if os.path.exists(path):
            with open(path, 'r', encoding='utf-8') as f:
                data = (yaml.safe_load(f) or {}).get('sweep', {})
            return [v['name'] for v in data.get('variables', [])]
    return []


def tx_vehicle_names(eval_dir):
    """シナリオスナップショットから tx 車両名を得る (アンテナ名→車両名の対応用)。"""
    for name in ['config/scenario.yaml', 'scenario_config_backup.yaml']:
        path = os.path.join(eval_dir, name)
        if os.path.exists(path):
            with open(path, 'r', encoding='utf-8') as f:
                scen = yaml.safe_load(f) or {}
            scen = scen.get('scenario', scen)
            return [e['name'] for e in scen.get('entities', [])
                    if e.get('role') == 'tx']
    return []


def build_suffix_regex(var_names):
    """`method_kkf_full_density_2` 形式のタスクサフィックスを変数名で分解する
    正規表現。ラベルに `_` を含めるため、次の変数名を区切りに使う。"""
    if not var_names:
        return None
    parts = []
    for i, name in enumerate(var_names):
        parts.append(re.escape(name) + r'_(?P<' + name + r'>.+?)')
    return re.compile('^' + '_'.join(parts) + '$')


def vehicle_of_antenna(antenna, vehicles):
    """アンテナ名 (例 car_1_front) から車両名 (car_1) を最長前方一致で得る。"""
    best = ''
    for v in vehicles:
        if antenna == v or antenna.startswith(v + '_'):
            if len(v) > len(best):
                best = v
    return best or antenna


def parse_result_filename(filename, suffix_re):
    """ワーカー出力CSVのファイル名からメタデータを抽出する。

    形式: sweep_summary_<ts>_run<N>_<task_suffix>_w<W>_<残り>
    返り値: (meta_dict, 残り文字列)。解読不能なら (None, None)。
    """
    m = re.match(r'sweep_summary_\d{8}_\d{6}_run(\d+)_(.+?)_w(\d+)_(.+)$', filename)
    if not m:
        return None, None
    meta = {'run': int(m.group(1)), 'worker': int(m.group(3))}
    if suffix_re is not None:
        sm = suffix_re.match(m.group(2))
        if not sm:
            return None, None
        meta.update(sm.groupdict())
    return meta, m.group(4)


# ---------------------------------------------------------------- 統合本体

def _write_parquet_incremental(out_path, frames_iter):
    """DataFrameのイテレータを1つのparquetへ逐次書き込む (メモリ節約)。"""
    import pyarrow as pa
    import pyarrow.parquet as pq
    writer = None
    n_rows = 0
    try:
        for df in frames_iter:
            if df is None or df.empty:
                continue
            table = pa.Table.from_pandas(df, preserve_index=False)
            if writer is None:
                writer = pq.ParquetWriter(out_path, table.schema,
                                          compression='snappy')
            else:
                table = table.cast(writer.schema)
            writer.write_table(table)
            n_rows += len(df)
    finally:
        if writer is not None:
            writer.close()
    return n_rows


def consolidate(eval_dir, delete_raw=False, verbose=True, force=False):
    """生CSV群を parquet に統合する。統合結果の行数辞書を返す。

    既に統合済みのテーブル (parquet が存在) は force=False なら再構築しない
    (delete_raw だけを実行できる)。delete_raw=True なら、統合が完了した
    テーブルの生CSVディレクトリを削除する (manifest に記録)。
    """
    rdir = raw_dir(eval_dir)
    var_names = sweep_variable_names(eval_dir)
    suffix_re = build_suffix_regex(var_names)
    vehicles = tx_vehicle_names(eval_dir)
    counts = {}

    def log(msg):
        if verbose:
            print(f"[eval_data] {msg}", flush=True)

    def already(name):
        if not force and os.path.exists(os.path.join(eval_dir, f'{name}.parquet')):
            counts[name] = -1  # 既存 (行数は manifest / parquet 参照)
            log(f"{name}.parquet は既存のためスキップ (force=True で再構築)")
            return True
        return False

    # --- data.parquet (detailed_logs) ---
    logs = sorted(glob.glob(os.path.join(rdir, 'detailed_logs', '*_log.csv')))
    if logs and already('data'):
        logs = []
    if logs:
        def frames():
            for i, path in enumerate(logs):
                fname = os.path.basename(path)
                meta, rest = parse_result_filename(fname, suffix_re)
                if meta is None:
                    log(f"skip (名前解読不能): {fname}")
                    continue
                antenna = rest[:-len('_log.csv')]
                try:
                    df = pd.read_csv(path)
                except Exception as e:
                    log(f"skip (読込失敗): {fname}: {e}")
                    continue
                for k, v in meta.items():
                    df[k] = v
                df['antenna'] = antenna
                df['vehicle'] = vehicle_of_antenna(antenna, vehicles)
                if (i + 1) % 200 == 0:
                    log(f"detailed_logs {i + 1}/{len(logs)}")
                yield df
        counts['data'] = _write_parquet_incremental(
            os.path.join(eval_dir, 'data.parquet'), frames())
        log(f"data.parquet: {counts['data']} rows from {len(logs)} files")

    # --- events.parquet ---
    evs = sorted(glob.glob(os.path.join(rdir, 'events', '*_handover_events.csv')))
    if evs and already('events'):
        evs = []
    if evs:
        def ev_frames():
            for path in evs:
                fname = os.path.basename(path)
                meta, rest = parse_result_filename(fname, suffix_re)
                if meta is None:
                    continue
                try:
                    df = pd.read_csv(path)
                except Exception:
                    continue
                if df.empty:
                    continue
                for k, v in meta.items():
                    df[k] = v
                df['vehicle'] = rest[:-len('_handover_events.csv')]
                yield df.astype({'details': str})
        counts['events'] = _write_parquet_incremental(
            os.path.join(eval_dir, 'events.parquet'), ev_frames())
        log(f"events.parquet: {counts['events']} rows from {len(evs)} files")

    # --- controls.parquet (存在する場合のみ) ---
    ctls = sorted(glob.glob(os.path.join(rdir, 'controls', '*.csv')))
    if ctls and already('controls'):
        ctls = []
    if ctls:
        def ctl_frames():
            for path in ctls:
                meta, rest = parse_result_filename(os.path.basename(path), suffix_re)
                if meta is None:
                    continue
                try:
                    df = pd.read_csv(path)
                except Exception:
                    continue
                for k, v in meta.items():
                    df[k] = v
                df['source'] = rest
                yield df
        counts['controls'] = _write_parquet_incremental(
            os.path.join(eval_dir, 'controls.parquet'), ctl_frames())
        log(f"controls.parquet: {counts['controls']} rows")

    # --- summaries.parquet (マージ済み run別サマリ。sweep変数列は注入済み) ---
    runs = sorted(glob.glob(os.path.join(eval_dir, 'sweep_summary_run*.csv')))
    runs = [p for p in runs if re.search(r'run\d+\.csv$', p)]
    if runs and already('summaries'):
        runs = []
    if runs:
        def sum_frames():
            for path in runs:
                run = int(re.search(r'run(\d+)\.csv$', path).group(1))
                df = pd.read_csv(path)
                df['run'] = run
                # vehicle_name はアンテナ名 (car_1_front) または車両合算行
                # (car_1_total)。分析用に車両名と合算フラグを列に昇格する
                names = df['vehicle_name'].astype(str)
                df['is_total'] = names.str.endswith('_total')
                df['vehicle'] = [n[:-len('_total')] if t
                                 else vehicle_of_antenna(n, vehicles)
                                 for n, t in zip(names, df['is_total'])]
                yield df
        counts['summaries'] = _write_parquet_incremental(
            os.path.join(eval_dir, 'summaries.parquet'), sum_frames())
        log(f"summaries.parquet: {counts['summaries']} rows from {len(runs)} runs")

    # --- manifest へ記録 + raw削除 ---
    m = eval_manifest.read_manifest(eval_dir)
    if m is not None:
        m.setdefault('tables', {})
        for name, n in counts.items():
            if n == -1 and name in m['tables']:
                continue  # 既存スキップ分は記録を保持
            m['tables'][name] = {'rows': int(n), 'file': f'{name}.parquet'}
    if delete_raw and counts:
        for sub in ['detailed_logs', 'events', 'controls', 'summaries']:
            # summaries(ワーカー個別CSV)はマージ済みrun別CSVがあるときのみ削除
            if sub == 'summaries' and 'summaries' not in counts:
                continue
            key = {'detailed_logs': 'data'}.get(sub, sub)
            if key in counts or sub == 'summaries':
                target = os.path.join(rdir, sub)
                if os.path.isdir(target):
                    shutil.rmtree(target, ignore_errors=True)
        log("raw CSV (統合済み分) を削除")
        if m is not None:
            m['raw_deleted'] = True
    if m is not None:
        eval_manifest.write_manifest(eval_dir, m)
    return counts


def load_tables(eval_dir):
    """統合済みテーブルを読み込む。存在しないものは None。"""
    out = {}
    for name in ['data', 'events', 'controls', 'summaries']:
        path = os.path.join(eval_dir, f'{name}.parquet')
        out[name] = pd.read_parquet(path) if os.path.exists(path) else None
    return out
