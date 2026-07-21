"""
eval_metrics.py
---------------
統合済みテーブル (data/events/summaries parquet) からの指標計算 (L1-L2)。

責務: 指標の定義と統計処理のみ。入力は eval_data.load_tables() の辞書、
出力は DataFrame。ファイルI/O・作図は持たない。

指標 (run × 条件グループ粒度):
  total_data_MB      全車合計転送量
  connected_time_s   全車合計接続時間
  min_car_data_MB    最小の車の転送量 (公平性: max-min)
  jain_index         Jain公平指数 (Σx)²/(n·Σx²)。1=完全公平
  <vehicle>_MB       車別転送量
  nlos_grant_pct     grant中tickのNLOS率 [%]
  ho_count           真のサービング切替回数 (measure遷移・プローブ復帰を除く)

注意: summaries CSV の handover_count は external_schedule ではプローブ切替も
数えるため使わない。ho_count は events の非measure BS遷移から数える (旧
analyze_multicar_eval.serving_ho_count と同一の定義)。
"""
import re

import numpy as np
import pandas as pd
from scipy import stats

_BS_TRANSITION = re.compile(r'bs\s+-?\d+\s+->\s+(-?\d+)')


def mean_ci(x, conf=0.95):
    """平均と95%CI半幅。n<2 は CI=0、n=0 は (nan, nan)。"""
    x = np.asarray([v for v in x if np.isfinite(v)], dtype=float)
    n = len(x)
    if n == 0:
        return np.nan, np.nan
    if n < 2:
        return float(x.mean()), 0.0
    half = stats.t.ppf(0.5 + conf / 2, n - 1) * x.std(ddof=1) / np.sqrt(n)
    return float(x.mean()), float(half)


def jain_index(values):
    x = np.asarray(list(values), dtype=float)
    x = x[np.isfinite(x)]
    if len(x) == 0 or (x ** 2).sum() == 0:
        return np.nan
    return float(x.sum() ** 2 / (len(x) * (x ** 2).sum()))


def per_vehicle_totals(summaries, group_vars):
    """summaries から (条件, run, vehicle) 粒度の転送量・接続時間を得る。

    2アンテナ以上の車両は *_total 行を採用。単一アンテナ車 (total行なし) は
    アンテナ行を車両で合算する (grantは車両あたり排他なので合算で正しい)。
    """
    keys = group_vars + ['run', 'vehicle']
    has_total = summaries.groupby(keys)['is_total'].transform('max')
    rows = summaries[summaries['is_total'] | ~has_total.astype(bool)]
    return rows.groupby(keys).agg(
        total_data_MB=('total_data_MB', 'sum'),
        connected_time_s=('connected_time_s', 'sum')).reset_index()


def nlos_grant_pct(data, group_vars):
    """grant中tickのNLOS率 [%] を (条件, run) 粒度で。"""
    g = data[data['has_link_grant'] == 1]
    if g.empty:
        return pd.DataFrame(columns=group_vars + ['run', 'nlos_grant_pct'])
    out = g.assign(nlos=(g['link_los'] == 0).astype(float)) \
        .groupby(group_vars + ['run'])['nlos'].mean().mul(100.0) \
        .rename('nlos_grant_pct').reset_index()
    return out


def serving_ho_count(events, group_vars):
    """真のサービング切替回数を (条件, run) 粒度で。

    (measure) 付き遷移は近傍プローブなので除外し、(アンテナ, 遷移先BS) の
    実効ターゲットが変化した回数を車両ごとに数えて合算する。
    """
    if events is None or events.empty:
        return pd.DataFrame(columns=group_vars + ['run', 'ho_count'])
    rows = []
    for key, grp in events.groupby(group_vars + ['run']):
        count = 0
        for _, veh_grp in grp.groupby('vehicle'):
            prev = None
            veh_grp = veh_grp.sort_values('time_s')
            for ant, details in zip(veh_grp['active_antenna'],
                                    veh_grp['details'].astype(str)):
                if '(measure)' in details:
                    continue
                m = _BS_TRANSITION.search(details)
                if not m:
                    continue
                target = (ant, int(m.group(1)))
                if prev is not None and target != prev:
                    count += 1
                prev = target
        key = key if isinstance(key, tuple) else (key,)
        rows.append(dict(zip(group_vars + ['run'], key), ho_count=count))
    return pd.DataFrame(rows)


def runs_table(tables, group_vars):
    """run粒度の指標表を構成する (分析の基本テーブル)。

    sweep変数列は「ラベル」なので、由来 (CSV注入=数値 / ファイル名解読=文字列)
    によらず str に正規化して突き合わせる。
    """
    tables = {name: df for name, df in tables.items()}
    for name, df in tables.items():
        if df is None:
            continue
        for v in group_vars:
            if v in df.columns:
                df[v] = df[v].astype(str)
    summaries = tables.get('summaries')
    if summaries is None or summaries.empty:
        raise ValueError('summaries.parquet がありません (consolidate 未実行?)')
    veh = per_vehicle_totals(summaries, group_vars)

    keys = group_vars + ['run']
    base = veh.groupby(keys).agg(
        total_data_MB=('total_data_MB', 'sum'),
        connected_time_s=('connected_time_s', 'sum'),
        min_car_data_MB=('total_data_MB', 'min')).reset_index()
    jain = veh.groupby(keys)['total_data_MB'].apply(jain_index) \
        .rename('jain_index').reset_index()
    base = base.merge(jain, on=keys, how='left')

    # 車別列 (<vehicle>_MB)
    wide = veh.pivot_table(index=keys, columns='vehicle',
                           values='total_data_MB', aggfunc='sum')
    wide.columns = [f'{c}_MB' for c in wide.columns]
    base = base.merge(wide.reset_index(), on=keys, how='left')

    data = tables.get('data')
    if data is not None and not data.empty:
        base = base.merge(nlos_grant_pct(data, group_vars), on=keys, how='left')
    events = tables.get('events')
    ho = serving_ho_count(events, group_vars)
    if not ho.empty:
        base = base.merge(ho, on=keys, how='left')
    return base


DEFAULT_METRICS = ['total_data_MB', 'connected_time_s', 'min_car_data_MB',
                   'jain_index', 'nlos_grant_pct', 'ho_count']


def aggregate(runs_df, group_vars, metrics=None):
    """条件グループ粒度の 平均±95%CI 表。"""
    metrics = [m for m in (metrics or DEFAULT_METRICS) if m in runs_df.columns]
    rows = []
    for key, grp in runs_df.groupby(group_vars):
        key = key if isinstance(key, tuple) else (key,)
        row = dict(zip(group_vars, key))
        row['n'] = len(grp)
        for met in metrics:
            m, h = mean_ci(grp[met])
            row[f'{met}_mean'] = m
            row[f'{met}_ci95'] = h
        rows.append(row)
    return pd.DataFrame(rows).sort_values(group_vars).reset_index(drop=True)


def paired(runs_df, baseline, arm_var, group_vars, metrics=None):
    """CRN前提の paired 差分 (baseline − 各方式) と Wilcoxon 符号順位検定。

    baseline: 基準となる arm_var の値 (例 'kkf_full')
    group_vars: arm_var 以外の条件変数 (例 ['density'])
    """
    metrics = [m for m in (metrics or DEFAULT_METRICS) if m in runs_df.columns]
    rows = []
    cond_keys = group_vars if group_vars else [None]
    grouped = runs_df.groupby(group_vars) if group_vars else [(None, runs_df)]
    for cond, grp in grouped:
        cond = cond if isinstance(cond, tuple) else (cond,)
        ref = grp[grp[arm_var] == baseline].set_index('run')
        for arm in grp[arm_var].unique():
            if arm == baseline:
                continue
            other = grp[grp[arm_var] == arm].set_index('run')
            common = ref.index.intersection(other.index)
            if len(common) < 2:
                continue
            for met in metrics:
                diff = (ref.loc[common, met] - other.loc[common, met]).dropna()
                if len(diff) < 2:
                    continue
                m, h = mean_ci(diff)
                try:
                    p = stats.wilcoxon(diff).pvalue if (diff != 0).any() else 1.0
                except ValueError:
                    p = np.nan
                row = {}
                if group_vars:
                    row.update(dict(zip(group_vars, cond)))
                row.update({'baseline': arm, 'metric': met,
                            'n_pairs': int(len(diff)),
                            'diff_mean': m, 'diff_ci95': h, 'wilcoxon_p': p})
                rows.append(row)
    return pd.DataFrame(rows)
