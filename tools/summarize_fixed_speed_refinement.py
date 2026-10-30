"""固定アンテナの0.1m/s追加試行を平均化し、速度境界を保存する。"""
import argparse
from collections import defaultdict
import csv
import json
from pathlib import Path
import statistics


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path)
    a=p.parse_args();root=a.directory.resolve();rows=[]
    for directory in sorted((root/'batches').iterdir()):
        assert (directory/'completed').exists(),directory
        with (directory/'trials.csv').open() as f:rows.extend(csv.DictReader(f))
    grouped=defaultdict(list)
    for r in rows:grouped[r['model'],r['scenario'],r['antenna_mode'],float(r['speed_mps'])].append(r)
    summary=[]
    for (model,scenario,mode,speed),rs in sorted(grouped.items()):
        rs.sort(key=lambda r:int(r['seed']));values=[int(r['timeouts']) for r in rs];n=len(rs)
        summary.append(dict(model=model,scenario=scenario,antenna_mode=mode,speed_mps=speed,trials=n,
            mean_timeouts=statistics.mean(values),connected_fraction=sum(int(r['connected']) for r in rs)/n,
            zero_timeout_connection_fraction=sum(r['connected']=='1' and r['timeouts']=='0' for r in rs)/n,
            first_half_mean=statistics.mean(values[:max(1,n//2)]),last_half_mean=statistics.mean(values[n//2:])))
    for name,data in [('trials',rows),('speed_summary',summary)]:
        with (root/'analysis'/f'{name}.csv').open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(data[0]));w.writeheader();w.writerows(data)
    assert len(rows)==6611 and len(summary)==22
    statistical=sorted([r for r in summary if r['model']=='statistical'],key=lambda r:r['speed_mps'])
    near_zero=[r for r in statistical if r['mean_timeouts']<=.01 and r['zero_timeout_connection_fraction']>=.99]
    upper=min(r['speed_mps'] for r in near_zero)
    lower=max(r['speed_mps'] for r in statistical if r['speed_mps']<upper)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams['font.family']='Noto Sans CJK JP'
    fig,axes=plt.subplots(1,2,figsize=(12,5),layout='constrained')
    for model,label,marker in [('threshold','① 閾値（各1回）','o'),('statistical','② 実測PER＋仮定（各600回）','s')]:
        rs=[r for r in summary if r['model']==model]
        axes[0].plot([r['speed_mps'] for r in rs],[r['mean_timeouts'] for r in rs],marker=marker,label=label)
    axes[0].set(title='並行・固定：境界周辺を0.1 m/s刻みで確認',xlabel='各機の速度 [m/s]',ylabel='初回リンク確立までの平均Timeout回数',xticks=[3+i*.1 for i in range(11)],ylim=(-.1,2.3))
    axes[0].grid(alpha=.25);axes[0].legend()
    axes[1].set_title('十字・固定')
    axes[1].text(.5,.5,'既存の1〜10 m/sではリンク未確立\nTimeout 0回は未試行によるもの\n0回接続の速度境界として扱わない',ha='center',va='center',transform=axes[1].transAxes)
    axes[1].set_axis_off()
    fig.supxlabel(f'固定アンテナは従来の開始位置（各機、最接近点まで25 m）を維持。\n②のほぼ0回接続への切替は、この0.1 m/s格子では{lower:g}〜{upper:g} m/sの間。',fontsize=10)
    for ext in ['png','pdf','svg']:fig.savefig(root/'analysis'/f'fixed_speed_timeout.{ext}',dpi=180)
    plt.close(fig)
    (root/'analysis/verification.json').write_text(json.dumps(dict(trials=len(rows),statistical_trials=6600,threshold_conditions=11,
        speed_step_mps=.1,timeout_scope='before first connection',independent_seed_trials=True,
        statistical_near_zero_lower_mps=lower,statistical_near_zero_upper_mps=upper),indent=2))
    print('固定アンテナ0.1m/sの比較図・平均表を保存しました',flush=True)


if __name__=='__main__':main()
