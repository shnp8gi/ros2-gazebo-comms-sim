"""速度境界の独立試行を集計する。図・表に信頼区間は表示しない。"""
import argparse
from collections import defaultdict
import csv
import json
from pathlib import Path
import statistics


def write_csv(path,rows):
    if not rows:return
    with path.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)


def stats(rows):
    values=[int(r['timeouts']) for r in rows];n=len(rows)
    return dict(trials=n,mean_timeouts=statistics.mean(values),min_timeouts=min(values),max_timeouts=max(values),
        connected_trials=sum(int(r['connected']) for r in rows),
        zero_timeout_connected_trials=sum(r['connected']=='1' and r['timeouts']=='0' for r in rows),
        zero_timeout_connection_fraction=sum(r['connected']=='1' and r['timeouts']=='0' for r in rows)/n,
        first_half_mean=statistics.mean(values[:max(1,n//2)]),
        last_half_mean=statistics.mean(values[n//2:]))


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path)
    a=p.parse_args();root=a.directory.resolve();output=root/'analysis'
    trials=[];seen=set()
    for batch in sorted((root/'batches').iterdir()):
        if not (batch/'completed').exists():continue
        parameters=json.loads((batch/'parameters.json').read_text())
        rows=list(csv.DictReader((batch/'trials.csv').open()))
        assert len(rows)==len(parameters['speeds'])*parameters['repetitions']
        for r in rows:
            key=(r['model'],r['scenario'],r['antenna_mode'],float(r['speed_mps']),int(r['seed']))
            assert key not in seen,(batch,key)
            seen.add(key);r['batch']=batch.name
            r['phase']='confirmation' if batch.name.startswith('confirm_') or batch.name=='cross_tracking_repeats' else 'exploration'
            trials.append(r)
    write_csv(output/'trials.csv',trials)
    grouped=defaultdict(list)
    for r in trials:grouped[(r['model'],r['scenario'],r['antenna_mode'],float(r['speed_mps']),r['phase'])].append(r)
    summaries=[];convergence=[]
    for (model,scenario,antenna,speed,phase),rows in sorted(grouped.items()):
        rows.sort(key=lambda r:int(r['seed']))
        key=dict(model=model,scenario=scenario,antenna_mode=antenna,speed_mps=speed,phase=phase)
        summaries.append(dict(**key,**stats(rows)))
        if phase=='confirmation':
            for n in sorted({x for x in [50,100,200,300,600,1000,len(rows)] if x<=len(rows)}):
                convergence.append(dict(**key,**stats(rows[:n])))
    write_csv(output/'speed_summary.csv',summaries)
    write_csv(output/'convergence.csv',convergence)
    confirmed=[s for s in summaries if s['phase']=='confirmation']
    write_csv(output/'confirmation_summary.csv',confirmed)
    # 既存の1〜10m/s全80条件も参照。独立反復の平均には混ぜない。
    baseline=list(csv.DictReader((output/'baseline_validation.csv').open()))
    estimates=[]
    qualifies=lambda s:s['trials']>=300 and s['mean_timeouts']<=.01 and s['zero_timeout_connection_fraction']>=.99
    for model in ['threshold','statistical']:
        for scenario,antenna in [('parallel','fixed'),('parallel','tracking'),('cross','fixed'),('cross','tracking')]:
            result=dict(model=model,scenario=scenario,antenna_mode=antenna,
                result='',last_nonzero_region_speed_mps=None,near_zero_region_speed_mps=None,
                mean_timeouts=None,zero_timeout_connection_fraction=None,trials=None,notes='')
            if scenario=='cross':
                if antenna=='fixed':
                    result.update(result='unconnected_not_a_zero_timeout_boundary',notes='1〜10 m/sで両モデル未試行・未接続。0回成功には含めない。')
                else:
                    result.update(result='already_zero_at_lowest_tested_speed',near_zero_region_speed_mps=1,
                        mean_timeouts=0,zero_timeout_connection_fraction=1,trials=300 if model=='statistical' else 1,
                        notes='1 m/sから0回で接続。1 m/s未満の境界は調べていない。②は1・10 m/sを各300回。')
            elif model=='threshold':
                result.update(result='no_zero_timeout_boundary_found',notes='1〜20 m/sの確認した速度で0回接続なし。最初のRequestが未達になる既存簡易MAC。')
            else:
                series=sorted([s for s in confirmed if (s['model'],s['scenario'],s['antenna_mode'])==(model,scenario,antenna)],key=lambda s:s['speed_mps'])
                # 一点だけ0になった場所を境界としない。その先の確認点もほぼ0であることを要求。
                good=next((s for i,s in enumerate(series) if all(qualifies(t) for t in series[i:])),None)
                if good:
                    lower=[s['speed_mps'] for s in series if s['speed_mps']<good['speed_mps'] and not qualifies(s)]
                    result.update(result='near_zero_boundary_bracket',last_nonzero_region_speed_mps=max(lower) if lower else None,
                        near_zero_region_speed_mps=good['speed_mps'],mean_timeouts=good['mean_timeouts'],
                        zero_timeout_connection_fraction=good['zero_timeout_connection_fraction'],trials=good['trials'],
                        notes='平均<=0.01回、0回で接続した割合>=99%。有限試行・確認した速度点の範囲。実機の境界精度ではない。')
                else:result.update(result='confirmation_incomplete_or_no_near_zero_point',notes='確認試行の完了後に再集計。')
            estimates.append(result)
    write_csv(output/'boundary_estimates.csv',estimates)
    plot(output,summaries,baseline)
    metadata=dict(trials=len(trials),statistical_trials=sum(r['model']=='statistical' for r in trials),
        definition='TIMEOUT events before first CONNECTED; unconnected runs counted until original 50/speed observation limit',
        setup_timeout_ms=500,confidence_intervals_displayed=False,
        maximum_confirmation_half_mean_difference=max((abs(s['first_half_mean']-s['last_half_mean']) for s in confirmed),default=None),
        near_zero_maximum_half_mean_difference=max((abs(s['first_half_mean']-s['last_half_mean']) for s in confirmed if qualifies(s)),default=None),
        confirmed_speeds=len(confirmed),all_completed_batches_validated=True)
    (output/'verification.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2))
    print(json.dumps(metadata,ensure_ascii=False))
    for s in confirmed:print(s)


def plot(output,summaries,baseline):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.ticker import ScalarFormatter,MaxNLocator
    plt.rcParams['font.family']='Noto Sans CJK JP'
    fig,axes=plt.subplots(2,2,figsize=(13,9),layout='constrained')
    patterns=[('parallel','fixed','並行・固定'),('parallel','tracking','並行・追従'),
              ('cross','fixed','十字・固定'),('cross','tracking','十字・追従')]
    for ax,(scenario,antenna,title) in zip(axes.flat,patterns):
        ax.set_title(title);ax.set_xlabel('各機の速度 [m/s]');ax.set_ylabel('初回リンク確立までの平均Timeout回数')
        ax.grid(alpha=.25);ax.xaxis.set_major_formatter(ScalarFormatter(useOffset=False))
        if (scenario,antenna)==('cross','fixed'):
            ax.text(.5,.56,'1〜10 m/s：両モデルともリンク未確立\nTimeout 0回は「未試行」\n0回で接続できる速度境界の対象外',ha='center',va='center',transform=ax.transAxes,fontsize=13)
            ax.set_xlim(1,10);ax.set_ylim(-.1,1.1);continue
        for model,label,color,marker in [('threshold','① 閾値','C0','o'),('statistical','② 実測PER＋仮定','C1','s')]:
            source=[s for s in summaries if (s['model'],s['scenario'],s['antenna_mode'])==(model,scenario,antenna)]
            selected={}
            for s in source:
                v=s['speed_mps']
                if v not in selected or s['phase']=='confirmation':selected[v]=s
            if scenario=='cross':
                for r in baseline:
                    v=float(r['speed_mps'])
                    if (r['model'],r['scenario'],r['antenna_mode'])==(model,scenario,antenna) and v not in selected:
                        selected[v]=dict(speed_mps=v,mean_timeouts=int(r['timeouts']),trials=1,connected_trials=int(r['connected']),phase='baseline')
            # 境界付近だけ表示。広域の探索値はCSVに保持。
            lo,hi=(3.5858,3.5882) if scenario=='parallel' and antenna=='fixed' else (10.6835,10.6905) if scenario=='parallel' else (1,10)
            series=[s for v,s in sorted(selected.items()) if lo<=v<=hi and s['connected_trials']>0]
            final=[s for s in series if s['phase']=='confirmation']
            if model=='threshold':
                ax.plot([s['speed_mps'] for s in series],[s['mean_timeouts'] for s in series],
                        color=color,marker=marker,markersize=4,label=label+'（各1回）')
            else:
                ax.plot([s['speed_mps'] for s in series],[s['mean_timeouts'] for s in series],
                        color=color,marker=marker,markersize=3,linestyle=':',alpha=.4,
                        label='② 探索を含む（探索10〜20回）' if scenario=='parallel' else '② 元の各1回＋確認点')
                if final:
                    ax.plot([s['speed_mps'] for s in final],[s['mean_timeouts'] for s in final],
                            color=color,marker=marker,markersize=5,linestyle='--',label='② 確認試行の平均')
            if final:
                ns=sorted({s['trials'] for s in final})
                note='② 確認点：'+', '.join(map(str,ns))+'回 / 速度'
                if scenario=='cross':note+='（1・10 m/s）'
                ax.text(.98,.96,note,transform=ax.transAxes,ha='right',va='top',fontsize=10)
            ax.set_xlim(lo-(hi-lo)*.02,hi+(hi-lo)*.02)
        ax.set_ylim(-.08,2.25 if scenario=='parallel' else 1.1)
        ax.yaxis.set_major_locator(MaxNLocator(integer=True));ax.legend(loc='center right' if scenario=='parallel' else 'upper left',fontsize=10)
    fig.suptitle('Timeoutが0回になる速度境界 — 初回リンク確立まで、待機500 ms',fontsize=16)
    fig.supxlabel('①は決定論、②は独立seedで反復。境界の小数桁は計算上の探索幅であり、実機の精度ではありません。\n①②ではMACの再送処理も異なります。未接続を「0回で接続」に含めません。',fontsize=10)
    for ext in ['png','pdf','svg']:fig.savefig(output/f'boundary_comparison.{ext}',dpi=180)
    plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(12,4.5),layout='constrained')
    for ax,antenna,title in zip(axes,['fixed','tracking'],['並行・固定','並行・追従']):
        s=sorted([r for r in summaries if r['model']=='statistical' and r['scenario']=='parallel' and r['antenna_mode']==antenna and r['phase']=='confirmation'],key=lambda r:r['speed_mps'])
        if s:
            ax.plot([r['speed_mps'] for r in s],[100*r['zero_timeout_connection_fraction'] for r in s],'o-',color='C1')
        ax.set(title=title,xlabel='各機の速度 [m/s]',ylabel='Timeout 0回で接続できた割合 [%]',ylim=(-3,103))
        ax.xaxis.set_major_formatter(ScalarFormatter(useOffset=False));ax.grid(alpha=.25)
    fig.suptitle('② 独立反復：Timeoutなしで接続できた割合')
    for ext in ['png','pdf','svg']:fig.savefig(output/f'zero_timeout_fraction.{ext}',dpi=180)
    plt.close(fig)


if __name__=='__main__':main()
