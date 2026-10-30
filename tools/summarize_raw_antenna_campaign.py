"""生CSVによる50組の実結果を統合し、散布図・境界表・通常再生対象を作る。"""
import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path
from raw_antenna_campaign_common import read_csv, typed, category, write_csv

PATTERNS=[('original','parallel','fixed','並行・固定'),('original','parallel','tracking','並行・追従'),
          ('original','cross','fixed','十字・固定'),('original','cross','tracking','十字・追従'),
          ('dual60','cross','fixed','十字・固定（双方向60 GHz）')]
COLORS=['#0072b2','#dc8a00','#009e73','#ad69a0','#d55e00'];MARKERS=['o','s','^','D','v']


def key(r):return r['profile'],r['model'],r['scenario'],r['antenna_mode'],float(r['speed_mps']),float(r['position_m'])


def add_connection_timing(rows,trials,speed_refinement=False):
    groups=defaultdict(list)
    for r in trials:
        k=(r['profile'],r['model'],r['scenario'],r['antenna_mode'],float(r['speed_mps']),float(r['start_path_m']))
        groups[k,r['phase']].append(r)
    for r in rows:
        phases={'convergence','speed_fine' if speed_refinement else 'fine'} if r['phase']=='convergence' else {r['phase']}
        selected=[t for phase in phases for t in groups[key(r),phase]]
        assert len(selected)==r['trials'],('接続時刻の集計試行数',key(r),len(selected),r['trials'])
        times=[float(t['first_connected_s']) for t in selected if t['connected']=='1']
        r.update(connected_trials=len(times),first_connected_min_s=min(times) if times else None,
                 first_connected_max_s=max(times) if times else None,
                 first_connected_spread_ms=(max(times)-min(times))*1000 if times else None)


def boundaries(rows, axis='position_m'):
    groups=defaultdict(list)
    for r in rows:
        g=(r['profile'],r['model'],r['scenario'],r['antenna_mode'])
        if axis=='position_m':g+=(r['speed_mps'],)
        groups[g].append(r)
    out=[]
    for g,rs in sorted(groups.items()):
        rs.sort(key=lambda r:r[axis])
        for l,r in zip(rs,rs[1:]):
            if category(l)==category(r):continue
            out.append(dict(profile=g[0],model=g[1],scenario=g[2],antenna_mode=g[3],
                speed_mps=g[4] if axis=='position_m' else '',start_path_m=25 if axis!='position_m' else '',
                lower=l[axis],upper=r[axis],unit='m' if axis=='position_m' else 'm/s',
                lower_class=category(l),upper_class=category(r)))
    return out


def plot(root,rows,transitions):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    from matplotlib.backends.backend_pdf import PdfPages
    from matplotlib.lines import Line2D
    from matplotlib.ticker import MaxNLocator
    import numpy as np
    speed_rows=[typed(r) for r in read_csv(root/'analysis/fixed_speed_summary.csv')]
    speed30=[typed(r) for r in read_csv(root/'analysis/speed_30m_summary.csv')] if (root/'speed_30m/completed').exists() else []
    fonts={f.name for f in font_manager.fontManager.ttflist}
    jp=next((f for f in ['MS Mincho','ＭＳ 明朝','MS明朝'] if f in fonts),'Noto Serif CJK JP')
    latin=next((f for f in ['Times New Roman','TimesNewRomanPSMT'] if f in fonts),'Liberation Serif')
    plt.rcParams.update({'font.family':jp,'pdf.fonttype':3,'axes.labelsize':11,
        'mathtext.fontset':'custom','mathtext.rm':latin,'mathtext.it':latin+':italic',
        'mathtext.bf':latin+':bold','mathtext.cal':latin})
    folder=root/'figures';folder.mkdir(exist_ok=True)
    report_assets=root/'report/assets';report_assets.mkdir(parents=True,exist_ok=True)
    (folder/'fonts_used.json').write_text(json.dumps(dict(japanese=jp,latin=latin),ensure_ascii=False,indent=2))
    def draw(ax,subset,size,axis,series_key,series):
        for value,color,marker in zip(series,COLORS,MARKERS):
            rs=sorted([r for r in subset if r[series_key]==value],key=lambda r:r[axis])
            x=np.array([r[axis] for r in rs]);y=np.array([r['mean_timeouts'] for r in rs])
            connected=np.array([r['connected_fraction'] for r in rs]);good=connected>0
            ax.plot(x,np.where(good,y,np.nan),color=color,lw=.65,alpha=.6,zorder=2)
            ax.scatter(x[good],y[good],marker=marker,s=size,facecolors='none',edgecolors=color,linewidths=.95,zorder=3)
            ax.scatter(x[~good],y[~good],marker='x',s=size+5,color=color,linewidths=1,zorder=4)
            mixed=good & (connected<1)
            if mixed.any():ax.scatter(x[mixed],y[mixed],marker='o',s=size+40,facecolors='none',edgecolors='#666666',linewidths=.8,zorder=4)
        ax.grid(alpha=.2,color='#637385');ax.set_axisbelow(True);ax.tick_params(labelsize=10)
        for s in ax.spines.values():s.set_color('#a4aeb9')
    manifest=[]
    with PdfPages(folder/'timeout_scatter_results.pdf') as pdf:
        plans=[(*p,'speed_mps',30,'baseline_speed') for p in PATTERNS[:4]] + [
            (*p,'position_m',None,'tracking_distance') for p in PATTERNS if p[2]=='tracking'] + [
            (*PATTERNS[-1],'speed_mps',30,'dual60_comparison')] if speed30 else [
            (*p,'speed_mps' if p[2]=='fixed' else 'position_m',25 if p[2]=='fixed' else None,'legacy') for p in PATTERNS]
        for profile,scenario,mode,title,axis,start,section in plans:
            fixed=axis=='speed_mps'
            series_key='position_m' if fixed else 'speed_mps'
            series=[start] if fixed else list(range(1,6))
            source=(speed30 or speed_rows) if fixed else rows
            subset=[r for r in source if (r['profile'],r['scenario'],r['antenna_mode'])==(profile,scenario,mode)]
            assert subset
            top=max(1.5,max(r['mean_timeouts'] for r in subset)*1.15)
            fig,axes=plt.subplots(1,2,figsize=(14,6.5))
            fig.subplots_adjust(left=.075,right=.97,top=.96 if fixed else .88,bottom=.13,wspace=.17)
            if not fixed:
                handles=[Line2D([],[],color=c,marker=m,markerfacecolor='none',lw=.8,markersize=6,label=f'{v} m/s')
                         for v,c,m in zip(series,COLORS,MARKERS)]
                fig.legend(handles=handles,loc='upper center',bbox_to_anchor=(.49,1),ncol=5,frameon=False,
                           prop={'family':latin,'size':12},columnspacing=2.2)
            for model,ax in zip(['threshold','statistical'],axes):
                rs=[r for r in subset if r['model']==model];draw(ax,rs,19,axis,series_key,series)
                ax.set(xlim=(.9,5.1) if fixed else (0,61),ylim=(-top*.035,top),
                       xlabel=('各機の飛行速度 '+r'$\mathrm{[m/s]}$') if fixed else ('各機の最接近地点までの開始距離 '+r'$\mathrm{[m]}$'))
                if fixed:ax.set_xticks(range(1,6))
                ax.set_ylabel(('初回リンク確立までの' if model=='threshold' else '初回リンク確立までの平均')+r'$\mathrm{Timeout}$'+'回数')
                ax.yaxis.set_major_locator(MaxNLocator(nbins=6,integer=True))
            for ax in axes:
                for current in [ax,*ax.child_axes]:
                    for label in [*current.get_xticklabels(),*current.get_yticklabels()]:label.set_fontfamily(latin)
            name=f'{profile}_{scenario}_{mode}'+('_speed_30m' if fixed and speed30 else '')
            for ext in ['png','pdf']:fig.savefig(folder/f'{name}.{ext}',dpi=190,bbox_inches='tight')
            pdf.savefig(fig,bbox_inches='tight')
            # Wordの本文幅でも読めるよう、同じ左右配置を小さい用紙寸法で再描画する。
            fig.set_size_inches(7.2,5.2);fig.subplots_adjust(left=.115,right=.98,top=.98 if fixed else .9,bottom=.15,wspace=.32)
            for text in fig.texts:text.set_fontsize(13);text.set_position((.06,.95))
            for legend in fig.legends:
                for text in legend.get_texts():text.set_fontsize(9)
            for ax in axes:
                ax.title.set_fontsize(11);ax.xaxis.label.set_fontsize(8.5);ax.yaxis.label.set_fontsize(8.5)
                for current in [ax,*ax.child_axes]:
                    current.tick_params(labelsize=8 if current is ax else 7)
                    for collection in current.collections:
                        if hasattr(collection,'get_sizes'):collection.set_sizes(collection.get_sizes()*.65)
                    legend=current.get_legend()
                    if legend:
                        for text in legend.get_texts():text.set_fontsize(6.5)
                    for text in current.texts:text.set_fontsize(8)
            fig.savefig(report_assets/f'{name}.png',dpi=300,bbox_inches='tight');plt.close(fig)
            manifest.append(dict(profile=profile,scenario=scenario,antenna_mode=mode,title=title,file=f'figures/{name}.png',
                                 report_file=f'report/assets/{name}.png',x_axis=axis,
                                 fixed_start_path_m=start,study_section=section,zoom=None,
                                 titles_in_figure=False,single_series_legend=False,model_order=['threshold','statistical']))
    (folder/'index.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))


def select_replay_targets(root,summaries,limits,trials):
    # 5パターン×2モデル×3速度。各速度にちょうど6件を割り当てる。
    speed_sets=[(1,3,5),(1,2,4),(2,3,5),(1,3,4),(2,4,5)]
    targets=[]
    for (profile,scenario,mode,title),speeds in zip(PATTERNS,speed_sets):
        for model in ['threshold','statistical']:
            for rank,speed in enumerate(speeds):
                rs=[r for r in summaries if (r['profile'],r['model'],r['scenario'],r['antenna_mode'],r['speed_mps'])==
                    (profile,model,scenario,mode,float(speed))]
                if not rs:continue
                changes=[t for t in limits if (t['profile'],t['model'],t['scenario'],t['antenna_mode'],t['speed_mps'])==
                         (profile,model,scenario,mode,float(speed)) and 'near_zero' in [t['lower_class'],t['upper_class']]]
                pos=25.;selection='開始25 mの比較'
                if all(category(r)=='unconnected' for r in rs):
                    pos=[25.,10.,50.][rank];selection='未確立の確認'
                elif rank and changes:
                    change=changes[0];want_zero=rank==1
                    side='lower' if (change['lower_class']=='near_zero')==want_zero else 'upper'
                    pos=change[side];selection='0回側' if want_zero else '境界の反対側'
                s=next((r for r in rs if r['position_m']==pos),None)
                if s is None:continue
                candidates=[r for r in trials if (r['profile'],r['model'],r['scenario'],r['antenna_mode'],
                    float(r['speed_mps']),float(r['start_path_m']))==(profile,model,scenario,mode,float(speed),pos)]
                original=[r for r in candidates if r['phase']==s['phase']]
                if original:candidates=original
                trial=min(candidates,key=lambda r:(abs(int(r['timeouts'])-s['mean_timeouts']),int(r['seed'])))
                targets.append(dict(profile=profile,model=model,scenario=scenario,antenna_mode=mode,speed_mps=float(speed),
                    start_path_m=pos,end_path_m=60.,seed=int(trial['seed']),timeouts=int(trial['timeouts']),
                    first_connected_s=float(trial['first_connected_s']),source_trial=trial['source_component']+'/'+trial['id'],
                    config_path=str(root/'configs'/profile/f'{model}.yaml'),selection=selection,
                    replay_scope='initial_connection',
                    label=f"{title}／{'①' if model=='threshold' else '②'}／{speed} m/s／開始{pos:g} m"))
    return targets


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path);a=p.parse_args();root=a.directory.resolve()
    summaries=[];trials=[]
    index=json.loads((root/'index.json').read_text())
    for c in index['components']:
        assert c['status']=='complete'
        directory=Path(c['output']);source=str(directory.relative_to(root))
        summaries += [dict(typed(r),profile=c['profile'],speed_mps=float(c['speed_mps']),source_component=source)
                      for r in read_csv(directory/'analysis/position_summary.csv')]
        trials += [dict(r,profile=c['profile'],source_component=source,source_trial=r['id'])
                   for r in read_csv(directory/'analysis/trials.csv')]
    selected={key(r):r for r in summaries};assert len(selected)==len(summaries)
    refined=[typed(r) for r in read_csv(root/'speed_refinement/speed_summary.csv')]
    speed_rows={key(r):dict(r) for r in summaries if r['antenna_mode']=='fixed' and r['position_m']==25}
    for r in refined:speed_rows[key(r)]=dict(r,source_component='speed_refinement')
    for r in read_csv(root/'speed_refinement/convergence_summary.csv'):
        s=typed(r);source_phase=s.pop('source_phase')
        target=speed_rows if source_phase=='speed_fine' else selected
        target[key(s)].update(s)
    for filename,phase in [('trials.csv','speed_fine'),('convergence_trials.csv','convergence')]:
        trials += [dict(r,phase=phase,source_component='speed_refinement',source_trial=r['id'])
                   for r in read_csv(root/'speed_refinement'/filename)]
    speed30=[]
    if (root/'speed_30m/completed').exists():
        speed30=[typed(r) for r in read_csv(root/'speed_30m/speed_summary.csv')]
        trials += [dict(r,source_trial=r['id']) for r in read_csv(root/'speed_30m/trials.csv')]
    summaries=[selected[k] for k in sorted(selected)];speed_rows=[speed_rows[k] for k in sorted(speed_rows)]
    add_connection_timing(summaries,trials);add_connection_timing(speed_rows,trials,True)
    analysis=root/'analysis';write_csv(analysis/'position_summary.csv',summaries);write_csv(analysis/'trials.csv',trials)
    write_csv(analysis/'fixed_speed_summary.csv',speed_rows)
    if speed30:
        write_csv(analysis/'speed_30m_summary.csv',speed30)
        limits30=boundaries(speed30,'speed_mps')
        for r in limits30:r['start_path_m']=30
        write_csv(analysis/'speed_30m_boundaries.csv',limits30)
    limits=boundaries(summaries);speed_limits=boundaries(speed_rows,'speed_mps')
    write_csv(analysis/'position_boundaries.csv',limits);write_csv(analysis/'speed_boundaries.csv',speed_limits)
    # 全未接続・全0回の組も省略せず、境界未観測として表に残す。
    groups=defaultdict(list)
    for r in summaries:groups[key(r)[:5]].append(r)
    overview=[]
    for k,rs in sorted(groups.items()):
        changes=[t for t in limits if (t['profile'],t['model'],t['scenario'],t['antenna_mode'],t['speed_mps'])==k]
        overview.append(dict(profile=k[0],model=k[1],scenario=k[2],antenna_mode=k[3],speed_mps=k[4],
            conditions=len(rs),all_unconnected=all(category(r)=='unconnected' for r in rs),
            all_near_zero=all(category(r)=='near_zero' for r in rs),
            boundary_intervals_m='; '.join(f"{t['lower']:g}–{t['upper']:g}: {t['lower_class']} → {t['upper_class']}" for t in changes)))
    write_csv(analysis/'condition_overview.csv',overview)
    assert len(groups)==50
    seeds=[int(r['seed']) for r in trials];assert len(seeds)==len(set(seeds)),'乱数seed重複'
    for path,digest in json.loads((root/'metadata/input_sha256.json').read_text()).items():
        assert hashlib.sha256((root/path).read_bytes()).hexdigest()==digest,path
    audit=read_csv(root/'speed_refinement/convergence_audit.csv')
    audit+=read_csv(root/'speed_30m/convergence_audit.csv') if speed30 else []
    verification=dict(speed_30m_conditions=len(speed30),conditions=len(summaries),trials=len(trials),groups=len(groups),all_seeds_unique=True,
        input_checksums_match=True,speed_conditions=len(speed_rows),convergence_checked=len(audit),
        convergence_unstable=sum(r['stable']!='True' for r in audit))
    (analysis/'verification.json').write_text(json.dumps(verification,ensure_ascii=False,indent=2))
    repository=Path(__file__).resolve().parents[1]
    sources=[*sorted((repository/'src/comms_sim_pkg/src').glob('*.cpp')),
             repository/'src/comms_sim_pkg/src/DroneEncounterPlugin.cc',
             *sorted((repository/'src/comms_sim_pkg/include').rglob('*.hpp')),
             *sorted((repository/'tools').glob('*raw*campaign*.py')),
             repository/'tools/run_speed_30m_campaign.py',repository/'tools/run_start_position_sweep.py',repository/'tools/refine_raw_antenna_speed.py',
             repository/'tools/write_raw_antenna_report.py',repository/'tools/prepare_timeout_boundary_replays.py',
             repository/'tools/monitor_hrcp_signals.py',repository/'tools/hrcp_monitor_geometry.py',
             repository/'tools/hrcp_monitor_replay.py',repository/'tools/hrcp_monitor_timeouts.py',
             repository/'tools/prepare_balanced_replays.py',repository/'tools/select_timeout_replays.py',
             repository/'build/no_expected_rssi/comms_sim_pkg/initial_connection_sweep',
             repository/'install/no_expected_rssi/comms_sim_pkg/lib/comms_sim_pkg/plugins/DroneEncounterPlugin.so']
    (root/'metadata/code_sha256.json').write_text(json.dumps({str(f.relative_to(repository)):
        hashlib.sha256(f.read_bytes()).hexdigest() for f in sources},indent=2))
    plot(root,summaries,limits)
    targets=select_replay_targets(root,summaries,limits,trials)
    assert len(targets)==30, "バランス選定が30件になっていません"
    (root/'metadata/replay_targets.json').write_text(json.dumps(targets,ensure_ascii=False,indent=2))
    (root/'README_ja.txt').write_text(
        '生CSVアンテナ利得：速度・開始位置と初回リンク確立までのTimeout\n\n'
        'figures/：左①・右②。4パターンの開始30 m速度比較→追従2パターンの距離比較→双方向60 GHz十字固定の順。\n'
        'analysis/position_summary.csv：開始距離比較の平均・試行数・接続率。\n'
        'analysis/position_boundaries.csv：0回付近／未接続の区分が変わる隣接計算点。\n'
        'analysis/fixed_speed_summary.csv：開始25 m、固定アンテナの速度比較と境界。\n'
        'speed_refinement/convergence_audit.csv：600試行以上の前半・後半平均確認。\n'
        'analysis/trials.csv：全試行。source_componentとsource_trialで原結果を参照。\n'
        'monitor.sh：通常Gazeboで照合した代表条件の再生。\nprogress.sh：実行進捗の表示。\nreport/：指定書式のWord/PDF。\n\n'
        '①はRSSI≥−68.5 dBmで受信。②は既存の実測PER曲線を長さ補正して独立抽選。\n'
        'アンテナはE/H面CSV合成、正面利得への補正と30 dB打切りは使用しない。\n'
        '0回付近は平均≤0.01回かつ0回接続率≥99%。有限試行での判定であり厳密な0確率ではない。\n'
        '×は全試行未接続、灰色の輪は一部未接続。未接続の0回を接続成功と扱わない。\n'
        '未接続試行のTimeoutは終了位置（最接近後60 m）まで計数し、平均にも含める。\n'
        'replays内のsummary.csvは全区間の元集計。初回確立までの比較はanalysisとreplays/index.jsonを参照。\n'
        '1 m/s・1 mの粗探索で区分が変わった領域を0.1 m/s・0.1 mで確認する。\n'
        '探索点間の狭い非単調区間は未観測の可能性がある。確認範囲外には外挿しない。\n'
        '①と②はMAC実装も異なる。②の未測定方向への曲線共用と誤り独立性は仮定。\n'
        '図内タイトル・拡大図・単一系列の凡例・信頼区間・下部の灰色注釈は掲載しない。\n')
    print('統合集計完了:',verification,flush=True)


if __name__=='__main__':main()
