"""計算済みの条件からバランス選定30件を準備し、通常Gazeboで初回確立まで記録する。"""
import argparse
import io
import csv
import json
from pathlib import Path
import subprocess
import sys
import time
from raw_antenna_campaign_common import read_csv,typed
from run_start_position_sweep import summarize
from summarize_raw_antenna_campaign import select_replay_targets,boundaries


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path);p.add_argument('--jobs',type=int,default=1)
    a=p.parse_args();root=a.directory.resolve();progress=root/'metadata/replay_selection_progress.json'
    def status(**value):
        temp=progress.with_suffix('.tmp');temp.write_text(json.dumps(value,ensure_ascii=False,indent=2));temp.replace(progress)
    while True:
        summaries=[];trials=[]
        for c in json.loads((root/'index.json').read_text())['components']:
            directory=Path(c['output']);source=str(directory.relative_to(root))
            if c['status']=='complete':
                ss=[typed(r) for r in read_csv(directory/'analysis/position_summary.csv')];tt=read_csv(directory/'analysis/trials.csv')
            else:
                tt=[]
                for path in sorted((directory/'batches/coarse_1m').glob('results_*.csv')):
                    data=path.read_bytes();data=data[:data.rfind(b'\n')+1]
                    tt.extend(dict(r,phase='coarse') for r in csv.DictReader(io.StringIO(data.decode())))
                ss=[dict(r,phase='coarse') for r in summarize(tt,'path') if r['trials']==(1 if r['model']=='threshold' else 20)]
            summaries += [dict(r,profile=c['profile'],speed_mps=float(c['speed_mps']),source_component=source) for r in ss]
            trials += [dict(r,profile=c['profile'],source_component=source) for r in tt]
        targets=select_replay_targets(root,summaries,boundaries(summaries),trials)
        if len(targets)==30:break
        status(status='waiting',available_conditions=len(targets),total=30,
               message=f'30件のうち{len(targets)}件の条件が確定。残りの1 m/s条件の計算を待っています。')
        print('バランス選定:',len(targets),'/30 条件確定',flush=True);time.sleep(10)
    from collections import Counter
    assert Counter(t['speed_mps'] for t in targets)=={float(v):6 for v in range(1,6)}
    path=root/'metadata/replay_targets.json';temp=path.with_suffix('.tmp');temp.write_text(json.dumps(targets,ensure_ascii=False,indent=2));temp.replace(path)
    status(status='recording',total=30,message='30件を選定済み。追加記録と初回接続・Timeoutの照合を進めています。')
    subprocess.run([sys.executable,str(Path(__file__).with_name('prepare_timeout_boundary_replays.py')),str(root),'--jobs',str(a.jobs)],check=True)
    status(status='complete',total=30,message='30件を再生できます。')


if __name__=='__main__':main()
