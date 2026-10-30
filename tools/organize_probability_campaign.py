"""完了済み80条件を、モデル/シナリオ/速度の階層へ整理し参照を更新する。"""
import argparse
import csv
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    args=parser.parse_args();root=args.directory.resolve()
    root.relative_to(ROOT)
    index=json.loads((root/'index.json').read_text())
    if index.get('layout')=='model/scenario/speed':
        print('既に階層化済みです');return
    if index['status']!='complete' or len(index['runs'])!=80 or not all(r['status']=='complete' for r in index['runs']):
        raise ValueError('全80条件が完了してから整理してください')
    for file in ['timeout_comparison.png','timeout_comparison.pdf','timeout_comparison.svg','summary.csv','replay.py']:
        if not (root/file).exists():raise ValueError('集計処理が未完了です: '+file)
    movements=[];replacements={}
    def register(old,new):
        replacements[str(old)]=str(new)
        replacements['/workspace/'+str(old.relative_to(ROOT))]='/workspace/'+str(new.relative_to(ROOT))
    for row in index['runs']:
        old=root/row['name']
        model='model_1_threshold' if row['model']=='threshold' else 'model_2_statistical'
        new=root/model/f"{row['scenario']}_{row['antenna_mode']}"/f"{row['speed_mps']:02d}mps"
        if not old.is_dir() or new.exists():raise ValueError('移動元/移動先が不正: '+str(old))
        movements.append((row,old,new));register(old,new)
        register(root/'configs'/f"{row['name']}.yaml",new/'input.yaml')
        register(root/'logs'/f"{row['name']}.launch.log",new/'launch.log')
        register(root/'logs'/f"{row['name']}.verify.log",new/'verification.log')
    (root/'organization_plan.json').write_text(json.dumps(
        dict(moves=[dict(source=str(a),destination=str(b)) for _,a,b in movements],status='moving'),indent=2))
    index['status']='organizing'
    (root/'index.json').write_text(json.dumps(index,ensure_ascii=False,indent=2))
    for row,old,new in movements:
        new.parent.mkdir(parents=True,exist_ok=True)
        old.rename(new)
        for old_file,new_file in [(root/'configs'/f"{row['name']}.yaml",new/'input.yaml'),
                (root/'logs'/f"{row['name']}.launch.log",new/'launch.log'),
                (root/'logs'/f"{row['name']}.verify.log",new/'verification.log')]:
            old_file.rename(new_file)
        # 大容量CSV/再生JSONLは内容不変。パスを持つ小さな設定・再生ファイルだけ更新。
        for relative in ['config.yaml','input.yaml','world.sdf','replay.sh','provenance.json','inputs/receiver_curve.provenance.json']:
            f=new/relative
            if not f.exists():continue
            original=f.read_text();updated=original
            for a,b in sorted(replacements.items(),key=lambda pair:len(pair[0]),reverse=True):updated=updated.replace(a,b)
            if updated!=original:f.write_text(updated)
        row.update(output=str(new),replay=str(new/'replay.sh'),relative_output=str(new.relative_to(root)))
    analysis=root/'analysis';analysis.mkdir()
    for name in ['timeout_comparison.png','timeout_comparison.pdf','timeout_comparison.svg','summary.csv']:
        (root/name).rename(analysis/name)
    for name in ['delivery_windows.csv','timeouts_by_speed.csv']:
        if (root/name).exists():(root/name).rename(analysis/name)
    if (root/'selector_preview.png').exists():(root/'selector_preview.png').rename(analysis/'selector_preview.png')
    if (root/'ui_preview').exists():(root/'ui_preview').rename(analysis/'ui_preview')
    fields=['name','model','scenario','antenna_mode','speed_mps','duration_s','timeouts','timeouts_before_first_connection',
            'timeouts_total','timeouts_after_first_connection','connection_established','connections','disconnections',
            'attempts','total_data_MB','first_connected_s','connected_at_end','wall_time_s','output','replay','relative_output']
    with (analysis/'summary.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields,extrasaction='ignore');writer.writeheader();writer.writerows(index['runs'])
    delivery=analysis/'delivery_windows.csv'
    if delivery.exists():
        text=delivery.read_text()
        for a,b in sorted(replacements.items(),key=lambda pair:len(pair[0]),reverse=True):text=text.replace(a,b)
        delivery.write_text(text)
    index.update(layout='model/scenario/speed',analysis_directory='analysis',status='complete')
    (root/'index.json').write_text(json.dumps(index,ensure_ascii=False,indent=2))
    # 全ファイルを移動した後、空の補助ディレクトリだけ取り除く。
    meta=root/'metadata';meta.mkdir()
    for name in ['selector.log','selector.pid']:
        if (root/name).exists():(root/name).rename(meta/name)
    (root/'configs').rmdir()
    (root/'logs').rmdir()
    (root/'organization_plan.json').rename(meta/'organization_plan.json')
    plan=json.loads((meta/'organization_plan.json').read_text());plan['status']='complete'
    (meta/'organization_plan.json').write_text(json.dumps(plan,ensure_ascii=False,indent=2))
    replay='''#!/usr/bin/env python3
import argparse,json,subprocess
from pathlib import Path
p=argparse.ArgumentParser(description='①/②・シナリオ・速度を選んで再生')
p.add_argument('model',choices=['1','2']);p.add_argument('scenario',choices=['parallel','cross'])
p.add_argument('antenna',choices=['fixed','tracking']);p.add_argument('speed',type=int,choices=range(1,11))
a=p.parse_args();root=Path(__file__).resolve().parent
model='threshold' if a.model=='1' else 'statistical'
index=json.loads((root/'index.json').read_text())
row=next(r for r in index['runs'] if r['model']==model and r['scenario']==a.scenario and r['antenna_mode']==a.antenna and r['speed_mps']==a.speed)
subprocess.run(['bash',str(root/row['relative_output']/'replay.sh')],check=True)
'''
    (root/'replay.py').write_text(replay)
    readme=root/'README_ja.txt'
    text=readme.read_text().replace('index.jsonは進行状況と各結果へのパス。configsと各結果inputsに実行時設定を保存。',
        'index.jsonは各結果への索引。モデル/シナリオ/速度ごとに結果・input.yaml・launch.log・verification.log・inputsを保存。')
    readme.write_text('整理後の構造:\n analysis/ 比較図・集計CSV\n model_1_threshold/シナリオ/速度/ ①の40条件\n model_2_statistical/シナリオ/速度/ ②の40条件\n monitor.sh 選択画面\n replay.py モデル・シナリオ・速度を引数で再生\n metadata/ 整理履歴\n\n'+text)
    # 移動後も全条件の再生入口とCSV・設定が存在することを確認。
    for row in index['runs']:
        out=Path(row['output'])
        for name in ['replay.sh','live_signals.json','monitor_replay.jsonl','summary.csv','config.yaml','input.yaml','launch.log','verification.log']:
            assert (out/name).is_file(),out/name
        assert str(out/'live_signals.json') in (out/'replay.sh').read_text()
        assert str(out/'inputs') not in (out/'config.yaml').read_text()  # runtimeはコンテナ内パス
        assert ('/workspace/'+str(out.relative_to(ROOT))+'/inputs') in (out/'config.yaml').read_text()
    print('80条件を階層化し、再生・設定・集計の参照を更新しました:',root)


if __name__=='__main__':main()
