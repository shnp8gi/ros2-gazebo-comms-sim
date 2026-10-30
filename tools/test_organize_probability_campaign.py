"""整理によって結果内容や再生・コンテナ内パスを壊さないことを確認する。"""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import organize_probability_campaign as organize


class OrganizeTest(unittest.TestCase):
    def test_layout_preserves_results_and_rewrites_references(self):
        with tempfile.TemporaryDirectory() as directory:
            repo=Path(directory);root=repo/'campaign';root.mkdir()
            (root/'configs').mkdir();(root/'logs').mkdir()
            rows=[]
            for model in ['threshold','statistical']:
                for scenario in ['parallel','cross']:
                    for mode in ['fixed','tracking']:
                        for speed in range(1,11):
                            name=f'{model}_{scenario}_{mode}_{speed}mps';out=root/name;out.mkdir()
                            (out/'inputs').mkdir()
                            container='/workspace/campaign/'+name
                            (out/'config.yaml').write_text('table: '+container+'/inputs/table.csv\n')
                            (out/'world.sdf').write_text('<config>'+container+'/config.yaml</config>')
                            (out/'replay.sh').write_text('python viewer '+str(out/'live_signals.json'))
                            for file in ['live_signals.json','monitor_replay.jsonl','summary.csv']:
                                (out/file).write_bytes(b'untouched result\n')
                            (out/'inputs/table.csv').write_bytes(b'original curve\n')
                            (root/'configs'/f'{name}.yaml').write_text('source configuration\n')
                            (root/'logs'/f'{name}.launch.log').write_text('original launch log\n')
                            (root/'logs'/f'{name}.verify.log').write_text('original verification log\n')
                            rows.append(dict(name=name,status='complete',model=model,scenario=scenario,
                                antenna_mode=mode,speed_mps=speed,output=str(out),replay=str(out/'replay.sh')))
            (root/'index.json').write_text(json.dumps(dict(status='complete',runs=rows)))
            for name in ['timeout_comparison.png','timeout_comparison.pdf','timeout_comparison.svg','summary.csv','replay.py']:
                (root/name).write_text('existing artifact\n')
            (root/'README_ja.txt').write_text('説明\n')
            with patch.object(organize,'ROOT',repo),patch('sys.argv',['organize',str(root)]):
                organize.main()
                # 再実行でデータを再移動しない。
                organize.main()
            index=json.loads((root/'index.json').read_text())
            self.assertEqual(index['layout'],'model/scenario/speed')
            self.assertFalse((root/'configs').exists())
            self.assertEqual((root/'analysis/timeout_comparison.png').read_text(),'existing artifact\n')
            for row in index['runs']:
                out=Path(row['output'])
                self.assertEqual((out/'monitor_replay.jsonl').read_bytes(),b'untouched result\n')
                self.assertEqual((out/'inputs/table.csv').read_bytes(),b'original curve\n')
                self.assertIn('/workspace/'+str(out.relative_to(repo))+'/config.yaml',(out/'world.sdf').read_text())
                self.assertEqual((out/'launch.log').read_text(),'original launch log\n')
                self.assertFalse((root/row['name']).exists())


if __name__=='__main__':unittest.main()
