"""別PCでの結果整理・図表再描画・Word編集・記録再生用の軽量ZIPを作る。"""
import argparse
import ast
import csv
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
from zipfile import ZipFile,ZIP_DEFLATED


def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path);parser.add_argument('zip_path',type=Path)
    a=parser.parse_args();root=a.directory.resolve();dest=a.zip_path.resolve();repo=Path(__file__).resolve().parents[1]
    assert json.loads((root/'index.json').read_text())['status']=='complete'
    if dest.exists():raise FileExistsError('既存ZIPは上書きしません: '+str(dest))
    dest.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='comms_edit_export_') as temp:
        package=Path(temp)/'comms_sim_results_edit';package.mkdir()
        def copy(source,relative):
            target=package/relative;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,target)
        for name in ['analysis','figures','report','inputs','configs']:
            shutil.copytree(root/name,package/name)
        for name in ['speed_refinement','speed_30m']:
            for file in (root/name).glob('*.csv'):copy(file,Path(name)/file.name)
        copy(root/'speed_30m/completed','speed_30m/completed')
        for name in ['input_sha256.json','code_sha256.json','report_format_reference.docx',
                     'monitor_seek_verification.json','replay_window_support.json']:
            copy(root/'metadata'/name,Path('metadata')/name)
        # 絶対パスを含む実行履歴は、編集用ファイルと区別して保存する。
        copy(root/'index.json','metadata/original_campaign_index.json')
        shutil.copytree(repo/'measurements/tjx_remeasured_20261029',package/'reference/measurements')
        for name in json.loads((root/'metadata/code_sha256.json').read_text()):
            source=repo/name
            if source.suffix in ['.py','.cpp','.cc','.hpp'] and source.is_file():copy(source,Path('reference/source')/name)
        for name in ['monitor_hrcp_signals.py','hrcp_monitor_replay.py','hrcp_monitor_timeouts.py',
                     'hrcp_monitor_geometry.py','hrcp_monitor_legend.py','select_timeout_replays.py',
                     'summarize_raw_antenna_campaign.py','write_raw_antenna_report.py']:
            copy(repo/'tools'/name,Path('tools')/name)
        # 編集時にはDocker/ROS/Unix専用fcntlを読み込まない独立したデータ処理ヘルパー。
        helper='"""編集用データ処理。元ツールの純粋関数をそのまま収録。"""\nimport csv\nfrom pathlib import Path\n\n'
        for filename,names in [('raw_antenna_campaign_common.py',{'read_csv','typed'}),
                               ('run_start_position_sweep.py',{'write_csv','category'})]:
            source=(repo/'tools'/filename).read_text()
            for node in ast.parse(source).body:
                if isinstance(node,ast.FunctionDef) and node.name in names:helper+=ast.get_source_segment(source,node)+'\n\n'
        (package/'tools/raw_antenna_campaign_common.py').write_text(helper)
        monitor=package/'tools/monitor_hrcp_signals.py'
        source=monitor.read_text().replace("'IPAexGothic', 'DejaVu Sans'", "'IPAexGothic', 'MS Gothic', 'Yu Gothic', 'MS Mincho', 'DejaVu Sans'")
        monitor.write_text(source)
        selected=json.loads((root/'replays/index.json').read_text());omitted=[];catalog=[]
        allowed={'live_signals.json','monitor_replay.jsonl','config.yaml','summary.csv','timeout_events.csv',
                 'completed','timeseries.csv','representative_verification.json','statistical_verification.yaml',
                 'probability_bins.csv','provenance.json','verification.log'}
        for number,item in enumerate(selected,1):
            source=Path(item['output']);relative=source.relative_to(root)
            for file in source.iterdir():
                if not file.is_file():continue
                if file.name in allowed:copy(file,relative/file.name)
                else:omitted.append(dict(path=str(relative/file.name),bytes=file.stat().st_size,
                                         reason='編集・モニター再生に不要なフレーム詳細ログ、軌道重複、旧起動ファイル'))
            item['output']=relative.as_posix();item['replay']='open_monitor.py'
            item['config_path']=f"configs/{item['profile']}/{item['model']}.yaml"
            copy_manifest=package/relative/'representative_verification.json'
            copy_manifest.write_text(json.dumps(item,ensure_ascii=False,indent=2))
            catalog.append(dict(number=number,label=item['label'],speed_mps=item['speed_mps'],start_path_m=item['start_path_m'],
                model=item['model'],timeouts=item['timeouts'],first_connected_s=item['first_connected_s'],
                seed=item['seed'],output=relative.as_posix()))
        (package/'replays/index.json').write_text(json.dumps(selected,ensure_ascii=False,indent=2))
        (package/'replays/extra_index.json').write_text('[]\n')
        with (package/'replays/catalog.csv').open('w',newline='',encoding='utf-8-sig') as f:
            w=csv.DictWriter(f,fieldnames=list(catalog[0]));w.writeheader();w.writerows(catalog)
        selector=package/'tools/select_timeout_replays.py';source=selector.read_text()
        source=source.replace("        if item:subprocess.Popen(['bash',item.data(0,QtCore.Qt.UserRole)['replay']])",'''        if item:
            record=item.data(0,QtCore.Qt.UserRole)
            command=[sys.executable,str(root/'tools/monitor_hrcp_signals.py'),
                     str(root/record['output']/'live_signals.json'),'--replay','--autoplay','--window','full','--playback-speed','2']
            if record.get('replay_scope')=='initial_connection' and record['first_connected_s']>=0:
                command.append('--stop-at-connection')
            subprocess.Popen(command)''')
        selector.write_text(source)
        verification=json.loads((package/'analysis/replay_verification.json').read_text());verification['results']=selected
        (package/'analysis/replay_verification.json').write_text(json.dumps(verification,ensure_ascii=False,indent=2))
        # 既存モニターを任意の展開先から、Windows/Linux双方で起動する。
        (package/'open_monitor.py').write_text('''from pathlib import Path
import os,sys,tempfile
root=Path(__file__).resolve().parent
os.environ.setdefault('MPLCONFIGDIR',str(Path(tempfile.gettempdir())/'comms_results_matplotlib'))
sys.path.insert(0,str(root/'tools'))
from select_timeout_replays import main
sys.argv=[sys.argv[0],str(root),*sys.argv[1:]]
main()
''')
        (package/'redraw_figures.py').write_text('''from pathlib import Path
import sys
root=Path(__file__).resolve().parent
sys.path.insert(0,str(root/'tools'))
from raw_antenna_campaign_common import read_csv,typed
from summarize_raw_antenna_campaign import plot,boundaries
rows=[typed(r) for r in read_csv(root/'analysis/position_summary.csv')]
plot(root,rows,boundaries(rows))
print('図7枚とレポート用画像を再生成しました。')
''')
        (package/'rebuild_report.py').write_text('''from pathlib import Path
import argparse,sys
root=Path(__file__).resolve().parent
p=argparse.ArgumentParser(description='集計CSVからWordを再生成。直接編集したWordは上書きされます。')
p.add_argument('--pdf',action='store_true',help='LibreOfficeがPATHにある場合、PDFも再生成')
a=p.parse_args()
sys.path.insert(0,str(root/'tools'))
import write_raw_antenna_report as writer
if not a.pdf:
    writer.render=lambda path:print('Wordを生成しました。既存PDFは更新していません。Word等からPDFへ保存してください。')
sys.argv=[sys.argv[0],str(root),str(root/'metadata/report_format_reference.docx')]
writer.main()
''')
        (package/'requirements.txt').write_text('numpy\nmatplotlib\nPyYAML\nPyQt5\nlxml\nPillow\n')
        (package/'README_ja.txt').write_text('''別PC用：通信シミュレーション結果・編集パッケージ

最初に見るもの
  report/：編集可能なWord（.docx）と閲覧用PDF。
  figures/timeout_scatter_results.pdf：最終7図。各図のPNG/PDFとWord用画像も収録。
  analysis/speed_30m_summary.csv：開始30 m、速度1〜5 m/sを0.1 m/s刻みで比較（410条件）。
  analysis/position_summary.csv：開始距離の比較（3,315条件）。追従はantenna_mode=trackingで抽出。
  analysis/trials.csv：全142,185試行。集計前のTimeout・接続成否・接続時刻・seedを収録。
  analysis/*boundaries.csv：区分境界。fixed_speed_summary.csvは旧開始25 mの補助比較。
  replays/catalog.csv：再生可能な30記録の一覧。

報告の順序
  1. 従来4パターンの開始30 mでの速度比較。
  2. 追従2パターンの開始距離による追加検討（速度1〜5 m/s）。
  3. 双方向60 GHz・−7 dBmの十字固定（開始30 m）。
  図は左①しきい値、右②確率モデル。×は未接続。未接続の0回を0回接続と混同しない。
  図中タイトル・拡大図・単一系列の凡例は省略。条件は本文・キャプションを参照。

通常の編集
  Wordでdocxを開けば編集可能。PythonやROSは不要。
  CSVはUTF-8。Excelでは「データ→テキスト/CSVから」で読み込むと文字コードを指定できる。
  WordはMS明朝／Times New Romanを指定。フォント差で改ページが変わる場合がある。

Pythonを使う場合（Python 3.10以上、Windows/Linux）
  ZIP全体を展開し、展開先フォルダをターミナルで開く。
  python -m pip install -r requirements.txt
  python redraw_figures.py       集計CSVから図7枚を再描画
  python rebuild_report.py       Wordを再生成（既存Wordを上書き。手編集版は先に別名保存）
  python rebuild_report.py --pdf LibreOfficeがPATHにある場合のみ、PDFも再生成
  python open_monitor.py         30記録からモニターを選択して再生
  動作確認はUbuntu 22.04の別展開先で実施。Windows実機・macOSでは未検証。
  UbuntuでGUIが起動しない場合はQtの依存ライブラリを確認する。
  モニターの再生と図の再描画に、ROS/Gazebo/Dockerは不要。
  redraw_figures.pyは集計CSVを描画する。trials.csvを変更しても自動再集計はしない。
  図の外観はtools/summarize_raw_antenna_campaign.py内のplot、
  レポート本文はtools/write_raw_antenna_report.pyを編集できる。

収録範囲と軽量化
  events.csv / frames.csv（フレーム単位の詳細ログ）、Gazebo姿勢の重複ログ、
  OS依存バイナリ、重複するバッチ出力は収録しない。省略内容はmetadata/export_manifest.json。
  全試行の結果CSVと30件のコンパクトな再生記録は収録している。
  元の詳細ログは元PCに保存。フレーム判定を最初から再監査する場合は元PCの記録が必要。
  このZIPは結果編集用であり、ROS/Gazeboの新規シミュレーション環境一式ではない。
  現在の30件のモニターは速度・距離を分散選定した記録で、開始30 mに統一した集合ではない。
  成功試行の反復計算は初回リンク確立まで。未接続は最接近後60 mまで。

設定・出典
  inputs/、configs/：計算時の入力原本。YAML内に当時の/workspaceパスが残るが、
  編集用スクリプトと再生一覧は展開先から相対的に参照するので移動しても利用できる。
  reference/measurements/：校正に使用した測定CSVと処理記録。
  reference/source/：モデルと処理の参照用ソース（ビルド済み実行環境ではない）。
  metadata/input_sha256.json：元入力のハッシュ。
  metadata/package_sha256.json：このパッケージの収録ファイルのハッシュ。
  絶対RSSI校正、未測定方向への曲線共用等の仮定はレポートを参照。
''')
        filelist=sorted(p for p in package.rglob('*') if p.is_file())
        save=dict(created_at=datetime.now().astimezone().isoformat(),purpose='data_analysis_report_editing_and_replay',
            original_directory=str(root),source_trials=142185,replay_cases=30,
            omitted_replay_files=omitted,omitted_replay_bytes=sum(x['bytes'] for x in omitted),
            omitted_categories=['シミュレータのビルド成果物','重複するバッチ試行CSVと起動YAML','実行中の進捗/PID/ロックファイル'])
        (package/'metadata/export_manifest.json').write_text(json.dumps(save,ensure_ascii=False,indent=2))
        hashes={p.relative_to(package).as_posix():digest(p) for p in sorted(package.rglob('*')) if p.is_file()}
        (package/'metadata/package_sha256.json').write_text(json.dumps(hashes,ensure_ascii=False,indent=2))
        partial=dest.with_suffix('.zip.partial')
        with ZipFile(partial,'w',ZIP_DEFLATED,compresslevel=9) as z:
            for p in sorted(package.rglob('*')):
                if p.is_file():z.write(p,Path(package.name)/p.relative_to(package))
        with ZipFile(partial) as z:
            assert z.testzip() is None
            for name,h in hashes.items():assert hashlib.sha256(z.read(package.name+'/'+name)).hexdigest()==h
        partial.replace(dest)
        print(json.dumps(dict(zip=str(dest),zip_bytes=dest.stat().st_size,
            unpacked_bytes=sum(p.stat().st_size for p in package.rglob('*') if p.is_file()),
            files=len(hashes)+1,omitted_replay_bytes=save['omitted_replay_bytes'],sha256=digest(dest)),ensure_ascii=False,indent=2))


if __name__=='__main__':main()
