"""生CSV利得による計算・検証・レポート作成の進捗を表示する。"""
import argparse
from collections import deque
from datetime import datetime,timedelta
import json
from pathlib import Path
import subprocess
import sys
import time
from PyQt5 import QtCore,QtWidgets


PHASES={'prepare':'入力準備','position_sweeps':'速度・開始位置の計算',
        'speed_30m':'開始30 m・速度0.1 m/s刻みの追加比較',
        'fixed_speed_refinement':'固定アンテナの速度境界を追加計算',
        'plots_and_summary':'図と集計表の作成','gazebo_replays':'通常Gazeboでの照合・再生保存',
        'report':'Wordレポートの作成','complete':'全工程完了'}


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('directory',type=Path)
    parser.add_argument('--screenshot',type=Path);parser.add_argument('--render-once',action='store_true');args=parser.parse_args()
    root=args.directory.resolve();app=QtWidgets.QApplication(sys.argv);window=QtWidgets.QWidget()
    window.setWindowTitle('シミュレーションの進捗 — CSV利得・双方向60 GHz追加')
    layout=QtWidgets.QVBoxLayout(window)
    title=QtWidgets.QLabel('本シミュレーションの進捗');title.setStyleSheet('font-size:21px;font-weight:bold;padding:6px;');layout.addWidget(title)
    overview=QtWidgets.QLabel('読み込み中');overview.setWordWrap(True);layout.addWidget(overview)
    current=QtWidgets.QLabel();current.setWordWrap(True);layout.addWidget(current)
    bar=QtWidgets.QProgressBar();layout.addWidget(bar)
    eta=QtWidgets.QLabel();layout.addWidget(eta)
    table=QtWidgets.QTreeWidget();table.setHeaderLabels(['無線条件','各機速度','状態','工程','完了試行数'])
    table.setRootIsDecorated(False);table.setAlternatingRowColors(True);layout.addWidget(table)
    detail=QtWidgets.QLabel('①しきい値／②確率モデル。正面利得補正なし、30 dBの減衰打切りなし。\n追加の十字固定：PRDEV・PRCとも60 GHz・−7 dBm、LNAなし。')
    detail.setWordWrap(True);layout.addWidget(detail)
    buttons=QtWidgets.QHBoxLayout();layout.addLayout(buttons)
    folder=QtWidgets.QPushButton('結果フォルダを開く');buttons.addWidget(folder)
    folder.clicked.connect(lambda:subprocess.Popen(['xdg-open',str(root)]))
    log=QtWidgets.QPushButton('実行ログを開く');buttons.addWidget(log)
    log.clicked.connect(lambda:subprocess.Popen(['xdg-open',str(root/'metadata/speed_30m_execution.log'
        if (root/'metadata/speed_30m_execution.log').exists() else root.parent/'metadata/raw_csv_campaign_execution.log')]))
    history={}
    def refresh():
        try:
            d=json.loads((root/'index.json').read_text());phase=d['phase'];state=d['status'];components=d['components']
            completed=sum(c['status']=='complete' for c in components)
            elapsed=(datetime.now().astimezone()-datetime.fromisoformat(d.get('speed_30m_started_at',d['started_at']))).total_seconds()
            overview.setText(f"工程：{PHASES.get(phase,phase)}　／　経過 {int(elapsed//60)}分\n速度・無線条件の組：{completed}/{len(components)} 完了（①②・計5パターン×速度1〜5 m/s）")
            overview.setStyleSheet('color:#b00020;' if state=='failed' else 'color:#13724b;' if state=='complete' else '')
            table.clear();total=0;active=None
            for c in components:
                progress={};path=Path(c['output'])/'progress.json'
                if path.exists():
                    try:progress=json.loads(path.read_text())
                    except json.JSONDecodeError:pass
                done=c.get('trials',progress.get('trials',progress.get('done',0)));total+=done
                label={'pending':'待機','running':'計算中','complete':'完了'}.get(c['status'],c['status'])
                step={'coarse_1m':'1 m刻みの探索','fine_01m':'境界0.1 m・反復確認'}.get(progress.get('phase',''),'')
                if c['status']=='complete':step='探索・境界確認 完了'
                item=QtWidgets.QTreeWidgetItem(['従来の無線条件（4パターン）' if c['profile']=='original' else '双方向60 GHz（十字固定）',
                    f"{c['speed_mps']} m/s",label,step,f"{done:,}"+(f" / {progress['total']:,}" if 'total' in progress and c['status']!='complete' else '')])
                table.addTopLevelItem(item)
                if c['status']=='running':active=(c,progress,step)
            for i in range(5):table.resizeColumnToContents(i)
            if state=='failed':
                current.setText('エラー：'+d.get('error','ログを確認してください'));bar.setRange(0,1);bar.setValue(0);eta.setText('エラーの確認・修正が必要です。');return
            if phase=='speed_30m':
                directory=root/'speed_30m';p=json.loads((directory/'progress.json').read_text()) if (directory/'progress.json').exists() else {}
                plan=json.loads((directory/'plan.json').read_text()) if (directory/'plan.json').exists() else {}
                name=p.get('phase','準備');n=p.get('done',0);limit=p.get('total',0)
                caption='全速度の計算' if name=='uniform_01mps' else '0回付近・境界の600試行確認' if name=='zero_region_600' else '平均の安定性確認'
                current.setText(f'開始30 m・5パターン×2モデル×41速度 ／ {caption}：{n:,} / {limit:,} 試行')
                overview.setText(f'工程：{PHASES[phase]} ／ 今回の経過 {int(elapsed//60)}分\n従来4パターンの速度比較 → 追従の距離比較 → 双方向60 GHz・十字固定')
                bar.setRange(0,limit or 0);bar.setValue(n)
                samples=history.setdefault(('speed_30m',name),deque(maxlen=90));samples.append((time.time(),n))
                dt=samples[-1][0]-samples[0][0];dn=samples[-1][1]-samples[0][1]
                if dt>=15 and dn>0:remaining=(limit-n)*dt/dn
                elif name=='uniform_01mps':remaining=plan.get('estimated_initial_seconds',0)*(1-n/max(1,limit))
                else:remaining=plan.get('estimated_followup_seconds',0)*(1-n/max(1,limit))
                if name=='uniform_01mps':remaining+=plan.get('estimated_followup_seconds',0)
                remaining+=plan.get('estimated_postprocess_seconds',120)
                now=datetime.now();lower=now+timedelta(seconds=remaining*.7);upper=now+timedelta(seconds=remaining*1.5)
                eta.setText(f'全体の推定終了：{lower:%H:%M}〜{upper:%H:%M} 頃（JST）\n境界反復・図・レポートを含む概算。追加確認数と実行速度で更新します。')
                return
            if active:
                c,p,step=active;n=p.get('done',0);limit=p.get('total',0)
                current.setText(f"現在：{'従来条件' if c['profile']=='original' else '双方向60 GHz'} ／ {c['speed_mps']} m/s ／ {step}")
                bar.setRange(0,limit or 0);bar.setValue(n)
                key=(c['profile'],c['speed_mps'],p.get('phase'));samples=history.setdefault(key,deque(maxlen=90));samples.append((time.time(),n))
                dt=samples[-1][0]-samples[0][0];dn=samples[-1][1]-samples[0][1]
                if dt>=20 and dn>0 and limit>n:
                    remaining=(limit-n)*dt/dn
                    eta.setText(f'現在の工程の残り目安：約{max(1,round(remaining/60))}分（実測速度から更新。後続工程は含みません）')
                else:eta.setText('現在の工程の残り時間を計測中。境界の数により後続の試行数が変わります。')
            else:
                current.setText(PHASES.get(phase,phase));bar.setRange(0,1 if state=='complete' else 0);bar.setValue(1 if state=='complete' else 0)
                eta.setText('図・CSV・再生・レポートを保存しました。' if state=='complete' else '後処理の進捗は工程名と実行ログで確認できます。')
        except (OSError,KeyError,ValueError) as exc:current.setText('進捗情報の更新待ち：'+str(exc))
    timer=QtCore.QTimer();timer.timeout.connect(refresh);timer.start(2000);refresh()
    window.resize(1000,660);window.show()
    if args.render_once:
        def capture():
            if args.screenshot:window.grab().save(str(args.screenshot))
            app.quit()
        QtCore.QTimer.singleShot(500,capture)
    app.exec_()


if __name__=='__main__':main()
