"""①/②、飛行シナリオ、速度を選択して保存済みモニターを再生する。"""
import argparse
import json
from pathlib import Path
import subprocess
import sys
from PyQt5 import QtCore, QtWidgets


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    parser.add_argument('--screenshot',type=Path)
    parser.add_argument('--render-once',action='store_true')
    args=parser.parse_args()
    root=args.directory.resolve()
    initial_index=json.loads((root/'index.json').read_text())
    available_patterns={(r['scenario'],r.get('antenna_mode','tracking')) for r in initial_index['runs']}
    app=QtWidgets.QApplication(sys.argv)
    window=QtWidgets.QWidget();window.setWindowTitle('①・② 速度別シミュレーションの再生')
    layout=QtWidgets.QVBoxLayout(window)
    title=QtWidgets.QLabel('モデル・飛行条件・速度を選んで再生')
    title.setStyleSheet('font-size:18px;font-weight:bold;padding:8px;');layout.addWidget(title)
    form=QtWidgets.QFormLayout();layout.addLayout(form)
    model=QtWidgets.QComboBox()
    model.addItem('① 受信電力の閾値 −68.5 dBm','threshold')
    model.addItem('② 実測PER＋未測定条件の仮定','statistical')
    scenario=QtWidgets.QComboBox()
    for name,value in [('並行・固定',('parallel','fixed')),('並行・追従',('parallel','tracking')),
                       ('十字・固定',('cross','fixed')),('十字・追従',('cross','tracking'))]:
        if value in available_patterns:scenario.addItem(name,value)
    speed=QtWidgets.QComboBox()
    for value in range(1,11):speed.addItem(f'{value} m/s（各機）',value)
    if initial_index.get('status')=='running':
        model.setCurrentIndex(1);speed.setCurrentIndex(9)
    form.addRow('モデル',model);form.addRow('シナリオ',scenario);form.addRow('飛行速度',speed)
    status=QtWidgets.QLabel();status.setWordWrap(True);layout.addWidget(status)
    play=QtWidgets.QPushButton('この条件をモニターで再生');layout.addWidget(play)
    plot=QtWidgets.QPushButton('Timeout比較グラフを開く');layout.addWidget(plot)
    boundary=QtWidgets.QPushButton('速度境界の代表試行をモニターで再生');layout.addWidget(boundary)
    note=QtWidgets.QLabel('Timeout＝初回リンク確立までの回数。未接続は観測終了まで。\n各機50 mの経路／各条件1回。①と②ではMAC処理も異なります。')
    if initial_index.get('policy')=='q_threshold_minus_lead_time':
        note.setText('Qが−68.5 dBmへ達する5秒前から開始。各条件1試行。\nTimeoutは初回リンク確立まで。①と②ではMAC処理も異なります。')
        boundary.setVisible(False)
    note.setWordWrap(True);layout.addWidget(note)
    selected={'row':None}
    def plot_path():
        organized=root/'analysis/timeout_comparison.png'
        return organized if organized.exists() else root/'timeout_comparison.png'
    def refresh():
        try:
            index=json.loads((root/'index.json').read_text())
            s,m=scenario.currentData()
            row=next(r for r in index['runs'] if r['model']==model.currentData() and r['scenario']==s
                     and r['antenna_mode']==m and r['speed_mps']==speed.currentData())
            selected['row']=row
            complete=row['status']=='complete' and index.get('status')!='organizing'
            play.setEnabled(complete)
            total=sum(r['status']=='complete' for r in index['runs'])
            if complete:
                status.setText(f"全体：{total}/{len(index['runs'])} 完了\n選択条件：初回確立までTimeout {row['timeouts']}回、接続 {row['connections']}回、切断 {row['disconnections']}回\n配送 {row['total_data_MB']:.3f} MB／観測 {row['duration_s']:.3f}秒"+
                               ('\n初回リンク未確立です。' if row['first_connected_s']<0 else '')+
                               ('\n観測終了時点でも接続状態です。' if row['connected_at_end'] else ''))
            else:status.setText(f"全体：{total}/{len(index['runs'])} 完了\n選択条件："+('フォルダ整理中。完了後に再生できます。' if index.get('status')=='organizing' else {'pending':'実行待ち','running':'計算・検証中','failed':'失敗（logsを確認）'}.get(row['status'],row['status'])))
            plot.setEnabled(plot_path().exists())
            boundary.setEnabled((root/'timeout_boundary/monitor.sh').exists())
        except (OSError,ValueError,KeyError,StopIteration) as exc:
            play.setEnabled(False);status.setText('記録を読み込めません：'+str(exc))
    def launch():
        refresh()
        row=selected['row']
        if not row or row['status']!='complete' or not play.isEnabled():return
        try:
            with (Path(row['output'])/'viewer.log').open('a') as log:
                subprocess.Popen(['bash',row['replay']],stdout=log,stderr=log,start_new_session=True)
        except OSError as exc:
            status.setText('再生を開始できません。整理完了後に再度お試しください：'+str(exc))
    play.clicked.connect(launch)
    plot.clicked.connect(lambda:subprocess.Popen(['xdg-open',str(plot_path())]))
    boundary.clicked.connect(lambda:subprocess.Popen(['bash',str(root/'timeout_boundary/monitor.sh')]))
    for combo in [model,scenario,speed]:combo.currentIndexChanged.connect(refresh)
    timer=QtCore.QTimer();timer.timeout.connect(refresh);timer.start(2000)
    refresh();window.resize(620,350);window.show()
    if args.render_once:
        def snapshot():
            if args.screenshot:window.grab().save(str(args.screenshot))
            app.quit()
        QtCore.QTimer.singleShot(300,snapshot)
    app.exec_()


if __name__=='__main__':main()
