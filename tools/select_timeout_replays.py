"""速度・パターン・モデルを絞り込み、バランス選定した代表記録を再生する。"""
import argparse
import json
from pathlib import Path
import subprocess
import sys
from PyQt5 import QtCore,QtWidgets


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path)
    p.add_argument('--render-once',action='store_true');p.add_argument('--screenshot',type=Path);a=p.parse_args();root=a.directory.resolve()
    app=QtWidgets.QApplication(sys.argv);w=QtWidgets.QWidget();w.setWindowTitle('試行のモニター — バランス選定30件')
    layout=QtWidgets.QVBoxLayout(w);status=QtWidgets.QLabel();layout.addWidget(status)
    filters=QtWidgets.QHBoxLayout();layout.addLayout(filters)
    choices=[]
    for title,values in [('パターン',['すべて','並行・固定','並行・追従','十字・固定','十字・追従','十字・固定（双方向60 GHz）']),
                         ('モデル',['すべて','① しきい値','② 確率']),('速度',['すべて','1 m/s','2 m/s','3 m/s','4 m/s','5 m/s'])]:
        filters.addWidget(QtWidgets.QLabel(title));combo=QtWidgets.QComboBox();combo.addItems(values);filters.addWidget(combo);choices.append(combo)
    extra=QtWidgets.QCheckBox('既存の補助記録も表示');layout.addWidget(extra)
    extra_path=root/'replays/extra_index.json'
    extra.setVisible(extra_path.exists() and bool(json.loads(extra_path.read_text())))
    tree=QtWidgets.QTreeWidget();tree.setHeaderLabels(['パターン','モデル','速度','開始距離','Timeout','接続','選定内容','seed'])
    tree.setRootIsDecorated(False);tree.setAlternatingRowColors(True);layout.addWidget(tree)
    note=QtWidgets.QLabel('各行は1試行の記録です。平均値は集計CSVを参照。追加の接続成功例は初回確立直後まで、未確立例は観測終了まで記録します。')
    note.setWordWrap(True);layout.addWidget(note);button=QtWidgets.QPushButton('選択した試行のモニターを再生');layout.addWidget(button)
    state={'signature':None}
    def refresh(force=False):
        path=root/'replays/index.json'
        if not path.exists():status.setText('再生記録を準備しています');return
        try:
            rows=json.loads(path.read_text());base_count=len(rows)
            extra_path=root/'replays/extra_index.json'
            if extra.isChecked() and extra_path.exists():rows+=json.loads(extra_path.read_text())
            signature=(path.stat().st_mtime_ns,extra.isChecked(),*(c.currentText() for c in choices))
            selection_progress=root/'metadata/replay_selection_progress.json'
            progress=json.loads(selection_progress.read_text()) if selection_progress.exists() else {}
            status.setText(f'保存済みの選定記録：{base_count}件'+
                (' ／ 30件の準備中（各速度6件・各パターン6件・①②各15件）' if base_count<30 else
                 ' ／ 各速度6件・各パターン6件・①②各15件')+
                (f"\n{progress.get('message','')}" if progress.get('status')!='complete' else ''))
            if not force and state['signature']==signature:return
            selected=tree.currentItem().data(0,QtCore.Qt.UserRole)['output'] if tree.currentItem() else None
            tree.clear()
            for r in rows:
                pattern=r['label'].split('／')[0];model='① しきい値' if r['model']=='threshold' else '② 確率';speed=f"{r['speed_mps']:g} m/s"
                if any(c.currentIndex()>0 and c.currentText()!=v for c,v in zip(choices,[pattern,model,speed])):continue
                item=QtWidgets.QTreeWidgetItem([pattern,model,speed,f"{r['start_path_m']:g} m",str(r['timeouts']),
                    '確立' if r['first_connected_s']>=0 else '未確立',r.get('selection','境界の補助記録'),str(r['seed'])])
                item.setData(0,QtCore.Qt.UserRole,r);tree.addTopLevelItem(item)
                if r['output']==selected:tree.setCurrentItem(item)
            if tree.currentItem() is None and tree.topLevelItemCount():tree.setCurrentItem(tree.topLevelItem(0))
            for i in range(8):tree.resizeColumnToContents(i)
            state['signature']=signature
        except (OSError,ValueError,KeyError):status.setText('再生一覧を更新しています')
    def play():
        item=tree.currentItem()
        if item:subprocess.Popen(['bash',item.data(0,QtCore.Qt.UserRole)['replay']])
    for c in choices:c.currentIndexChanged.connect(lambda _:refresh(True))
    extra.toggled.connect(lambda _:refresh(True));button.clicked.connect(play);tree.itemDoubleClicked.connect(lambda *_:play())
    timer=QtCore.QTimer();timer.timeout.connect(refresh);timer.start(2000);refresh();w.resize(1200,650);w.show()
    if a.render_once:
        def capture():
            if a.screenshot:w.grab().save(str(a.screenshot))
            app.quit()
        QtCore.QTimer.singleShot(500,capture)
    app.exec_()


if __name__=='__main__':main()
