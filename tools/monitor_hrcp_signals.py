#!/usr/bin/env python3
"""PRDEV(A)/PRC(B)の受信RSSIと接続状態を表示するローカルモニター。"""
import argparse
import csv
import json
import os
from pathlib import Path
import sys
import time
import yaml
from bisect import bisect_right
from hrcp_monitor_timeouts import TimeoutEvents, TimeoutHighlight
from hrcp_monitor_replay import format_event_time
from hrcp_monitor_geometry import positions, antenna_angles, direction_styles, xor_unmet, judgement_state, JUDGEMENT_LABELS, JUDGEMENT_COLORS
from hrcp_monitor_legend import MovingLegends
os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib-hrcp-live')
import numpy as np
from PyQt5 import QtCore, QtWidgets
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib import font_manager
from matplotlib.figure import Figure
from matplotlib.ticker import FuncFormatter, ScalarFormatter
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('snapshot', type=Path)
    parser.add_argument('--screenshot', type=Path)
    parser.add_argument('--view', choices=('rssi','power','iq','a_tx','b_tx'), default='rssi', help=argparse.SUPPRESS)
    parser.add_argument('--render-once', action='store_true', help='保存済みsnapshotを描画して終了')
    parser.add_argument('--replay', action='store_true', help='保存したモニターを時間送り・一時停止して再生')
    parser.add_argument('--playback-speed', choices=('0.00001','0.0001','0.001','0.01','0.1','1','2','5','10'), default='1', help='再生速度の初期値')
    parser.add_argument('--autoplay', action='store_true', help='再生モードで先頭から自動再生')
    parser.add_argument('--guided-replay', action='store_true', help='通し試験の場面別速度で再生する（計算時間は変更しない）')
    parser.add_argument('--stop-at-connection', action='store_true', help='最初のリンク確立が完了した時点で再生を止める')
    parser.add_argument('--details', action='store_true', help='受信波形RSSIなどの補助表示も開く')
    parser.add_argument('--window', choices=('setup','full'), default='setup')
    args = parser.parse_args()
    config_path = args.snapshot.parent/'config.yaml'
    initial_config=yaml.safe_load(config_path.read_text()) if config_path.exists() else {}
    guided_segments=initial_config.get('guided_replay',[])
    if args.guided_replay and (not args.replay or not guided_segments):
        parser.error('--guided-replayには再生記録と場面別速度の設定が必要です')
    replay=None
    if args.replay:
        from hrcp_monitor_replay import MonitorReplay
        replay=MonitorReplay(args.snapshot)
    if args.stop_at_connection:
        if not replay:
            parser.error('--stop-at-connectionには--replayが必要です')
        connection=next((row[0] for row in replay.rows['setup'] if row[3]),None)
        if connection is None:
            parser.error('記録にリンク確立がありません')
        replay.duration=(connection+1)/replay.hz
    playback={'seconds':0.,'wall':time.monotonic()}
    timeout_events=TimeoutEvents(args.snapshot.parent)
    timeout_highlight=TimeoutHighlight(hold_seconds=2.0)
    timeout_display={'active':None,'count':0,'times':[],'loaded_menu_count':-1}
    first_connected_s=None
    summary_path=args.snapshot.parent/'summary.csv'
    if summary_path.exists():
        with summary_path.open() as stream:
            summary=next(csv.DictReader(stream),{})
        value=float(summary.get('first_connected_s',-1))
        if value>=0:first_connected_s=value
    if first_connected_s is None and replay:
        first_connected_s=next((row[0]/replay.hz for row in replay.rows['setup'] if row[3]),None)

    def jump_to(seconds, sample=None):
        if replay:
            pause.setChecked(True)
            seek_to(replay.after_event(seconds,sample))
            previous['drawn']=None

    stage_first={}
    if replay:
        for sample,name in replay.rows['stage']:
            stage_first.setdefault(name,sample)

    for font in ('Noto Sans CJK JP', 'Noto Sans CJK SC', 'IPAexGothic', 'DejaVu Sans'):
        if any(f.name == font for f in font_manager.fontManager.ttflist):
            matplotlib.rcParams['font.family'] = font
            break
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle('Fusion')
    window = QtWidgets.QMainWindow()
    window.setWindowTitle('ドローン通信 — 方向別判定・接続段階')
    widget = QtWidgets.QWidget()
    layout = QtWidgets.QVBoxLayout(widget)
    title = QtWidgets.QLabel('方向別判定・航跡モニター')
    title.setToolTip('A = PRDEV（Pairnet Device：端末）、B = PRC（Pairnet Coordinator：調整局）。Q=A→B、P=B→A。片方向だけ不充足の場合に強調。')
    title.setStyleSheet('font-size:17px; font-weight:bold; padding:6px;')
    layout.addWidget(title)
    roles = QtWidgets.QLabel('')
    roles.setStyleSheet('font-size:14px; padding:2px 6px;')
    layout.addWidget(roles)
    controls = QtWidgets.QHBoxLayout()
    mode = QtWidgets.QComboBox()
    mode.addItems(['リンク確立区間', '直近5 ms', '直近10 ms', '直近100 ms', '直近1秒', '全区間', '直近10秒'])
    if args.window=='full':
        mode.setCurrentIndex(5)
    pause = QtWidgets.QPushButton('表示を一時停止')
    pause.setCheckable(True)
    reset_route = QtWidgets.QPushButton('俯瞰を全体表示')
    details = QtWidgets.QPushButton('詳細')
    details.setCheckable(True)
    status = QtWidgets.QLabel('シミュレータの開始を待っています…')
    status.setWordWrap(True)
    controls.addWidget(mode)
    controls.addWidget(pause)
    controls.addWidget(reset_route)
    controls.addWidget(details)
    controls.addWidget(status, 1)
    layout.addLayout(controls)
    seek=None
    guided=None
    if replay:
        replay_controls=QtWidgets.QHBoxLayout()
        replay_controls.addWidget(QtWidgets.QLabel('再生位置 [秒]'))
        seek=QtWidgets.QDoubleSpinBox();seek.setDecimals(9);seek.setRange(0,replay.duration);seek.setSingleStep(.001)
        slider=QtWidgets.QSlider(QtCore.Qt.Horizontal);slider.setRange(0,100000)
        speed=QtWidgets.QComboBox();speed.addItems(['0.00001×','0.0001×','0.001×','0.01×','0.1×','1×','2×','5×','10×']);speed.setCurrentText(args.playback_speed+'×')
        beginning=QtWidgets.QPushButton('先頭へ')
        replay_controls.addWidget(seek);replay_controls.addWidget(slider,1);replay_controls.addWidget(speed);replay_controls.addWidget(beginning)
        layout.addLayout(replay_controls)
        pause.setChecked(not args.autoplay);pause.setText('一時停止' if args.autoplay else '再生')
        def seek_to(value):
            playback['seconds']=value;playback['wall']=time.monotonic()
            seek.blockSignals(True);seek.setValue(value);seek.blockSignals(False)
            slider.blockSignals(True);slider.setValue(round(value/replay.duration*100000));slider.blockSignals(False)
        seek.valueChanged.connect(seek_to)
        slider.valueChanged.connect(lambda value:seek_to(value*replay.duration/100000))
        beginning.clicked.connect(lambda:seek_to(0))
        if guided_segments:
            guided=QtWidgets.QCheckBox('通し確認（場面ごとに再生速度を変更）')
            guided.setChecked(args.guided_replay)
            speed.setEnabled(not args.guided_replay)
            guided.toggled.connect(lambda value:speed.setEnabled(not value))
            guided.toggled.connect(lambda value:playback.update(wall=time.monotonic()))
            replay_controls.addWidget(guided)
        verification_path=args.snapshot.parent/'verification.yaml'
        if initial_config.get('evaluation_scope')=='protocol_fault_injection' and verification_path.exists():
            verified=yaml.safe_load(verification_path.read_text())
            checkpoint_times=[('P=1 / Q=0', (verified['first_request_failure_s']+initial_config['fault_injection']['q_recovery_s'])/2),
                       ('接続・転送', (verified['connected_s']+initial_config['fault_injection']['both_outage_s'])/2),
                       ('通信断', initial_config['fault_injection']['both_outage_s']+.001),
                       ('切断直前', verified['disconnected_s']-1e-5),
                       ('切断後', min(replay.duration,verified['disconnected_s']+.001))]
            checkpoints=QtWidgets.QHBoxLayout()
            for caption,seconds in checkpoint_times:
                button=QtWidgets.QPushButton(caption)
                button.clicked.connect(lambda checked=False,t=seconds:(pause.setChecked(True),seek_to(t)))
                checkpoints.addWidget(button)
            layout.addLayout(checkpoints)
    phase = QtWidgets.QLabel('接続段階を確認中…')
    phase.setStyleSheet('font-size:17px; font-weight:bold; padding:6px; background:#183544; color:white;')
    phase.setWordWrap(True)
    layout.addWidget(phase)
    stage_row = QtWidgets.QHBoxLayout()
    stage_labels = []
    for name,caption in zip(('BEACON','REQUEST','RESPONSE','ACK','DATA'),
                           ('① Beacon探索', '② Request準備・送信', '③ Response待ち', '④ ACK確認', '⑤ データ通信')):
        sample=stage_first.get(name)
        stamp=sample/replay.hz if sample is not None else None
        label=QtWidgets.QPushButton(caption+(f'\n初回 {format_event_time(stamp)}' if stamp is not None else '\n記録なし' if replay else ''))
        label.setObjectName('stage_jump_'+name)
        label.setEnabled(stamp is not None if replay else False)
        label.setToolTip('クリックすると初回の状態遷移直後へ移動して一時停止します。' if stamp is not None else 'この記録には該当する状態がありません。')
        if stamp is not None:label.clicked.connect(lambda checked=False,t=stamp,s=sample:jump_to(t,s))
        stage_row.addWidget(label,1)
        stage_labels.append(label)
    timeout_button=QtWidgets.QToolButton()
    timeout_button.setObjectName('timeout_jump')
    timeout_button.setText('Timeout：0回')
    timeout_button.setPopupMode(QtWidgets.QToolButton.MenuButtonPopup)
    timeout_menu=QtWidgets.QMenu(timeout_button);timeout_button.setMenu(timeout_menu)
    timeout_button.setToolTip('接続確立前のTimeout。クリックで表示時刻へ移動。矢印から各回を選べます。接続後の切断は別です。')
    def jump_timeout():
        times=timeout_display['times']
        if times:jump_to(times[max(0,timeout_display['count']-1)])
    timeout_button.clicked.connect(jump_timeout)
    stage_row.addWidget(timeout_button,1)
    layout.addLayout(stage_row)
    rssi_row = QtWidgets.QHBoxLayout()
    rssi_labels = []
    for name in ('P：A受信（B → A）', 'Q：B受信（A → B）'):
        label = QtWidgets.QLabel(f'{name}　RSSI：待機中')
        label.setStyleSheet('font-size:21px; font-weight:bold; padding:8px; background:#233641; color:#8fe3ff;')
        label.setToolTip('受信波形のI²+Q²をWで平均しdBmへ変換。信号と雑音を含む総受信電力。')
        rssi_row.addWidget(label)
        rssi_labels.append(label)
    layout.addLayout(rssi_row)
    rssi_note = QtWidgets.QLabel('RSSI：受信波形から測定')
    rssi_note.setWordWrap(True)
    layout.addWidget(rssi_note)
    reception_row = QtWidgets.QHBoxLayout()
    reception_labels = []
    for name in ('PRDEV → PRC（A → B）', 'PRC → PRDEV（B → A）'):
        label = QtWidgets.QLabel(f'{name}：未判定')
        label.setWordWrap(True)
        reception_row.addWidget(label, 1)
        reception_labels.append(label)
    layout.addLayout(reception_row)
    admission_label = QtWidgets.QLabel('期待RSSI条件：Beacon受信後に判定（PRC → PRDEVのみ）')
    admission_label.setWordWrap(True)
    layout.addWidget(admission_label)
    figure = Figure(figsize=(12, 8), facecolor='#101820')
    figure.text(.03,.025,'P：PRC（B）→ PRDEV（A） ／ Q：PRDEV（A）→ PRC（B）　｜　1：受信条件OK　0：条件未達　?：未判定',
                color='#d4dbe2',fontsize=9)
    canvas = FigureCanvasQTAgg(figure)
    ax, route_ax = figure.subplots(1,2,gridspec_kw={'width_ratios':[1.1,1]})
    state_ax=figure.add_axes((.20,.48,.35,.40))
    stage_ax=figure.add_axes((.20,.15,.35,.20))
    route_ax.set_position((.68,.18,.30,.67))
    ax.set_visible(False)
    toolbar = NavigationToolbar2QT(canvas, window)
    layout.addWidget(toolbar)
    layout.addWidget(canvas, 1)
    detail = QtWidgets.QLabel()
    detail.setStyleSheet('font-family:monospace; padding:5px;')
    layout.addWidget(detail)
    note = QtWidgets.QLabel('P：PRC（B）→ PRDEV（A）　｜　Q：逆方向　｜　凡例ドラッグ・俯瞰ホイール拡大')
    note.setWordWrap(True)
    layout.addWidget(note)
    window.setCentralWidget(widget)
    window.resize(1550, 1000)

    previous = {'stamp': None, 'snapshot': None, 'drawn': None, 'saved': 0}
    route_view = {'limits':None, 'dragging':False}
    legends = MovingLegends(canvas, lambda: previous.update(drawn=None))
    for item in (admission_label, detail):
        item.hide()
    def show_details(value):
        detail.setVisible(value)
        admission_label.setVisible(value and (previous['snapshot'] or {}).get('admission',{}).get('enabled',True))
        previous['drawn']=None
    details.toggled.connect(show_details)
    details.setChecked(args.details)
    def reset_map():
        route_view['limits']=None
        previous['drawn']=None
    reset_route.clicked.connect(reset_map)
    def remember_map():
        route_view['limits']=(route_ax.get_xlim(),route_ax.get_ylim())
        route_view['dragging']=False
    def press_map(event):
        if toolbar.mode and event.inaxes is route_ax and legends.dragging is None:
            route_view['dragging']=True
    canvas.mpl_connect('button_press_event',press_map)
    def release_map(event):
        if route_view['dragging']:
            QtCore.QTimer.singleShot(0,remember_map)
    canvas.mpl_connect('button_release_event',release_map)
    def wheel_map(event):
        if event.inaxes is not route_ax or event.xdata is None or legends.dragging is not None:
            return
        factor=.8 if event.button=='up' else 1.25
        xlim,ylim=route_ax.get_xlim(),route_ax.get_ylim()
        route_ax.set_xlim([event.xdata+(v-event.xdata)*factor for v in xlim])
        route_ax.set_ylim([event.ydata+(v-event.ydata)*factor for v in ylim])
        remember_map()
        canvas.draw_idle()
    canvas.mpl_connect('scroll_event',wheel_map)
    scenario = {'config':None}
    state_ja = {'SCANNING':'探索中', 'WAIT_RESPONSE':'Response待ち', 'WAIT_ACK':'ACK待ち', 'ASSOCIATED':'接続済み'}
    reasons = {'HALF_DUPLEX_OVERLAP':'送信中と重複（受信失敗時の観測）',
               'EMPIRICAL_PACKET_ERROR':'測定曲線のPERによる受信失敗', 'EMPIRICAL_BELOW_RANGE':'測定曲線の下限未満',
               'RSSI_BELOW_SENSITIVITY':'受信電力が閾値未満',
               'AWGN_PACKET_ERROR':'統計モデルの雑音によるフレーム誤り', 'RECEIVER_TRANSMITTING':'送信中のため受信不可',
               'NO_SYNC_OR_HEADER':'同期またはヘッダー未検出', 'BODY_LDPC':'誤り訂正で復号できず',
               'EXPECTED_RSSI_REJECT':'Beacon受信成功・期待RSSI条件未達', 'BEACON_CRC':'Beaconの検査不合格'}

    def render(data):
        nonlocal first_connected_s
        if first_connected_s is None:
            first_connected_s=next((row[0]/data['sample_rate_hz'] for row in data.get('setup_history',[]) if row[3]),None)
        if scenario['config'] is None and config_path.exists():
            scenario['config'] = yaml.safe_load(config_path.read_text())
        config = scenario['config']
        display_duration=min(config['duration_s'],replay.duration) if config and replay else config['duration_s'] if config else 0
        preview = bool(config and config.get('preview_mode'))
        threshold = bool(config and config.get('phy_error_model')=='threshold_only')
        statistical = bool(config and config.get('phy_error_model')=='empirical_packet_v1')
        calculated = threshold or statistical
        if config:
            if config['protocol'].get('data_from_prc',False):
                roles.setText('データ送信側：PRC（B） → データ受信側：PRDEV（A）')
            else:
                roles.setText('データ送信側：PRDEV（A） → データ受信側：PRC（B）〔旧設定〕')
            roles.setToolTip('データの主方向を表します。Beacon・Request・Response・ACKなどの制御通信は両方向です。')
            title.setText(('【簡易プレビュー】 ' if preview else '')+f"{'並行' if config['scenario']=='parallel' else '十字'}・{'固定アンテナ' if config['antenna_mode']=='fixed' else '追従アンテナ'}　受信判定・航跡")
            if statistical:
                status_label = {'synthetic_demo': '【動作確認用の仮データ】',
                                'measured_with_assumptions': '【実測曲線＋未測定条件の仮定】'}.get(config.get('calibration_status'), '')
                title.setText('② BER/PER確率モデル — 詳細プロトコル／MCSテーブル'+status_label)
                window.setWindowTitle('② BER/PER確率モデル — 受信判定・航跡')
                title.setToolTip('実測表でフレーム成否を抽選。詳細MACを共用し、波形計算は省略。')
            elif threshold:
                title.setText('【受信電力閾値モデル】 '+f"{'並行' if config['scenario']=='parallel' else '十字'}・{'固定' if config['antenna_mode']=='fixed' else '追従'}／{config['radio_ab']['sensitivity_dbm']:g} dBm以上で受信／MCSテーブル")
                window.setWindowTitle('受信電力閾値モデル — MCSテーブル・並行固定')
                title.setToolTip('受信回路出力の計算電力で判定。既存簡易MACで接続し、DATA方向のMCS速度を使用。ACK確認後に転送量を加算。')
            elif preview:
                window.setWindowTitle('簡易プレビュー — 方向別判定・接続段階')
                title.setToolTip('統計QPSKの受信判定と簡易接続処理。波形同期・LDPC復号・期待RSSI条件を省略。接続時刻や通信量の正式評価には使用しません。')
            elif config.get('evaluation_scope')=='protocol_fault_injection':
                title.setText('通信本体の通し試験 — 詳細PHY・MAC／ATP 500 ms／既知の障害を挿入')
                window.setWindowTitle('通信本体の通し試験 — ATP 500 ms')
                seconds=(data['sample']+1)/data['sample_rate_hz']
                faults=config['fault_injection']
                condition=('Q方向に減衰を追加' if seconds<faults['q_recovery_s'] else
                           '追加減衰なし' if seconds<faults['both_outage_s'] else '両方向に減衰を追加（通信断）')
                roles.setText('データ送信：B → A　｜　試験条件：'+condition+'　｜　リンク確立表示は転送継続を保証しません')
            elif config.get('evaluation_scope')=='natural_geometry_short_interval':
                title.setText('強制減衰なし — 並行・固定／詳細PHY・MAC／ATP 500 ms')
                window.setWindowTitle('強制減衰なし — リンク確立の確認')
                roles.setText('データ送信：B → A　｜　最接近付近の短区間・各機1 m/s　｜　距離・指向性による減衰と熱雑音を使用')
        measured=data.get('rssi_source')=='rx_iq_matched_filter'
        for label, name, key in zip(rssi_labels, ('P：B → A','Q：A → B'),
                                     ('rssi_a_dbm','rssi_b_dbm')):
            value=data.get(key if measured else key.replace('rssi_', 'predicted_signal_')) if measured or calculated else None
            reading=f'{value:.2f} dBm' if value is not None and np.isfinite(value) else '測定なし'
            ep=0 if key=='rssi_a_dbm' else 1
            if measured and value is None and not data.get('rssi_includes_noise',True) and data.get('rssi_valid_chips',[0,0])[ep]>0:
                reading='0 W（信号なし）'
            label.setText(f"{name}　RSSI（{'計算' if calculated else '波形'}）：{reading}")
            label.setToolTip('受信回路出力（TJX入力）の計算信号電力。雑音は受信判定に使用しません。' if calculated else '受信波形の平均電力。')
        if statistical:
            rssi_note.setText('計算受信電力（TJX入力）：測定曲線からPERを求めて確率判定。TJXの生RSSIとは単位が異なります。')
        elif threshold:
            rssi_note.setText('計算受信電力（TJX入力）：−68.5 dBm以上で受信。速度はMCSテーブル、転送量はACK確認分。')
        elif measured:
            window_us=data['rssi_window_samples']/data['sample_rate_hz']*1e6
            rssi_note.setText(f'波形RSSI：{window_us:g} µs平均・'+('信号＋雑音' if data.get('rssi_includes_noise',True) else '無AWGNの信号電力')+'。送信中は測定対象外。')
        else:
            rssi_note.setText('この実行は波形RSSIを未計測です。簡易プレビューと旧ログの電力予測値はRSSIとして表示しません。')
        hz = data['sample_rate_hz']
        connected = data['state_a']=='ASSOCIATED' and data['state_b']=='ASSOCIATED'
        def result_code(key):
            result=data.get(key,{})
            return (1 if result['ok'] else -1) if result.get('known') else 0
        highlight_q, highlight_p = xor_unmet(result_code('setup_ab'),result_code('setup_ba'),connected)
        for label, name, key, highlighted in zip(reception_labels, ('Q：A → B','P：B → A'),
                                    ('setup_ab','setup_ba'),(highlight_q,highlight_p)):
            result = data.get(key,{})
            if connected:
                message, color = 'リンク確立済み\n接続後データの受信成否はこの判定から除外', '#205941'
            elif not result.get('known'):
                message, color = '未判定：確立用フレームの観測待ち', '#344451'
            else:
                color = '#205941' if result['ok'] else ('#843737' if highlighted else '#344451')
                outcome = '充足を確認' if result['ok'] else '不充足を確認'
                reason = '' if result['ok'] else '\n' + reasons.get(result['reason'], result['reason'])
                message = f"確立用フレーム：{outcome}　{result['frame']}\n確認時刻 {result['sample']/hz*1000:.6f} ms{reason}"
            label.setText(name + '\n' + message)
            label.setToolTip(message)
            label.setText(name + '　' + ('確立済み' if connected else '?：未判定' if not result.get('known') else '1：直近条件OK' if result['ok'] else '0：直近条件未達'))
            label.setStyleSheet(f'font-size:14px; padding:8px; color:white; background:{color};')
        admission = data.get('admission',{})
        admission_label.setVisible(details.isChecked() and admission.get('enabled',True))
        if preview:
            admission_label.setText('簡易モデル：期待RSSI条件・波形復号は評価しません。色分けは統計モデルの受信結果です。')
        elif not admission.get('enabled',True):
            admission_label.clear()
        elif admission.get('known'):
            verdict = '充足' if admission['ok'] else '未達（Request送信を許可せず）'
            admission_label.setText(f"期待RSSI条件：{verdict}／PRC → PRDEV、最後のBeacon判定 {admission['sample']/hz*1000:.6f} ms\n"
                                    f"アンテナ利得補正後の推定値 {admission['measured_dbm']:.2f} dBm ≥ 基準 {admission['expected_dbm']:.2f} dBm。グラフとは測定点が異なります。")
            admission_label.setStyleSheet('color:#165b33;' if admission['ok'] else 'color:#ad2222; font-weight:bold;')
        else:
            admission_label.setText('期待RSSI条件：未判定（Beacon復号待ち）。PRC → PRDEVのみ適用。')
        points = np.asarray(data.get('rssi_history', []) if measured else data.get('predicted_signal_history', []) if calculated else [], dtype=float)
        legends.clear()
        ax.clear()
        ax.set_facecolor('#101820')
        ax.tick_params(colors='#d4dbe2', labelsize=10)
        for spine in ax.spines.values():
            spine.set_color('#405262')
        ax.grid(alpha=.2, color='#a8bbc9')
        ax.set_ylabel('計算受信電力 [dBm]' if calculated else '波形RSSI [dBm]', color='white', fontsize=12)
        ax.set_xlabel('シミュレーション時刻 [ms]', color='white', fontsize=12)
        end_ms = (data['sample']+1)/data['sample_rate_hz']*1000
        history = data.get('setup_history',[])
        if mode.currentIndex()==0:
            established = next((row[0]/hz*1000 for row in history if row[3]), None)
            right = max(.2,established*1.15) if established is not None else max(.2,end_ms)
            width = right
        elif mode.currentIndex()==5:
            width = right = max(.2,display_duration*1000 if config else end_ms)
        elif mode.currentIndex()==6:
            width = 10000.
            right = max(width,end_ms)
        else:
            width = (5.,10.,100.,1000.)[mode.currentIndex()-1]
            right = max(width,end_ms)
        if width>=2000:
            ax.xaxis.set_major_formatter(FuncFormatter(lambda x,pos:f'{x/1000:g}'))
            ax.set_xlabel('シミュレーション時刻 [s]',color='white')
        else:
            ax.xaxis.set_major_formatter(ScalarFormatter())
        ax.set_xlim(right-width,right)
        ax.set_ylim(-110,0)
        if points.size:
            times = points[:,0]/data['sample_rate_hz']*1000
            for column, color, label in ((1,'#53cfff','P：B → A'),
                                         (2,'#ffce73','Q：A → B')):
                ax.plot(times,points[:,column],color=color,lw=2,label=label)
                ax.plot(times[-1],points[-1,column],'o',color=color,ms=5)
            visible = points[times>=right-width,1:]
            finite=visible[np.isfinite(visible)]
            if finite.size:
                ax.set_ylim(float(finite.min())-5,float(finite.max())+5)
        else:
            ax.text(.5,.5,'波形RSSI：測定なし',
                    transform=ax.transAxes,color='white',ha='center')
        if threshold:
            ax.axhline(config['radio_ab']['sensitivity_dbm'],color='#ff7777',ls=':',label=f"受信閾値 {config['radio_ab']['sensitivity_dbm']:g} dBm")
        labeled = set()
        intervals = []
        for index, (sample,ab,ba,established) in enumerate(history):
            stop_sample = history[index+1][0] if index+1<len(history) else data['sample']+1
            if established and right-width<=sample/hz*1000<=right:
                ax.axvline(sample/hz*1000,color='#76df98',ls='--',lw=1.4,
                           label='リンク確立' if 'リンク確立' not in labeled else None)
                labeled.add('リンク確立')
            unmet_q, unmet_p = xor_unmet(ab,ba,established)
            if unmet_q or unmet_p:
                left,stop = sample/hz*1000,stop_sample/hz*1000
                intervals.append((left,stop,ab<0))
                if stop>=right-width and left<=right:
                    label = 'Qのみ不足' if unmet_q else 'Pのみ不足'
                    if points.size:
                        # 境界に近いRSSI点を補間して、該当する方向の線だけを太くする。
                        lo,hi=max(left,times[0]),min(stop,times[-1])
                        if lo<hi:
                            selected=times[(times>lo)&(times<hi)]
                            selected=np.concatenate(([lo],selected,[hi]))
                            values=np.interp(selected,times,points[:,2 if unmet_q else 1])
                            ax.plot(selected,values,color='#ff655f' if unmet_q else '#d78aff',lw=5,
                                    label=label if label not in labeled else None)
                            labeled.add(label)
        failures = [row for row in data.get('failures',[]) if row[4] in ('BEACON','REQUEST','RESPONSE','ACK')]
        markers = {}
        for start, finish, ab, rssi, frame, reason in (failures if details.isChecked() else []):
            left, stop = start/hz*1000, finish/hz*1000
            if stop < right-width or left > right:
                continue
            if not points.size:
                continue
            column=2 if ab else 1
            nearby=np.argmin(np.abs(times-stop))
            rssi=points[nearby,column]
            if not np.isfinite(rssi):
                continue
            admission_failure = reason == 'EXPECTED_RSSI_REJECT'
            color = '#a8b0b7' # 詳細の失敗印は中立色。XOR強調と混同しない。
            label = '期待RSSI条件未達' if admission_failure else 'A → B確立用フレーム失敗' if ab else 'B → A確立用フレーム失敗'
            key=(label,color,'D' if admission_failure else 'x')
            markers.setdefault(key,[]).append((stop,rssi))
        for (label,color,marker),coordinates in markers.items():
            xy=np.asarray(coordinates)
            ax.scatter(xy[:,0],xy[:,1],marker=marker,color=color,s=25,
                       label=label if label not in labeled else None)
            labeled.add(label)
        if points.size or labeled:
            legends.add(ax,'rssi',loc='upper left',fontsize=9,facecolor='#22303b',edgecolor='#405262')
        end = ('再生終了' if data.get('complete') else '記録を再生中') if data.get('replay') else ('実行終了・最終結果を表示' if data.get('complete') else '実行中')
        status.setText(f'{end}  |  計算時刻 {end_ms:.6f} ms')
        if config:
            elapsed=max(0,time.time()-config_path.stat().st_mtime)
            seconds=end_ms/1000
            remaining=max(0,display_duration-seconds)*elapsed/seconds if seconds>0 else 0
            status.setText(f"{end} | {seconds:.6f} / {display_duration:g} 秒（{100*seconds/display_duration:.2f}%）"
                           + (f" | 概算残り {remaining/3600:.1f} 時間" if not data.get('complete') and seconds>0 and not data.get('replay') else ''))
        if calculated:
            total=data.get('delivered_mb',0)
            status.setText(status.text()+f" | {'重複除外の受信量' if statistical else 'ACK確認済み'} {total:.3f} MB")
            if data.get('complete') and end_ms>0:
                status.setText(status.text()+f' | 全区間平均 {total*8/end_ms:.3f} Gbps')
        route_ax.clear()
        route_ax.set_facecolor('#101820')
        route_ax.tick_params(colors='#d4dbe2',labelsize=9)
        route_ax.grid(alpha=.2,color='#a8bbc9')
        for spine in route_ax.spines.values():
            spine.set_color('#405262')
        route_ax.set_xlabel('X [m]',color='white')
        route_ax.set_ylabel('Y [m]',color='white')
        if config:
            duration=display_duration
            current=min(duration,end_ms/1000,right/1000)
            planned=positions(config,[0,duration])
            for track in planned:
                route_ax.plot(track[:,0],track[:,1],'--',color='#61717e',lw=1)
            records=[[0,0,0,0]]+history
            used=set()
            for index,(sample,ab,ba,established) in enumerate(records):
                start=sample/hz
                stop=min(current,records[index+1][0]/hz if index+1<len(records) else current)
                if start>=stop:
                    continue
                styles=direction_styles(ab,ba,established)
                for track,(color,label) in zip(positions(config,[start,stop]),styles):
                    route_ax.plot(track[:,0],track[:,1],color=color,lw=5 if label.startswith(('Q：','P：')) else 3,
                                  label=label if label not in used else None)
                    if not label.startswith('_'):
                        used.add(label)
            angles = antenna_angles(config, current)
            route_center_x = np.concatenate(planned)[:,0].mean()
            if route_view['limits'] is not None:
                route_center_x = sum(route_view['limits'][0])/2
            for index,track in enumerate(positions(config,[current])):
                x,y=track[0]
                color = '#53cfff' if index == 0 else '#ffce73'
                azimuth, elevation = angles[index]
                route_ax.plot(x,y,'o' if index==0 else 's',color=color,ms=8,zorder=6)
                if abs(elevation) < 89.99:
                    # 矢印は水平面への投影方向。仰角は隣接ラベルで示す。
                    yaw = np.radians(azimuth)
                    route_ax.quiver(x,y,np.cos(yaw),np.sin(yaw),angles='xy',
                                    scale=11,width=.006,color=color,zorder=5)
                else:
                    route_ax.annotate('⊙' if elevation > 0 else '⊗',(x,y),
                                      xytext=(0,15),textcoords='offset points',
                                      color=color,ha='center',fontsize=17)
                route_ax.annotate(
                    f"{'A' if index==0 else 'B'} ({x:.1f}, {y:.1f})\n"
                    f"方位 {azimuth:.1f}° / 仰角 {elevation:+.1f}°",(x,y),
                    xytext=(-7 if x>route_center_x else 7,17 if index==0 else -16),textcoords='offset points',
                    ha='right' if x>route_center_x else 'left',
                    va='bottom' if index==0 else 'top',color=color,fontsize=8,
                    bbox=dict(boxstyle='round,pad=.25',fc='#101820',ec=color,alpha=.85),
                    zorder=7)
            all_xy=np.concatenate(planned)
            xmin,ymin=all_xy.min(axis=0);xmax,ymax=all_xy.max(axis=0)
            padx=max(2,(xmax-xmin)*.12);pady=max(3,(ymax-ymin)*.25)
            route_ax.set_xlim(xmin-padx,xmax+padx);route_ax.set_ylim(ymin-pady,ymax+pady)
            route_ax.set_title(f"俯瞰 t={current:g} s　高度 A：{config['drone_a']['altitude_m']:g} m / B：{config['drone_b']['altitude_m']:g} m",
                               color='white',fontsize=10)
            if used:
                legends.add(route_ax,'route',loc='lower center',fontsize=8,facecolor='#22303b',edgecolor='#405262')
        if route_view['limits'] is not None:
            route_ax.set_xlim(route_view['limits'][0]);route_ax.set_ylim(route_view['limits'][1])
        states = (data['state_a'], data['state_b'])
        if states == ('ASSOCIATED','ASSOCIATED'):
            stage = '④ 接続成立：データ転送・受領確認を行う段階'
        elif 'WAIT_ACK' in states:
            stage = '③ 接続確認：BがACK（Acknowledgment：受領確認）を待つ段階'
        elif 'WAIT_RESPONSE' in states:
            stage = '② 接続要求：AがResponse（接続応答）を待つ段階'
        else:
            stage = '① 接続先探索：Beacon（ビーコン）を送受信する段階'
        phase.setToolTip(stage)
        phase.setText(('終了時：' if data.get('complete') else '状態：')
                      + ('リンク確立済み' if connected else f"A：{state_ja.get(states[0],states[0])} ／ B：{state_ja.get(states[1],states[1])}"))
        active_timeout=timeout_display['active']
        if active_timeout is not None:
            event=timeout_events.rows[active_timeout]
            phase.setText(f"TIMEOUT #{active_timeout+1}　発生時刻 {event['time_s']:.9f} s　｜　"+phase.text())
            phase.setStyleSheet('font-size:23px; font-weight:bold; padding:8px; background:#a71927; color:#ffffff; border:3px solid #ffbd59;')
        else:
            phase.setStyleSheet('font-size:17px; font-weight:bold; padding:6px; background:#183544; color:white;')
        recent = [f"{end/hz*1000:.6f} ms　{'A → B' if ab else 'B → A'}　{frame}：{reasons.get(reason,reason)}"
                  for start,end,ab,rssi,frame,reason in failures[-3:]]
        interval_text = [f"{start:.6f}～{stop:.6f} ms：{'A → B' if ab else 'B → A'}のみ不充足を確認"
                         for start,stop,ab in intervals[-2:]]
        detail.setText('リンク確立前の片方向不充足区間\n' + ('\n'.join(interval_text) if interval_text else '確認された区間なし（未判定を含む可能性あり）')
                       + '\n確立用フレームの直近の不成立：' + ('\n'+'\n'.join(recent) if recent else 'なし'))
        # 主図: 4種類の方向別判定。未判定・リンク確立は別状態。
        state_ax.clear()
        state_ax.set_facecolor('#101820')
        state_ax.tick_params(colors='white',labelsize=9)
        state_ax.grid(axis='x',alpha=.2,color='#a8bbc9')
        state_ax.set_yticks(range(6),JUDGEMENT_LABELS)
        state_ax.set_ylim(-.5,5.5)
        state_ax.set_xlim(right-width,right)
        state_ax.set_xlabel('シミュレーション時刻 [s]' if width>=2000 else 'シミュレーション時刻 [ms]',color='white')
        if width>=2000:
            state_ax.xaxis.set_major_formatter(FuncFormatter(lambda x,pos:f'{x/1000:g}'))
        state_ax.set_title('方向別の直近判定　● 更新時刻 ／ 線 判定を保持',color='white',fontsize=10)
        records=[[0,0,0,0]]+history
        previous_state=None
        for index,(sample,ab,ba,established) in enumerate(records):
            start=sample/hz*1000
            stop=(records[index+1][0]/hz*1000 if index+1<len(records) else end_ms)
            level=judgement_state(ab,ba,established)
            if stop>=right-width and start<=right:
                state_ax.plot([start,stop],[level,level],color=JUDGEMENT_COLORS[level],lw=3)
                if index:
                    state_ax.plot(start,level,'o',color=JUDGEMENT_COLORS[level],ms=4)
                if previous_state is not None and previous_state!=level:
                    state_ax.plot([start,start],[previous_state,level],color='#82929e',lw=1,ls=':')
            previous_state=level
        state_ax.set_position((.20,.48,.35,.40))
        state_ax.set_xlabel('')
        state_ax.tick_params(labelbottom=False)
        stage_ax.clear()
        stage_ax.set_position((.20,.15,.35,.20))
        stage_ax.set_facecolor('#101820')
        stage_ax.tick_params(colors='white',labelsize=9)
        stage_ax.grid(axis='x',alpha=.2,color='#a8bbc9')
        stage_ax.set_yticks(range(5),('Beacon探索','Request準備・送信','Response待ち','ACK確認','データ通信'))
        stage_ax.set_ylim(-.5,4.5)
        stage_ax.set_xlim(right-width,right)
        stage_ax.set_title('接続手順の段階',color='white',fontsize=10)
        stage_ax.set_xlabel('シミュレーション時刻 [s]' if width>=2000 else 'シミュレーション時刻 [ms]',color='white')
        if width>=2000:
            stage_ax.xaxis.set_major_formatter(FuncFormatter(lambda x,pos:f'{x/1000:g}'))
        stage_indices={'BEACON':0,'REQUEST':1,'RESPONSE':2,'ACK':3,'DATA':4}
        stage_history=data.get('stage_history',[])
        if statistical and len(stage_history)>1200:
            # 全区間表示の密集した再送だけ描画解像度へ間引く。保存記録・通信判断は変更しない。
            bucket_width=max(1,width*hz/1000/600)
            selected={}
            left_sample=(right-width)*hz/1000
            previous_stage=None
            for entry in stage_history:
                if entry[0]<left_sample:previous_stage=entry;continue
                if entry[0]>right*hz/1000:break
                selected[int((entry[0]-left_sample)/bucket_width)]=entry
            stage_history=([previous_stage] if previous_stage else [])+list(selected.values())

        last_level=None
        for index,(sample,name) in enumerate(stage_history):
            start=sample/hz*1000
            stop=stage_history[index+1][0]/hz*1000 if index+1<len(stage_history) else end_ms
            level=stage_indices[name]
            if stop>=right-width and start<=right:
                color='#76df98' if level==4 else '#ffc66d'
                stage_ax.plot([start,stop],[level,level],color=color,lw=3)
                stage_ax.plot(start,level,'o',color=color,ms=3)
                if last_level is not None and last_level!=level:
                    stage_ax.plot([start,start],[last_level,level],color='#82929e',ls=':',lw=1)
            last_level=level
        if not stage_history:
            stage_ax.text(.5,.5,'段階履歴は未収録',transform=stage_ax.transAxes,color='white',ha='center')
        for sample,ab,ba,established in history:
            at=sample/hz*1000
            if established and right-width<=at<=right:
                for panel in (state_ax,stage_ax):
                    panel.axvline(at,color='#76df98',ls='--',lw=1)
                stage_ax.annotate('リンク確立',(at,4),xytext=(5,-14),textcoords='offset points',color='#76df98',fontsize=9)

        # Timeoutは独立した状態段を追加せず、正確な発生時刻だけ重ねる。
        for number,event in enumerate(timeout_events.rows[:timeout_display['count']]):
            at=event['time_s']*1000
            if not right-width<=at<=right:continue
            hot=number==active_timeout
            for panel in (state_ax,stage_ax):
                panel.axvline(at,color='#ff4157',lw=5 if hot else 2.5,alpha=1 if hot else .85,zorder=25)
                top=panel.get_ylim()[1]-.15
                panel.plot(at,top,'v',color='#ffbd59' if hot else '#ff4157',ms=12 if hot else 7,zorder=26)
        if timeout_display['count']:
            state_ax.set_title('方向別の直近判定　● 更新時刻 ／ 赤線：Timeout発生時刻',color='white',fontsize=10)

        ax.set_visible(details.isChecked())
        route_ax.set_position((.68,.48,.30,.40) if details.isChecked() else (.68,.18,.30,.67))
        if details.isChecked():
            ax.set_position((.68,.15,.30,.20))
        stage_name=data.get('protocol_stage') or ('DATA' if connected else 'ACK' if 'WAIT_ACK' in states else 'RESPONSE' if 'WAIT_RESPONSE' in states else 'BEACON')
        active_stage={'BEACON':0,'REQUEST':1,'RESPONSE':2,'ACK':3,'DATA':4}.get(stage_name,0)
        for index,label in enumerate(stage_labels):
            label.setStyleSheet('font-size:14px; padding:7px; border-radius:4px; '+
                                ('background:#238654; color:white; font-weight:bold;' if index==active_stage else 'background:#dce2e6; color:#59656e;'))
        count=timeout_display['count'];times=timeout_display['times']
        stamp=times[max(0,count-1)] if times else None
        timestamps_available=timeout_events.header is not None or timeout_events.finished
        reported_count=count if timestamps_available else data.get('timeouts',0)
        if timestamps_available and first_connected_s is not None:
            reported_count=sum(t<first_connected_s for t in times[:count])
        timeout_button.setText(f'Timeout：{reported_count}回（初回確立まで）'+(f'\n{"直近" if count else "初回"} {format_event_time(stamp)}' if stamp is not None else '\n記録なし' if timestamps_available else '\n時刻未収録'))
        timeout_button.setToolTip('回数は初回リンク確立まで。未確立では表示時刻まで。\n時刻メニューとグラフには再接続時を含む全Timeoutを残します。')
        timeout_button.setEnabled(bool(replay and times))
        timeout_button.setStyleSheet('font-size:16px; padding:7px; font-weight:bold; border-radius:4px; '+
            ('background:#a71927; color:white; border:3px solid #ffbd59;' if active_timeout is not None else 'background:#f3d8dc; color:#7f1827;'))
        if replay and timeout_display['loaded_menu_count']!=len(times):
            timeout_menu.clear()
            for index,event in enumerate(timeout_events.rows):
                scope='（初回確立後・集計外）' if first_connected_s is not None and event['time_s']>=first_connected_s else ''
                action=timeout_menu.addAction(f"Timeout #{index+1}{scope}　{format_event_time(event['time_s'])}　{event['direction']} {event['reason']}")
                action.triggered.connect(lambda checked=False,t=event['time_s']:jump_to(t))
            timeout_display['loaded_menu_count']=len(times)
        canvas.draw_idle()
        previous['drawn'] = (data['sample'], mode.currentIndex(), data.get('complete'),timeout_display['active'],timeout_display['count'])
        if args.screenshot and time.monotonic()-previous['saved'] > 2:
            args.screenshot.parent.mkdir(parents=True, exist_ok=True)
            figure.savefig(args.screenshot, dpi=110, facecolor=figure.get_facecolor())
            previous['saved'] = time.monotonic()

    def update():
        try:
            if replay:
                wall=time.monotonic()
                if not pause.isChecked():
                    if guided and guided.isChecked():
                        from hrcp_monitor_replay import advance_playback
                        playback['seconds']=advance_playback(playback['seconds'],wall-playback['wall'],guided_segments,replay.duration)
                    else:
                        playback['seconds']=min(replay.duration,playback['seconds']+(wall-playback['wall'])*float(speed.currentText().rstrip('×')))
                if guided and guided.isChecked():
                    close_up=len(guided_segments)==1 or playback['seconds']<guided_segments[0]['until_s'] or playback['seconds']>=guided_segments[-2]['until_s']
                    mode.setCurrentIndex(2 if close_up else 5)
                playback['wall']=wall
                seek_to(playback['seconds'])
                previous['snapshot']=replay.at(playback['seconds'])
                if playback['seconds']>=replay.duration:pause.setChecked(True)
            elif not pause.isChecked():
                stamp = args.snapshot.stat().st_mtime_ns
                if stamp != previous['stamp']:
                    data = json.loads(args.snapshot.read_text())
                    if data.get('schema') != 'hrcp_signals_v1':
                        raise ValueError('未対応のモニターデータです')
                    previous['snapshot'], previous['stamp'] = data, stamp
            data = previous['snapshot']
            timeout_events.refresh()
            if data is not None:
                times=[row['time_s'] for row in timeout_events.rows]
                seconds=(data['sample']+1)/data['sample_rate_hz']
                timeout_display['times']=times
                timeout_display['count']=bisect_right(times,seconds+1e-9)
                timeout_display['active']=timeout_highlight.update(times,seconds,time.monotonic(),not pause.isChecked())
            if data is not None and legends.dragging is None and not route_view['dragging'] and previous['drawn'] != (data['sample'], mode.currentIndex(), data.get('complete'),timeout_display['active'],timeout_display['count']):
                render(data)
        except FileNotFoundError:
            pass
        except (ValueError, KeyError, OSError) as exc:
            status.setText(f'モニターデータの読み込み待ち: {exc}')

    pause.toggled.connect(lambda value: pause.setText(('再生' if value else '一時停止') if replay else ('表示を再開' if value else '表示を一時停止')))
    if replay:pause.toggled.connect(lambda value:playback.update(wall=time.monotonic()))
    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(200)
    window.show()
    window.raise_()
    window.activateWindow()
    update()
    if args.render_once:
        QtCore.QTimer.singleShot(1000, app.quit)
    app.exec_()


if __name__ == '__main__':
    main()
