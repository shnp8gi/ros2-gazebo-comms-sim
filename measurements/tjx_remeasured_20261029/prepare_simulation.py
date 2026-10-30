"""実測曲線と明示した仮定から②の通し運転用設定を生成する。"""
import csv
import hashlib
import json
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parents[1]
CONFIG = REPO / 'src/comms_sim_pkg/config'
SOURCE = ROOT / 'ideal_linear/per_relative.csv'
TABLE = CONFIG / 'receiver_curve_measured_provisional.csv'
YAML = CONFIG / 'drone_statistical_measured_provisional.yaml'
REFERENCE_DBM = -12.2  # 0 dB減衰時の仮基準。56.3 dB減衰を-68.5 dBmへ置く。
if TABLE.exists() or YAML.exists():
    raise ValueError('設定の既存出力を上書きしません')
with SOURCE.open() as stream:
    measured = list(csv.DictReader(stream))
rows, origins = [], []
for direction in ['Q_A_TO_B', 'P_B_TO_A']:
    for original in measured:
        for mcs in ([original['mcs'], '0'] if original['mcs'] == '3' else [original['mcs']]):
            power = round(REFERENCE_DBM + float(original['relative_power_db']), 9)
            row = dict(direction=direction, frame='DATA' if mcs == '0' else 'ALL', mcs=mcs, rssi_dbm=power,
                       ber='', per=original['per'], reference_bytes=original['reference_bytes'])
            rows.append(row)
            origins.append(dict(direction=direction, mcs=mcs, rssi_dbm=power,
                source_mcs=original['mcs'], source_relative_power_db=original['relative_power_db'],
                opposite_direction_proxy=(direction == 'P_B_TO_A'), mcs0_proxy=(mcs == '0'),
                synthetic_zero=(original['mcs'] == '11' and float(original['relative_power_db']) == -27.5)))
rows.sort(key=lambda r: (r['direction'], int(r['mcs']), r['rssi_dbm']))
assert len(rows) == 164
assert len({(r['direction'], r['mcs'], r['rssi_dbm']) for r in rows}) == 164
with TABLE.open('x', newline='') as stream:
    writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
    writer.writeheader()
    writer.writerows(rows)
assumptions = dict(
    absolute_power_reference_dbm_at_zero_attenuation=REFERENCE_DBM,
    power_alignment='MCS3の最初の全行PER1点56.3 dBを旧基準-68.5 dBmへ置く仮定。絶対感度の実測ではない。',
    opposite_direction='下りQの曲線を未測定の上りPにも適用。P/Qの計算受信電力はそれぞれの既存RF設定を維持。',
    control_mcs='ユーザー指定によりBeacon/Request/Response/ACK/DATA_ACK/DisassociateはMCS3。測定MCS3の受信曲線を参照。',
    data_mcs0='MCStableの0.5 Gbps DATA用だけ、未測定MCS0の曲線をMCS3で代用。繰り返し利得は付加しない。',
    frame_lengths='100000 bytesから既存independent式でDATA2048 bytesおよび短い制御フレームへ補正。長さ別実測校正ではない。',
    zero_success_observations='測定PER1は長さ補正しても1。未観測の成功確率は推定しない。',
    temporal_correlation='なし。フレームごとの独立抽選。',
    distortion='ideal_linear派生曲線を使用。MCS11の20〜27.5 dB実測を除外し27.5 dBに仮定のゼロ点。',
    per_aggregation='packet_lost不明のため検出数重み付き近似PER。',
    protocol='③と共通MAC。既存制御フレーム時間はTJX BPSK形式へ校正済みではない。',
    timeout='接続待ち500 ms、接続後ATP10000 ms。実機現在値の読出確認は未実施。',
    data_direction='既存シナリオを維持しP=B→A。測定方向Qとは異なり代理曲線を使用。',
    throughput='既存MCStable.csvを維持。実測PER曲線に合わせてMCS選択閾値を再最適化していない。')
provenance = dict(calibration_status='measured_with_assumptions', source=str(SOURCE),
    source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
    measurement_conditions=json.loads((ROOT/'measurement_conditions.json').read_text()),
    ideal_linear_provenance=json.loads((ROOT/'ideal_linear/provenance.json').read_text()),
    assumptions=assumptions, row_origins=origins)
TABLE.with_suffix('.csv.provenance.json').write_text(json.dumps(provenance, ensure_ascii=False, indent=2))
config = yaml.safe_load((CONFIG/'drone_statistical_parallel_1mps.yaml').read_text())
config['empirical_phy'].update(table=TABLE.name, metric='per', per_length_policy='independent',
                               calibration_status='measured_with_assumptions', seed=20261029, control_mcs=3)
config['measurement_assumptions'] = assumptions
YAML.write_text('# 実測曲線＋未測定条件の仮定による通し試験。詳細はmeasurement_assumptions。\n'+
                yaml.safe_dump(config, allow_unicode=True, sort_keys=False))
print(YAML)
