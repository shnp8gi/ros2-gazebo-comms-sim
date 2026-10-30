"""接続方法修正後の測定を保存・監査し、MCSごとの曲線へ変換する。"""
import csv
import hashlib
import json
import re
import shutil
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

OUT = Path(__file__).resolve().parent
REPO = OUT.parents[1]
sys.path.insert(0, str(REPO / 'tools'))
from prepare_receiver_curve import aggregate_samples, prepare

SOURCE = Path('/home/rosgazebo/Downloads/TJX_attenuator-20261029T092900Z-1-001/TJX_attenuator')
summary, conditions, sources = [], [], []
for folder in sorted(SOURCE.glob('MCS*'), key=lambda p: int(p.name[3:])):
    mcs = int(folder.name[3:])
    for path in sorted(folder.glob('*.csv'), key=lambda p: float(re.match(r'[\d.]+', p.stem)[0])):
        attenuation = float(re.match(r'[\d.]+', path.stem)[0])
        with path.open(encoding='utf-8-sig', newline='') as stream:
            rows = list(csv.DictReader(stream))
        ber, ber_info = aggregate_samples(rows, mcs, 'ber', 'clear_write')
        per, per_info = aggregate_samples(rows, mcs, 'per', 'clear_write', True)
        values = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}
        assert all(np.isfinite(v).all() for v in values.values()), path
        local = Path('raw') / folder.name / path.name
        dest = OUT / local
        dest.parent.mkdir(parents=True, exist_ok=True)
        if dest.exists():
            assert dest.read_bytes() == path.read_bytes(), dest
        else:
            shutil.copyfile(path, dest)
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        assert hashlib.sha256(dest.read_bytes()).hexdigest() == digest
        item = dict(mcs=mcs, attenuation_db=attenuation, file=str(local), rows=len(rows),
                    first_time_s=values['time'][0], last_time_s=values['time'][-1],
                    detected_packets=ber_info['denominator_sum'],
                    ber_detect_weighted=ber, per_detect_weighted_approx=per,
                    rssi_raw_median=float(np.median(values['RSSI'])),
                    snr_db_median=float(np.median(values['SNR'])))
        for key, label in [('PER','per'),('bER','ber'),('RSSI','rssi_raw')]:
            item[label+'_min'] = float(values[key].min())
            item[label+'_max'] = float(values[key].max())
        summary.append(item)
        conditions.append(dict(direction='Q_A_TO_B', frame='DATA', mcs=mcs, frame_bytes=100000,
            attenuation_db=attenuation, reference_attenuation_db=0, reference_rssi_dbm='',
            statistics_csv=str(local), aggregation='clear_write', ber_stage='unknown',
            frequency_hz=60480000000,
            notes='接続方法修正後。下り60.48 GHz・100 kB・clear write・MCS間の送信電力と配線は同じとユーザー確認。機器は上り下りで入れ替え。同型、個体ID不明。'))
        sources.append(dict(source=str(path), local_copy=str(local), sha256=digest,
                            ber_aggregation=ber_info, per_aggregation=per_info))

for name, records in [('measurement_summary.csv', summary), ('manifest.csv', conditions)]:
    with (OUT / name).open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)
for metric in ['ber', 'per']:
    output = OUT / f'{metric}_relative.csv'
    if output.exists():
        raise ValueError(f'受信曲線は上書きしません: {output}')
    prepare(OUT / 'manifest.csv', output, metric=metric, allow_unknown_ber=(metric == 'ber'),
            allow_detect_weighted_per=(metric == 'per'), relative_power=True)
(OUT / 'measurement_conditions.json').write_text(json.dumps(dict(
    calibration_status='measured_relative_power_only', direction='Q_A_TO_B',
    user_direction='下り', frequency_hz=60480000000, frame_bytes=100000,
    aggregation='clear_write', mcs_values=sorted(set(r['mcs'] for r in summary)),
    packet_length_interpretation='100 kB interpreted as 100000 bytes',
    tx_power_and_cabling_same_across_mcs=True,
    hardware_swap='上り下りの機器が逆。同型だが個体ID不明。今回の組み合わせに対する測定。',
    previous_measurement_invalid_reason='コネクターの接続方法が間違っていた（ユーザー申告）',
    reference_rssi_dbm=None, ber_stage='unknown',
    filename_assumptions={'MCS6/48.0d.csv':'48.0 dB attenuation'},
    file_count=len(summary), sample_count=sum(r['rows'] for r in summary), sources=sources
), ensure_ascii=False, indent=2), encoding='utf-8')

plt.rcParams['font.family'] = 'Noto Sans CJK JP'
fig, axes = plt.subplots(2, 2, figsize=(13, 8), layout='constrained')
for mcs in sorted(set(r['mcs'] for r in summary)):
    subset = [r for r in summary if r['mcs'] == mcs]
    x = [r['attenuation_db'] for r in subset]
    for ax, key in zip(axes.flat, ['per_detect_weighted_approx','ber_detect_weighted','rssi_raw_median','snr_db_median']):
        ax.plot(x, [r[key] for r in subset], 'o-', ms=4, label=f'MCS {mcs}')
for ax, label in zip(axes.flat, ['検出数で重み付けした近似PER','検出数で重み付けしたBER','RSSI（生値の中央値）','SNR（表示値の中央値）[dB]']):
    ax.set(xlabel='外部減衰量 [dB]', ylabel=label)
    ax.grid(True, alpha=.25)
axes[0,0].set_ylim(-.03, 1.03)
axes[0,0].legend()
axes[0,1].set_yscale('symlog', linthresh=1e-10)
axes[0,1].set_ylim(bottom=0)
fig.suptitle('接続方法修正後：下り60.48 GHz・100 kB・clear write\n絶対受信電力未校正／同型機器の入れ替えあり／旧測定は不使用')
fig.savefig(OUT / 'receiver_curves.png', dpi=160)
fig.savefig(OUT / 'receiver_curves.pdf')
plt.close(fig)
for mcs in sorted(set(r['mcs'] for r in summary)):
    subset = [r for r in summary if r['mcs'] == mcs]
    first_all_fail = next((r['attenuation_db'] for r in subset if r['per_min'] == 1), None)
    print('MCS',mcs,'files',len(subset),'PER最小',min(r['per_detect_weighted_approx'] for r in subset), '全行PER1の最初',first_all_fail)
print('files',len(summary),'samples',sum(r['rows'] for r in summary))
