"""高電力側の歪みを扱わない、ユーザー指定の理想変換用派生曲線を作る。"""
import csv
import hashlib
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'ideal_linear'
OUT.mkdir(exist_ok=False)
excluded, provenance = [], []
plt.rcParams['font.family'] = 'Noto Sans CJK JP'
fig, axes = plt.subplots(1, 2, figsize=(12, 4.8), layout='constrained')
for metric, ax in zip(['per', 'ber'], axes):
    source = ROOT / f'{metric}_relative.csv'
    with source.open() as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames
        original = list(reader)
    kept = []
    for row in original:
        attenuation = -float(row['relative_power_db'])
        if row['mcs'] == '11' and attenuation <= 27.5:
            excluded.append(dict(metric=metric, attenuation_db=attenuation, original_value=row[metric],
                                 reason='MCS11高電力側の誤り率横ばい領域。歪み疑いとして暫定除外'))
        else:
            kept.append(dict(row))
    anchor = dict(direction='Q_A_TO_B', frame='DATA', mcs='11', relative_power_db=-27.5,
                  ber='0' if metric == 'ber' else '', per='0' if metric == 'per' else '',
                  reference_bytes=100000)
    kept.append(anchor)
    kept.sort(key=lambda r: (int(r['mcs']), float(r['relative_power_db'])))
    with (OUT / f'{metric}_relative.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(kept)
    assert len(kept) == 69
    assert [r for r in original if r['mcs'] != '11'] == [r for r in kept if r['mcs'] != '11']
    assert all(r in kept for r in original if r['mcs'] == '11' and -float(r['relative_power_db']) >= 28)
    provenance.append(dict(metric=metric, source=str(source), source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                           excluded_measurement_points=16, retained_measurement_points=68,
                           synthetic_anchor=anchor, anchor_is_measured=False))
    orig11 = sorted([r for r in original if r['mcs'] == '11'], key=lambda r: -float(r['relative_power_db']))
    new11 = sorted([r for r in kept if r['mcs'] == '11'], key=lambda r: -float(r['relative_power_db']))
    ax.plot([-float(r['relative_power_db']) for r in orig11], [float(r[metric]) for r in orig11], 'o--', label='元の測定集計', alpha=.6)
    ax.plot([20, 27.5] + [-float(r['relative_power_db']) for r in new11[1:]],
            [0, 0] + [float(r[metric]) for r in new11[1:]], '-', label='理想変換用（27.5 dB以下は0と仮定）')
    ax.scatter([27.5], [0], marker='*', s=130, color='red', label='仮定のゼロ点（実測ではない）', zorder=5)
    ax.axvspan(20, 27.5, alpha=.1, color='gray')
    ax.set(xlabel='外部減衰量 [dB]', ylabel=metric.upper())
    ax.grid(True, alpha=.25)
    if metric == 'ber':
        ax.set_yscale('symlog', linthresh=1e-8)
        ax.set_ylim(bottom=0)
    else:
        ax.set_ylim(-.03, 1.03)
        ax.legend(fontsize=8)
fig.suptitle('MCS 11：高電力側の疑わしい区間を除外した派生曲線\n28 dB以降も歪みがないと実証したわけではない／絶対受信電力は未校正')
fig.savefig(OUT / 'mcs11_comparison.png', dpi=160)
fig.savefig(OUT / 'mcs11_comparison.pdf')
with (OUT / 'excluded_points.csv').open('w', newline='') as stream:
    writer = csv.DictWriter(stream, fieldnames=list(excluded[0]))
    writer.writeheader()
    writer.writerows(excluded)
(OUT / 'provenance.json').write_text(json.dumps(dict(
    calibration_status='ideal_linear_assumption_relative_power_only', simulator_schema_compatible=False,
    user_instruction='AMAM/AMPMをモデル化せず理想変換とし、歪みが疑われる部分を除外する',
    cutoff_attenuation_db=27.5, cutoff_is_provisional=True,
    high_power_extrapolation='MCS11のみ、減衰27.5 dB以下はBER/PER=0という仮定',
    other_mcs='保持。歪みの根拠が明確でない微小誤りを一律に削除しない',
    distortion_free_remaining_region_is_verified=False, curves=provenance
), ensure_ascii=False, indent=2), encoding='utf-8')
print('68実測点を保持、16点除外、仮定のゼロ点1点を追加。原測定曲線は変更なし。')
