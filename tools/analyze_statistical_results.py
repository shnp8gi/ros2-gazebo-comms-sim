#!/usr/bin/env python3
"""確率判定ログを検査し、方向・フレーム・MCS・電力帯ごとの集計を保存する。"""
import argparse
import csv
import math
from pathlib import Path
import yaml


def analyze(directory):
    directory = Path(directory)
    c = yaml.safe_load((directory/'config.yaml').read_text())
    if c.get('phy_error_model') != 'empirical_packet_v1':
        raise ValueError('②モデルの結果を指定してください')
    groups = {}
    count = 0
    with (directory/'frames.csv').open() as stream:
        for row in csv.DictReader(stream):
            count += 1
            per, draw = float(row['per']), float(row['random_draw'])
            if not 0 <= per <= 1 or not 0 <= draw < 1:
                raise ValueError('PERまたは乱数値が不正です')
            physical_reject = row['reason'] in ('HALF_DUPLEX_OVERLAP', 'NO_PAYLOAD_MCS')
            if not physical_reject and bool(int(row['success'])) != (draw >= per):
                raise ValueError('抽選とフレーム成否が一致しません')
            if int(row['processed_samples']) != 0:
                raise ValueError('②モデルで波形sampleを処理しています')
            key = (row['direction'], row['frame'], row['mcs'], math.floor(float(row['lookup_rssi_dbm'])))
            item = groups.setdefault(key, [0, 0, 0., 0])
            item[0] += 1
            item[1] += not bool(int(row['success']))
            item[2] += per
            item[3] += physical_reject
    if not count:
        raise ValueError('フレームが記録されていません')
    with (directory/'probability_bins.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['direction','frame','mcs','lookup_rssi_bin_dbm','frames','failed',
                         'observed_failure_rate','mean_model_per','half_duplex_or_no_mcs'])
        for key, (n, failed, per, rejected) in sorted(groups.items()):
            writer.writerow([*key,n,failed,failed/n,per/n,rejected])
    result = {'checked_frames':count,'probability_draws_match':True,'waveform_samples_processed':0,
              'calibration_status':c.get('calibration_status'),'metric':c['empirical_phy']['metric'],
              'seed':c['empirical_phy']['seed'],'received_payload_accounting':c['payload_accounting']}
    (directory/'statistical_verification.yaml').write_text(yaml.safe_dump(result, allow_unicode=True))
    print(f'確率判定検証成功: {count}フレーム。集計: probability_bins.csv')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    analyze(args.directory)
