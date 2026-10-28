#!/usr/bin/env python3
"""仮I/Q PHYの実行ログからsample数・時刻・受信判定を検証する。"""
import argparse
import csv
import json
import math
from collections import Counter
from pathlib import Path
import yaml
from validate_drone_results import validate


def validate_phy(directory):
    root = Path(directory)
    summary = validate(root)
    c = yaml.safe_load((root / 'config.yaml').read_text())
    if c.get('phy_error_model') != 'lab_iq_rect_v0_uncoded':
        raise ValueError('仮I/Q PHYの結果ではありません')
    reasons = Counter()
    count = samples = errors = bits = 0
    fs = 3.52e9
    for row in csv.DictReader((root / 'frames.csv').open()):
        count += 1
        start, size, rx, processed = (int(row[k]) for k in
            ('tx_start_sample', 'tx_samples', 'rx_start_sample', 'processed_samples'))
        expected_bytes = (c['lab_phy']['data_bytes'] if row['frame'] == 'DATA'
                          else 0 if row['frame'] in ('ACK', 'DATA_ACK') else 16)
        pre = 5120 if row['direction'] == 'A_TO_B' and row['frame'] != 'DATA' else 3328
        expected_samples = 2*(pre+208+(expected_bytes*4+16 if expected_bytes else 0)+10)
        assert size == expected_samples and rx >= start
        assert int(row['frame_bits']) == expected_bytes*8
        assert abs(float(row['frame_start_s'])-start/fs) < 1e-12
        assert abs(float(row['frame_end_s'])-(rx+size)/fs) < 1e-12
        assert processed in (0, size)
        if processed == 0:
            assert row['reason'] in ('RSSI_BELOW_SENSITIVITY', 'SNR_BELOW_REQUIRED', 'RECEIVER_TRANSMITTING')
        else:
            expected_bits = expected_bytes*8+32 if expected_bytes else 0
            assert int(row['checked_bits']) == expected_bits
        assert (row['success'] == '1') == (row['reason'] == '')
        assert float(row['random_draw']) == -1 and math.isnan(float(row['per']))
        reasons[row['reason'] or 'SUCCESS'] += 1
        samples += processed
        errors += int(row['bit_errors'])
        bits += int(row['checked_bits'])
    assert count > 0
    assert abs(float(summary['total_data_MB'])-int(summary['acked_data_frames'])*c['lab_phy']['data_bytes']/1e6)<1e-8
    result = dict(condition=root.name,frames=count,processed_samples=samples,bit_errors=errors,
                  checked_bits=bits,reasons=dict(reasons),sample_checks_passed=True)
    (root / 'phy_validation.json').write_text(json.dumps(result, indent=2, ensure_ascii=False)+'\n')
    print(json.dumps(result, ensure_ascii=False))
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directories', nargs='+')
    for name in parser.parse_args().directories:
        validate_phy(name)
