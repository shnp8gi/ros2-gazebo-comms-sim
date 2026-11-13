#!/usr/bin/env python3
"""IEEE Std 802.15.3-2023 の基準受信感度とデータレートから RSSI→レート表を作る。

各 MCS は「受信電力が基準受信感度以上なら使える」とし、使える MCS のうち
レートが最大のものを選ぶ。出力は MCStable.csv と同じ形式（RSSI [dBm], レート [Gb/s]）。
"""
import argparse
from pathlib import Path

# THz-SC PHY。Table 14-14（基準受信感度 [dBm]）と Table 14-5（PW 付きレート [Gb/s]）。
# 列は 2.16 / 4.32 / 8.64 GHz 幅。MCS 12〜15 は使わない（拡張仕様書 §6.3）。
THZ_SC = {
    0: ((-67, -63, -60), (1.13, 2.26, 4.52)),
    1: ((-64, -62, -59), (1.44, 2.87, 5.75)),
    2: ((-64, -60, -57), (2.26, 4.52, 9.03)),
    3: ((-61, -59, -56), (2.87, 5.75, 11.50)),
    4: ((-59, -56, -53), (3.39, 6.78, 13.55)),
    5: ((-56, -54, -51), (4.31, 8.62, 17.25)),
    6: ((-59, -57, -54), (3.39, 6.78, 13.55)),
    7: ((-56, -54, -51), (4.31, 8.62, 17.25)),
    8: ((-57, -55, -52), (4.52, 9.03, 18.07)),
    9: ((-54, -52, -49), (5.75, 11.50, 23.00)),
    10: ((-52, -48, -45), (6.78, 13.55, 27.10)),
    11: ((-48, -46, -43), (8.62, 17.25, 34.50)),
}
# HRCP-SC PHY、2.16 GHz 幅。Table 13-18 と Table 13-6（PW 付き）。
HRCP_SC = {
    0: (-61, 2.26), 1: (-58, 2.87), 2: (-55, 4.52), 3: (-51, 5.75),
    4: (-49, 6.78), 5: (-45, 8.62), 6: (-39, 11.50),
}
THZ_FILES = ('thz_rate_2p16.csv', 'thz_rate_4p32.csv', 'thz_rate_8p64.csv')
HRCP_FILE = 'hrcp_rate_2p16.csv'


def envelope(pairs):
    """(感度, レート) の一覧から、RSSI が上がるとレートが上がる行だけを残す。"""
    rows, best = [], 0.0
    for sensitivity in sorted({s for s, _ in pairs}):
        rate = max(r for s, r in pairs if s <= sensitivity)
        if rate > best:
            rows.append((sensitivity, rate))
            best = rate
    return rows


def tables():
    result = {
        name: envelope([(s[i], r[i]) for s, r in THZ_SC.values()])
        for i, name in enumerate(THZ_FILES)
    }
    result[HRCP_FILE] = envelope(list(HRCP_SC.values()))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default=str(Path(__file__).resolve().parents[1] / 'src/comms_sim_pkg/config'))
    args = parser.parse_args()
    for name, rows in tables().items():
        source = 'Table 13-18, 13-6' if name == HRCP_FILE else 'Table 14-14, 14-5'
        lines = [f'# IEEE Std 802.15.3-2023 {source}. tools/make_rate_tables.py で生成']
        lines += [f'{s}, {r:.2f}' for s, r in rows]
        (Path(args.output) / name).write_text('\n'.join(lines) + '\n')
        print(f'{name}: {len(rows)} 行')


if __name__ == '__main__':
    main()
