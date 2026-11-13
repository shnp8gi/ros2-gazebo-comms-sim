"""生成した表が拡張仕様書の付録 A.2・A.3 と一致し、config/ の CSV とも一致することを確かめる。"""
from pathlib import Path

from make_rate_tables import tables

# 付録 A.2（RSSI がこの値以上なら、このレート）
A2 = {
    'thz_rate_2p16.csv': [(-67, 1.13), (-64, 2.26), (-61, 2.87), (-59, 3.39), (-57, 4.52),
                          (-54, 5.75), (-52, 6.78), (-48, 8.62)],
    'thz_rate_4p32.csv': [(-63, 2.26), (-62, 2.87), (-60, 4.52), (-59, 5.75), (-57, 6.78),
                          (-55, 9.03), (-52, 11.50), (-48, 13.55), (-46, 17.25)],
    'thz_rate_8p64.csv': [(-60, 4.52), (-59, 5.75), (-57, 9.03), (-56, 11.50), (-54, 13.55),
                          (-52, 18.07), (-49, 23.00), (-45, 27.10), (-43, 34.50)],
}
# 付録 A.3
A3 = [(-61, 2.26), (-58, 2.87), (-55, 4.52), (-51, 5.75), (-49, 6.78), (-45, 8.62), (-39, 11.50)]
CONFIG = Path(__file__).resolve().parents[1] / 'src/comms_sim_pkg/config'


def test_tables_match_specification():
    generated = tables()
    for name, expected in A2.items():
        assert generated[name] == expected
    assert generated['hrcp_rate_2p16.csv'] == A3


def test_config_files_match_generated():
    for name, rows in tables().items():
        lines = [l for l in (CONFIG / name).read_text().splitlines() if l and not l.startswith('#')]
        parsed = [tuple(float(v) for v in line.split(',')) for line in lines]
        assert parsed == [(float(s), r) for s, r in rows]
