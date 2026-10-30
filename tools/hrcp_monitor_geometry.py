"""通信時刻と設定軌道から俯瞰位置・状態色を得る（実機計測ではない）。"""
import numpy as np

JUDGEMENT_LABELS = ('未判定（?）', '両方向の受信条件未達\nP=0 / Q=0',
                    'PRC → PRDEVだけ条件未達\nP=0 / Q=1',
                    'PRDEV → PRCだけ条件未達\nP=1 / Q=0',
                    '両方向の受信条件OK\nP=1 / Q=1', 'リンク確立\n接続手順完了')
JUDGEMENT_COLORS = ('#687580', '#b8c2cc', '#d78aff', '#ff655f', '#68b8f3', '#76df98')

def judgement_state(ab, ba, connected=False):
    if connected:
        return 5
    if ab == 0 or ba == 0:
        return 0
    return {(1,1):4, (-1,1):3, (1,-1):2, (-1,-1):1}[ab,ba]


def xor_unmet(ab, ba, connected=False):
    """Q=A→B、P=B→A。既知の充足/不充足が逆の場合だけ不足方向を返す。"""
    if connected:
        return False, False
    return (ab, ba) == (-1, 1), (ab, ba) == (1, -1)


def direction_styles(ab, ba, connected):
    q_unmet, p_unmet = xor_unmet(ab, ba, connected)
    styles = []
    for result, highlighted, color, name in ((ab, q_unmet, '#ff655f', 'Q：A → B不足'),
                                             (ba, p_unmet, '#d78aff', 'P：B → A不足')):
        if connected:
            styles.append(('#76df98', 'リンク確立済み'))
        elif highlighted:
            styles.append((color, name))
        elif result == 1:
            styles.append(('#68b8f3', '充足確認・接続待ち'))
        else:
            styles.append(('#b8c2cc', '_nolegend_'))
    return styles


def positions(config, seconds):
    t = np.asarray(seconds, dtype=float) - config['encounter_time_s']
    a = np.column_stack((t * config['drone_a']['speed_kmh'] / 3.6, np.zeros_like(t)))
    if config['scenario'] == 'parallel':
        b = np.column_stack((-t * config['drone_b']['speed_kmh'] / 3.6,
                             np.full_like(t, config['lateral_separation_m'])))
    else:
        b = np.column_stack((np.zeros_like(t), t * config['drone_b']['speed_kmh'] / 3.6))
    return a, b


def antenna_angles(config, seconds):
    """C++ geometry()と同じ指向角。方位は+X基準、仰角は上向き正。"""
    if config.get('antenna_mode', 'fixed') != 'tracking':
        return ((90., float(config['drone_a'].get('fixed_elevation_deg', 45))),
                (-90. if config['scenario'] == 'parallel' else 0.,
                 float(config['drone_b'].get('fixed_elevation_deg', -45))))
    a, b = positions(config, [seconds])
    delta = np.r_[b[0] - a[0],
                  config['drone_b']['altitude_m'] - config['drone_a']['altitude_m']]
    return tuple((float(np.degrees(np.arctan2(d[1], d[0]))),
                  float(np.degrees(np.arctan2(d[2], np.hypot(d[0], d[1])))))
                 for d in (delta, -delta))


def state_style(ab, ba, connected):
    if connected:
        return '#76df98', 'リンク確立済み'
    if (ab, ba) == (-1, 1):
        return '#ff655f', 'A → Bのみ不充足確認'
    if (ab, ba) == (1, -1):
        return '#d78aff', 'B → Aのみ不充足確認'
    if (ab, ba) == (-1, -1):
        return '#e79a54', '両方向不充足確認'
    if ab == 0 or ba == 0:
        return '#b8c2cc', '未判定を含む'
    return '#68b8f3', '両方向充足確認・接続待ち'
