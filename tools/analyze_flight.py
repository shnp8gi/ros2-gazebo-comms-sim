#!/usr/bin/env python3
"""飛行の記録（trace.bin）から、姿勢・アンテナのぶれ・最接近距離を求める。

等速区間（最接近の前後 window 秒）について、機体の前傾、アンテナのぶれ角
（取り付けどおりの向きからのずれ）の大きさとスペクトルのピークを出す。
"""
import argparse
import json
import math
from pathlib import Path
import numpy as np
import yaml

COLUMNS = 37


def load(directory):
    directory = Path(directory)
    meta = json.loads((directory / 'trace.json').read_text())
    trace = np.fromfile(directory / 'trace.bin').reshape(-1, COLUMNS)
    config = yaml.safe_load((directory / 'flight.yaml').read_text())
    return meta, trace, config


def rotation(q):
    """クォータニオン (w, x, y, z) の配列 → 回転行列の配列。"""
    w, x, y, z = q.T
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], -1),
        np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], -1),
        np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], -1)], -2)


def mount(elevation_deg):
    """機体←アンテナの既定の取り付け（左真横、仰角 elevation_deg）。"""
    e, c = math.radians(elevation_deg), math.cos(math.pi / 2)
    rz = np.array([[c, -1, 0], [1, c, 0], [0, 0, 1]])
    ry = np.array([[math.cos(-e), 0, math.sin(-e)], [0, 1, 0], [-math.sin(-e), 0, math.cos(-e)]])
    return rz @ ry


def analyze(directory, window=5.0):
    meta, trace, config = load(directory)
    step, time = meta['step_s'], trace[:, 0]
    distance = np.linalg.norm(trace[:, 19:22] - trace[:, 1:4], axis=1)
    closest = int(np.argmin(distance))
    span = slice(max(0, closest - int(window / step)), min(len(trace), closest + int(window / step)))
    result = {'d_min_m': float(distance[closest]), 'closest_time_s': float(time[closest]),
              'window_s': [float(time[span][0]), float(time[span][-1])]}
    elevation = config['antenna']['elevation_deg']
    for m, (name, sign) in enumerate((('a', 1), ('b', -1))):
        o = 1 + 18 * m
        body, antenna = rotation(trace[span, o + 3:o + 7]), rotation(trace[span, o + 10:o + 14])
        # 機首方向の前傾（機首下げが正）：機体の x 軸の鉛直成分から求める
        pitch = np.degrees(np.arcsin(np.clip(-body[:, 2, 0], -1, 1)))
        roll = np.degrees(np.arctan2(body[:, 2, 1], body[:, 2, 2]))
        # ぶれ角：取り付けどおりの向き（機体×取り付け）に対するアンテナ正面のずれを、アンテナ座標で表す
        nominal = body @ mount(sign * elevation)
        local = np.einsum('nji,njk->nik', nominal, antenna)  # nominal^T · antenna
        yaw_dev, pitch_dev = np.degrees(local[:, 1, 0]), np.degrees(-local[:, 2, 0])
        total = np.degrees(np.arccos(np.clip(local[:, 0, 0], -1, 1)))
        rotor = np.abs(trace[span, o + 14:o + 18]).mean()
        entry = {'pitch_mean_deg': float(pitch.mean()), 'roll_mean_deg': float(roll.mean()),
                 'body_pitch_pp_deg': float(np.ptp(pitch)),
                 'deflection_mean_deg': float(total.mean()), 'deflection_max_deg': float(total.max()),
                 'deflection_pitch_pp_deg': float(np.ptp(pitch_dev)), 'deflection_yaw_pp_deg': float(np.ptp(yaw_dev)),
                 'rotor_speed_mean_rad_s': float(rotor), 'rotor_frequency_hz': float(rotor / 2 / math.pi)}
        # スペクトルのピーク（平均を引いた後の上位3つ）
        for label, series in (('pitch', pitch_dev), ('yaw', yaw_dev)):
            spectrum = np.abs(np.fft.rfft((series - series.mean()) * np.hanning(len(series)))) * 4 / len(series)
            frequency = np.fft.rfftfreq(len(series), step)
            keep = frequency >= 5
            order = np.argsort(spectrum[keep])[::-1]
            peaks, seen = [], []
            for index in order:
                f = float(frequency[keep][index])
                if all(abs(f - g) > 5 for g in seen):
                    seen.append(f)
                    peaks.append({'frequency_hz': round(f, 1), 'amplitude_deg': float(spectrum[keep][index])})
                if len(peaks) == 3:
                    break
            entry[f'deflection_{label}_peaks'] = peaks
        result[f'drone_{name}'] = entry
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory')
    parser.add_argument('--window', type=float, default=5.0, help='最接近の前後で見る時間 [s]')
    args = parser.parse_args()
    print(json.dumps(analyze(args.directory, args.window), indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
