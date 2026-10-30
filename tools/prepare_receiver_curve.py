#!/usr/bin/env python3
"""TJXの条件別CSVを、②モデル用の受信電力–BER/PER表へ変換する。実機へは接続しない。"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path


FIELDS = ['direction', 'frame', 'mcs', 'rssi_dbm', 'ber', 'per', 'reference_bytes']


def finite(value, name):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f'{name}は有限値で指定してください')
    return value


def aggregate_samples(samples, mcs, metric, aggregation, allow_detect_weighted_per=False):
    """区間値は分母で重み付けする。累積値は末尾だけを採用する。"""
    if aggregation not in ('averaged', 'clear_write'):
        raise ValueError('aggregationはaveragedまたはclear_writeです')
    selected = samples[-1:] if aggregation == 'averaged' else samples
    weighted, weights, raw_rssi = [], [], []
    previous_time = None
    exact_per = all(row.get('packet_lost', '').strip() for row in selected)
    if aggregation == 'clear_write' and metric == 'per' and not exact_per and not allow_detect_weighted_per:
        raise ValueError('clear_writeのPER合算にはpacket_lost列が必要です。近似を使う場合は--allow-detect-weighted-perを明示してください')
    for row in selected:
        if finite(row['MCS'], 'MCS') != mcs:
            raise ValueError('測定CSVと条件表のMCSが一致しません')
        value = finite(row['PER' if metric == 'per' else 'bER'], metric)
        if not 0 <= value <= 1:
            raise ValueError('BER/PERは0〜1で記録してください')
        packet = finite(row['packet'], 'packet')
        if packet < 0 or not packet.is_integer():
            raise ValueError('packetは0以上の整数です')
        weight = packet
        if aggregation == 'clear_write' and metric == 'per' and exact_per:
            lost = finite(row['packet_lost'], 'packet_lost')
            if lost < 0 or not lost.is_integer():
                raise ValueError('packet_lostは0以上の整数です')
            weight += lost
        if weight <= 0:
            raise ValueError('分母0のツール既定値を実測BER/PERと見なしません。受信不能条件は別途記録してください')
        rssi = finite(row['RSSI'], 'RSSI')
        if not 0 <= rssi <= 511:
            raise ValueError('TJXの生RSSIは0〜511です')
        if aggregation == 'clear_write':
            time = finite(row['time'], 'time')
            if previous_time is not None and time <= previous_time:
                raise ValueError('clear_writeのtimeは単調増加である必要があります')
            previous_time = time
        weighted.append(weight * value)
        weights.append(weight)
        raw_rssi.append(rssi)
    method = ('last_cumulative_value' if aggregation == 'averaged' else
              'detected_packet_weighted_ber' if metric == 'ber' else
              'detected_plus_lost_weighted_per' if exact_per else
              'detected_packet_weighted_per_approximation')
    return math.fsum(weighted) / math.fsum(weights), dict(
        method=method, used_rows=len(selected), denominator_sum=math.fsum(weights),
        rssi_raw_min=min(raw_rssi), rssi_raw_max=max(raw_rssi))


def prepare(manifest, output, metric='per', allow_unknown_ber=False,
            allow_detect_weighted_per=False, relative_power=False):
    manifest, output = Path(manifest).resolve(), Path(output).resolve()
    provenance_path = output.with_suffix(output.suffix + '.provenance.json')
    if output.exists() or provenance_path.exists():
        raise ValueError('出力先には既存ファイルがあります。別の名前を指定してください')
    curves, sources, seen = [], [], set()
    with manifest.open(encoding='utf-8-sig', newline='') as stream:
        conditions = list(csv.DictReader(stream))
    for index, condition in enumerate(conditions, 2):
        try:
            direction = condition['direction'].strip()
            if direction not in ('P_B_TO_A', 'Q_A_TO_B'):
                raise ValueError('directionはP_B_TO_AまたはQ_A_TO_Bです')
            frame = condition['frame'].strip()
            if frame not in ('ALL', 'CONTROL', 'BEACON', 'REQUEST', 'RESPONSE', 'ACK', 'DATA', 'DATA_ACK', 'DISASSOCIATE'):
                raise ValueError('frameが不正です')
            mcs, length = int(condition['mcs']), int(condition['frame_bytes'])
            if not 0 <= mcs <= 255 or length <= 0:
                raise ValueError('MCS番号またはパケット長が不正です')
            aggregation = condition['aggregation'].strip()
            stage = condition.get('ber_stage', 'unknown').strip()
            if metric == 'ber' and stage not in ('post_fec', 'uncoded') and not allow_unknown_ber:
                raise ValueError('BERの測定位置が未確定です。実測PERを使うか、独立残留誤りの近似として--allow-unknown-berを明示してください')
            source = (manifest.parent / condition['statistics_csv']).resolve()
            with source.open(encoding='utf-8-sig', newline='') as stream:
                samples = list(csv.DictReader(stream))
            if not samples:
                raise ValueError('測定CSVにデータがありません')
            # 最終値が無効なら過去の成功値に戻さない。受信不能を隠さないため。
            last = samples[-1]
            value, aggregation_info = aggregate_samples(samples, mcs, metric, aggregation,
                                                        allow_detect_weighted_per)
            attenuation = finite(condition['attenuation_db'], 'attenuation_db')
            baseline = finite(condition['reference_attenuation_db'], 'reference_attenuation_db')
            reference = 0.0 if relative_power else finite(condition['reference_rssi_dbm'], 'reference_rssi_dbm')
            if attenuation < 0 or baseline < 0:
                raise ValueError('減衰量は0以上で指定してください')
            power = reference - (attenuation - baseline)
            raw_rssi = finite(last['RSSI'], 'RSSI')
            if not 0 <= raw_rssi <= 511:
                raise ValueError('TJXの生RSSIは0〜511です')
            identity = (direction, frame, mcs, power)
            if identity in seen:
                raise ValueError('同一曲線・電力の測定点が重複しています。測定数で集約してから入力してください')
            seen.add(identity)
            power_field = 'relative_power_db' if relative_power else 'rssi_dbm'
            curves.append(dict(direction=direction, frame=frame, mcs=mcs, **{power_field: power},
                               ber=value if metric == 'ber' else '', per=value if metric == 'per' else '',
                               reference_bytes=length))
            sources.append(dict(source=str(source), sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                                condition=condition, last_measurement=last, rssi_raw=raw_rssi,
                                aggregation=aggregation_info))
        except (KeyError, ValueError, OSError) as exc:
            raise ValueError(f'条件表 {index}行目: {exc}') from exc
    groups = {}
    for row in curves:
        key = (row['direction'], row['frame'], row['mcs'])
        groups.setdefault(key, []).append(row)
    if not groups:
        raise ValueError('条件表が空です')
    for key, rows in groups.items():
        if len(rows) < 2 or len({r['reference_bytes'] for r in rows}) != 1:
            raise ValueError(f'{key}: 同じ長さの測定点を2点以上用意してください')
    power_field = 'relative_power_db' if relative_power else 'rssi_dbm'
    curves.sort(key=lambda r: (r['direction'], r['frame'], r['mcs'], r[power_field]))
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=[power_field if f == 'rssi_dbm' else f for f in FIELDS])
        writer.writeheader()
        writer.writerows(curves)
    provenance_path.write_text(json.dumps(dict(manifest=str(manifest),
        manifest_sha256=hashlib.sha256(manifest.read_bytes()).hexdigest(), metric=metric,
        allow_unknown_ber=allow_unknown_ber, allow_detect_weighted_per=allow_detect_weighted_per,
        calibration_status='relative_power_only' if relative_power else 'measured_unvalidated',
        simulator_schema_compatible=not relative_power,
        power_mapping=('-(attenuation_db - reference_attenuation_db); NOT dBm' if relative_power else
                       'reference_rssi_dbm - (attenuation_db - reference_attenuation_db); fixed gain assumed'),
        sources=sources), ensure_ascii=False, indent=2))
    return len(curves)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest', type=Path, help='方向・減衰量・基準電力・測定CSVを対応付けた条件表')
    parser.add_argument('output', type=Path)
    parser.add_argument('--metric', choices=['ber', 'per'], default='per')
    parser.add_argument('--allow-unknown-ber', action='store_true', help='FEC前後未確定のBERを独立残留誤りとする近似を明示')
    parser.add_argument('--allow-detect-weighted-per', action='store_true', help='lost数不明のclear_write PERを検出数で重み付けする近似を明示')
    parser.add_argument('--relative-power', action='store_true', help='dBm基準が未確定の場合に相対電力表を出力。シミュレーターへ直接入力不可')
    args = parser.parse_args()
    try:
        count = prepare(args.manifest, args.output, args.metric, args.allow_unknown_ber,
                        args.allow_detect_weighted_per, args.relative_power)
    except (ValueError, OSError) as exc:
        parser.exit(2, f'変換できません: {exc}\n')
    print(f'{count}測定点を保存しました: {args.output}')
