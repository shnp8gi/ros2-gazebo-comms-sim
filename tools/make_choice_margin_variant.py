#!/usr/bin/env python3
"""
既存シナリオの RSU 配置と接続閾値だけを差し替えた変種を作る。

なぜ要るか
----------
「予測はいつ効くか」を測るには、**選択の余地** (ある時刻に接続可能な RSU が
2基以上ある区間の割合) を制御した一連のシナリオが要る。ところが
gen_urban_scenario.py で作り直すと、固定遮蔽や target_ratio といった
手で入れた設定まで戻ってしまう。そこで既存シナリオを土台に、
RSU 実体と snr_min_db だけを置き換える。

配置は tools/lib/road_geometry.py の rsu_tilt_pattern='dual' と同じで、
各ポールに上流面 (yaw=-180°+δ') と下流面 (yaw=-δ') を載せる。同じ向きの面
どうしがポール間隔だけ離れて並ぶので、間隔が窓長より短ければ担当領域が
重なり、選択の余地が生まれる。

現行シナリオ (alternate・2基) は2基が互いに逆を向くため窓が原理的に交わらず、
実測で「接続可能な RSU が2基以上の時刻 = 0.000%」だった。

生成した変種の選択余地率は tools/analyze_connection_windows.py で確認できる。

  python3 tools/make_choice_margin_variant.py \
      --base config/scenarios/road_urban_2lane.yaml \
      --out config/scenarios/urban_dual_s10.yaml \
      --name urban_dual_s10 --poles 2 --spacing 10 --tilt-deg 70 --snr-min 16.5
"""
import argparse
import math
import sys

import yaml


def rsu_entities(poles, spacing, tilt_deg, rsu_y, rsu_z):
    """dual 配置の rx 実体を作る (ポール数 x 2 面)。"""
    ents = []
    idx = 0
    for p in range(poles):
        x = (p - (poles - 1) / 2.0) * spacing
        # δ_rsu < 0 が上流向き (下り車担当)、> 0 が下流向き。
        # yaw = -90° + δ_rsu (gen_urban_scenario.py と同じ規約)
        for d in (-tilt_deg, +tilt_deg):
            yaw = math.radians(-90.0 + d)
            name = f'rsu_{idx}'
            ents.append({
                'name': name, 'model': 'antenna', 'role': 'rx', 'static': True,
                'pose': [round(x, 6), rsu_y, 0.0, 0.0, 0.0, round(yaw, 6)],
                'antennas': [{'name': name, 'offset': [0.0, 0.0, rsu_z],
                              'relative_rpy': [0.0, 0.0, 0.0]}],
            })
            idx += 1
    return ents


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--base', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--name', required=True)
    ap.add_argument('--poles', type=int, default=2)
    ap.add_argument('--spacing', type=float, default=10.0)
    ap.add_argument('--tilt-deg', type=float, default=70.0,
                    help='δ_rsu [deg] (gen_urban_scenario.py と同じ規約)')
    ap.add_argument('--veh-tilt-deg', type=float, default=None,
                    help='δ_veh [deg]。省略で 90 - δ_rsu (相互ボアサイト成立)。'
                         'δ_rsu を変えたらこちらも動かさないと、向きの効果と'
                         'ボアサイトずれの効果が交絡する')
    ap.add_argument('--snr-min', type=float, default=None,
                    help='接続に要る SNR [dB]。省略で土台のまま')
    a = ap.parse_args()

    doc = yaml.safe_load(open(a.base, encoding='utf-8'))
    s = doc['scenario']

    old = [e for e in s['entities'] if e.get('role') == 'rx']
    if not old:
        sys.exit('土台に role: rx の実体がありません')
    rsu_y = old[0]['pose'][1]
    rsu_z = old[0]['antennas'][0]['offset'][2]

    new_rsu = rsu_entities(a.poles, a.spacing, a.tilt_deg, rsu_y, rsu_z)
    others = [e for e in s['entities'] if e.get('role') != 'rx']
    s['entities'] = new_rsu + others

    # 車載アンテナの向き。δ_rsu + δ_veh = 90° が相互ボアサイトの条件なので、
    # 既定では δ_rsu に追従させる。前方アンテナが ±δ_veh、後方が ±(180° - δ_veh)
    veh_tilt = (a.veh_tilt_deg if a.veh_tilt_deg is not None
                else 90.0 - a.tilt_deg)
    v = math.radians(veh_tilt)
    rear = math.pi - v
    tmpl = s['traffic']['target_template']

    def yaw_pair(val):
        return {1: round(val, 6), -1: round(-val, 6)}

    tmpl['relative_yaw_by_direction'] = yaw_pair(v)
    for ant in tmpl.get('antennas', []):
        base = rear if ant.get('suffix') == 'rear' else v
        ant['relative_yaw_by_direction'] = yaw_pair(base)

    s['name'] = a.name
    snr_txt = ''
    if a.snr_min is not None:
        rate = (s['config_overrides']['comms_simulator_node']['ros__parameters']
                ['rate_model'])
        rate['snr_min_db'] = float(a.snr_min)
        nf = float(rate.get('noise_floor_dbm', -95.0))
        rssi_min = nf + float(a.snr_min)
        # 接続閾値を動かしたら L3 の実行可能性ゲートも一緒に動かすこと。
        # kkf_idle_lcb_db は「平均 mu がこれ未満のペアは割当対象から外す」判定で
        # (kkf_scheduler_node._replan_hungarian)、接続閾値と一致していないと
        # 意味が変わる。高すぎれば繋がるペアを捨て、低すぎれば繋がらないペアを掴む。
        # 実測: ゲートだけ 10 dB 高く残した結果、情報上界であるはずの oracle が
        # assoc_hold を 5.3% 下回った (新たに繋がる領域をすべて拒否していた)。
        # 追従させた再測定では +18.1% になった
        link = s['config_overrides']['link_controller_node']['ros__parameters']
        link['kkf_idle_lcb_db'] = rssi_min
        snr_txt = (f"、接続閾値 {rssi_min:.1f} dBm (snr_min {a.snr_min} dB、"
                   f"kkf_idle_lcb_db も同値に追従)")
    s['description'] = (
        f"選択余地の掃引用変種。RSU dual 配置 (ポール {a.poles} 本 x 上流/下流2面 "
        f"= 論理 RSU {len(new_rsu)} 基)、ポール間隔 {a.spacing} m、"
        f"δ_rsu ±{a.tilt_deg}° / δ_veh ±{veh_tilt}°"
        f"{snr_txt}。土台 {a.base} から tools/make_choice_margin_variant.py で生成。"
        f"選択余地率は tools/analyze_connection_windows.py で確認すること")

    with open(a.out, 'w', encoding='utf-8') as f:
        f.write('# tools/make_choice_margin_variant.py により生成 — 手編集しないこと\n')
        yaml.safe_dump(doc, f, allow_unicode=True, sort_keys=False, width=200)
    print(f'generated: {a.out}')
    for e in new_rsu:
        print('  %-8s x=%7.2f  yaw=%8.2f deg'
              % (e['name'], e['pose'][0], math.degrees(e['pose'][5])))
    return 0


if __name__ == '__main__':
    sys.exit(main())
