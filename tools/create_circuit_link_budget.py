#!/usr/bin/env python3
"""既存YAMLから正対時のリンクバジェットと回路要求の仮定例をCSVにする。"""

import argparse
import csv
import hashlib
import math
from pathlib import Path

import yaml


def budget(radio, distance, exponent):
    """drone_link.cppの電力・雑音式と同じ基準点で計算する。"""
    # PyYAMLは61.0e9等を文字列として読む場合がある。
    radio = {key: float(value) if value is not None else None for key, value in radio.items()}
    tx = 10 * math.log10(radio["power_mw"])
    path_loss = 20 * math.log10(4 * math.pi * radio["frequency_hz"] / 299792458)
    path_loss += 10 * exponent * math.log10(max(distance, 1.0))
    eirp = tx - radio["tx_insertion_loss_db"] + radio["tx_antenna_gain_dbi"]
    antenna = eirp - path_loss + radio["rx_antenna_gain_dbi"]
    lna = antenna + radio["rx_lna_gain_db"]
    mixer = lna + radio["rx_conversion_gain_db"]
    receiver = mixer - radio["rx_insertion_loss_db"]
    gain = receiver - antenna
    thermal = 10 * math.log10(
        1.380649e-23 * radio["noise_temperature_k"] * radio["noise_bandwidth_hz"]
    ) + 30
    noise = thermal + radio["receiver_noise_figure_db"]
    return {
        "距離_m": distance,
        "周波数_GHz": radio["frequency_hz"] / 1e9,
        "送信出力_dBm": tx,
        "送信挿入損失_dB": radio["tx_insertion_loss_db"],
        "送信アンテナ利得_dBi": radio["tx_antenna_gain_dbi"],
        "EIRP_dBm": eirp,
        "伝搬損失_dB": path_loss,
        "受信アンテナ利得_dBi": radio["rx_antenna_gain_dbi"],
        "アンテナ端信号_dBm": antenna,
        "LNA利得_dB": radio["rx_lna_gain_db"],
        "LNA後信号_dBm": lna,
        "変換利得_dB": radio["rx_conversion_gain_db"],
        "Mixer後信号_dBm": mixer,
        "受信挿入損失_dB": radio["rx_insertion_loss_db"],
        "受信系総利得_dB": gain,
        "TJX入力信号_dBm": receiver,
        "TJX入力閾値_dBm": radio["sensitivity_dbm"],
        "電力閾値との差_dB": receiver - radio["sensitivity_dbm"],
        "温度_K": radio["noise_temperature_k"],
        "雑音帯域_Hz": radio["noise_bandwidth_hz"],
        "熱雑音_dBm": thermal,
        "受信系等価NF_dB": radio["receiver_noise_figure_db"],
        "アンテナ端換算雑音_dBm": noise,
        "TJX端換算雑音_dBm": noise + gain,
        "SNR_dB": antenna - noise,
    }


def requirements(row, required_snr, margin):
    # 必要SNRには復調器の実装劣化を含める。余裕は一度だけ加算する。
    snr_shortfall = required_snr + margin - row["SNR_dB"]
    power_shortfall = margin - row["電力閾値との差_dB"]
    return {
        "条件": "仮定例・実機要求未確定",
        "方向": row["方向"],
        "距離_m": row["距離_m"],
        "仮定した必要SNR_dB": required_snr,
        "仮定した設計余裕_dB": margin,
        "設計余裕控除後SNR余剰_dB": -snr_shortfall,
        "設計余裕控除後電力余剰_dB": -power_shortfall,
        "必要送信出力_dBm": row["送信出力_dBm"] + max(snr_shortfall, power_shortfall),
        "送信出力の必要増分_dB": max(0.0, snr_shortfall, power_shortfall),
        "許容受信系NF上限_dB": row["アンテナ端信号_dBm"] - row["熱雑音_dBm"] - required_snr - margin,
        "電力閾値を満たす受信系最小利得_dB": row["TJX入力閾値_dBm"] + margin - row["アンテナ端信号_dBm"],
    }


def write_csv(path, rows):
    with path.open("x", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        for row in rows:
            writer.writerow({key: round(value, 9) if isinstance(value, float) else value
                             for key, value in row.items()})


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=root / "src/comms_sim_pkg/config/drone_params.yaml")
    parser.add_argument("--output-dir", type=Path, required=True, help="新規ディレクトリ")
    parser.add_argument("--distances", type=float, nargs="+", default=[4, 8, 12, 16])
    parser.add_argument("--example-required-snr-db", type=float, default=10.0)
    parser.add_argument("--example-design-margin-db", type=float, default=6.0)
    args = parser.parse_args()
    if any(not math.isfinite(d) or d < 1 for d in args.distances):
        parser.error("この正対モデルでは距離を1 m以上の有限値で指定してください")
    if not math.isfinite(args.example_required_snr_db):
        parser.error("必要SNRは有限値で指定してください")
    if not math.isfinite(args.example_design_margin_db) or args.example_design_margin_db < 0:
        parser.error("設計余裕は0以上の有限値で指定してください")
    source = args.config.read_bytes()
    config = yaml.safe_load(source)
    rows = []
    for key, direction in [("radio_ab", "A(PRDEV)→B(PRC)"), ("radio_ba", "B(PRC)→A(PRDEV)")]:
        radio = config[key]
        if any(name not in radio for name in ["tx_antenna_gain_dbi", "rx_antenna_gain_dbi"]):
            parser.error("アンテナ利得を明示したYAMLが必要です。CSV指向性の自動合成には未対応です")
        for distance in args.distances:
            rows.append({"方向": direction, **budget(radio, distance, config["propagation"]["exponent"])})
    examples = [requirements(row, args.example_required_snr_db, args.example_design_margin_db) for row in rows]
    args.output_dir.mkdir(parents=True, exist_ok=False)
    write_csv(args.output_dir / "link_budget.csv", rows)
    write_csv(args.output_dir / "circuit_requirements_example.csv", examples)
    (args.output_dir / "source.txt").write_text(
        f"入力設定: {args.config.resolve()}\nSHA256: {hashlib.sha256(source).hexdigest()}\n"
        "正対時の静的計算。方向名は本プロジェクトのA=PRDEV、B=PRCに対応。\n"
        "送信出力・感度は全距離で固定。大気吸収、偏波ずれ、指向誤差、遮蔽等は含まない。\n"
        "SNRは既存実装に合わせkTB+NFで計算。NFは受信系全体のアンテナ端換算値。\n"
        f"逆算例のみ: 必要SNR={args.example_required_snr_db:g} dB、設計余裕={args.example_design_margin_db:g} dB。\n"
        "逆算例は実機仕様・目標BER/PERを保証しない。\n"
        "必要送信出力はNF・利得を固定し電力閾値とSNRの両方から算出する。\n"
        "NF上限と最小利得は、送信出力など他の値を現状に固定した個別の制約。\n"
        "負のNF上限はこの熱雑音モデルではNF改善だけで目標を満たせないことを示す。\n",
        encoding="utf-8",
    )
    print(f"リンクバジェット{len(rows)}行と回路要求の仮定例を作成: {args.output_dir}")


if __name__ == "__main__":
    main()
