#!/usr/bin/env python3
"""TJX理論モデルを独立seedで実行し、平均・母分散・不偏分散を保存する。"""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import csv
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
import statistics
import subprocess
import yaml


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=root / "src/comms_sim_pkg/config/scenarios/drone_dual60_cross_fixed_theoretical.yaml")
    parser.add_argument("--binary", type=Path, default=root / "build/tjx_theory/tjx_theory_run")
    parser.add_argument("--output", type=Path, default=root / f"sim_results/tjx_theory_{datetime.now():%Y%m%d_%H%M%S}")
    parser.add_argument("--trials", type=int, default=30)
    parser.add_argument("--jobs", type=int, default=4)
    parser.add_argument("--seed", type=int, default=20261102)
    args = parser.parse_args()
    if args.trials < 2 or args.jobs < 1 or args.seed < 0 or args.seed + args.trials >= 2**64:
        parser.error("試行数は2以上、並列数は1以上、seedはuint64範囲で指定してください")
    config_path, binary, output = args.config.resolve(), args.binary.resolve(), args.output.resolve()
    if not binary.is_file():
        parser.error("実行ファイルがありません。先にtools/tjx_theoryをビルドしてください")
    config = yaml.safe_load(config_path.read_text())
    if not config.get("theoretical_phy", {}).get("enabled", False):
        parser.error("theoretical_phy.enabledがtrueの設定が必要です")
    output.mkdir(parents=True, exist_ok=False)
    inputs = output / "inputs"
    inputs.mkdir()
    provenance = {}
    for mapping, key, filename in ((config["antenna"], "e_plane", "e_plane.csv"),
                                   (config["antenna"], "h_plane", "h_plane.csv"),
                                   (config["theoretical_phy"], "table", "tjx_theoretical_lut.yaml")):
        source = (config_path.parent / mapping[key]).resolve()
        target = inputs / filename
        shutil.copyfile(source, target)
        provenance[filename] = {"source": str(source), "sha256": hashlib.sha256(target.read_bytes()).hexdigest()}
        mapping[key] = str(target)
    config["hrcp"]["monitor_enabled"] = False
    saved = output / "input_config.yaml"
    saved.write_text(yaml.safe_dump(config, allow_unicode=True, sort_keys=False))
    provenance["binary"] = {"source": str(binary), "sha256": hashlib.sha256(binary.read_bytes()).hexdigest()}
    provenance["git_head"] = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip()
    provenance["git_branch"] = subprocess.check_output(["git", "branch", "--show-current"], cwd=root, text=True).strip()
    (output / "tracked_changes.patch").write_text(subprocess.check_output(["git", "diff"], cwd=root, text=True))
    # 新規ファイルも含む実装のハッシュとコピー。未コミット状態でも再検証できるようにする。
    sources = list((root / "src/comms_sim_pkg/src").glob("*.cpp")) + list((root / "src/comms_sim_pkg/include/comms_sim_pkg").glob("*.hpp"))
    sources += [Path(__file__).resolve(), root / "tools/tjx_theory/CMakeLists.txt"]
    for source in sources:
        relative = source.relative_to(root)
        target = output / "source_snapshot" / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)
        provenance[str(relative)] = {"sha256": hashlib.sha256(source.read_bytes()).hexdigest()}
    provenance["trials"] = args.trials
    provenance["lut_trials_per_point"] = yaml.safe_load((inputs / "tjx_theoretical_lut.yaml").read_text())["trials_per_point"]
    provenance["execution"] = "native_shared_geometry_RadioModel_HrcpPairnet; no_Gazebo_GUI"
    provenance["variance_scope"] = "independent_packet_and_MAC_seeds_given_one_fixed_LUT; excludes_LUT_estimation_uncertainty"
    (output / "provenance.json").write_text(json.dumps(provenance, ensure_ascii=False, indent=2))

    def run(index):
        trial = output / f"trial_{index:02d}"
        command = [str(binary), str(saved), str(trial), str(args.seed + index)]
        with (output / f"trial_{index:02d}.log").open("w") as log:
            subprocess.run(command, cwd=root, stdout=log, stderr=subprocess.STDOUT, check=True)
        summary = yaml.safe_load((trial / "summary.yaml").read_text())
        return {"trial": index, **summary}

    rows = []
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        futures = [pool.submit(run, i) for i in range(args.trials)]
        for future in as_completed(futures):
            row = future.result()
            rows.append(row)
            print(f"完了 {len(rows)}/{args.trials}: trial={row['trial']} {row['total_data_MiB']:.3f} MiB", flush=True)
    rows.sort(key=lambda row: row["trial"])
    with (output / "trials.csv").open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    aggregate = {"trials": args.trials, "model": "tjx_ldpc_awgn_v1", "data_direction": rows[0]["data_direction"],
                 "transfer_bin_width_s": rows[0]["transfer_bin_width_s"], "statistics": {}}
    aggregate["counter_definitions"] = {
        "timeouts": "接続確立前の端点ごとの待機期限満了。Beacon探索中・フレーム単体失敗・再送は含まない。",
        "connections": "両端の接続が成立した回数。",
        "disconnections": "接続成立後にATP満了で接続が失われた回数。timeoutsとは別集計。"}
    for metric in ("total_data_MiB", "connected_time_s", "active_transfer_bins_s", "transfer_bin_span_s", "mean_goodput_Mbps",
                   "timeouts", "connections", "disconnections"):
        values = [r[metric] for r in rows]
        aggregate["statistics"][metric] = {
            "mean": statistics.mean(values), "population_variance": statistics.pvariance(values),
            "sample_variance": statistics.variance(values), "population_stddev": statistics.pstdev(values),
            "min": min(values), "max": max(values)}
    (output / "aggregate.json").write_text(json.dumps(aggregate, ensure_ascii=False, indent=2))
    (output / "reporting_notes.json").write_text(json.dumps({
        "internal_log_only": ["model_assumptions", "implementation_details", "LUT_finite_trial_uncertainty", "seeds_and_execution_environment"],
        "materials_include": ["antenna_gain_calculation_details", "essential_experiment_conditions", "model_identity", "units_and_metric_definitions", "30_trial_mean_and_variance"],
        "source": "ユーザーの既存方針: 詳細項目6（アンテナ利得）以外は内部ログに残す"}, ensure_ascii=False, indent=2))
    print(json.dumps(aggregate, ensure_ascii=False, indent=2))
    print(f"結果保存先: {output}")


if __name__ == "__main__":
    main()
