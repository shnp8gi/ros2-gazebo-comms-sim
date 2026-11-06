"""2機のドローン用設定・worldを保存してGazeboを起動する。"""
import copy
import hashlib
import json
import shutil
import os
from pathlib import Path
from datetime import datetime
import uuid
import tempfile
import xml.etree.ElementTree as ET
import yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def resolve_radio_models(config):
    """C++ローダーと同じ互換判定で、保存設定に受信電力の参照面を明記する。"""
    references = {}
    for side in ("ab", "ba"):
        radio = config[f"radio_{side}"]
        legacy = any(key in radio for key in
                     ("rx_lna_gain_db", "rx_conversion_gain_db", "rx_insertion_loss_db"))
        model = radio.setdefault("receiver_model", "rf_chain" if legacy else "equivalent_nf")
        if model not in ("equivalent_nf", "rf_chain"):
            raise ValueError("receiver_model は equivalent_nf または rf_chain です")
        references[side] = ("rx_antenna_terminal" if model == "equivalent_nf"
                            else "receiver_chain_output_equivalent")
    models = {config[f"radio_{side}"]["receiver_model"] for side in ("ab", "ba")}
    config["power_model_revision"] = ("antenna_equivalent_nf_v1" if models == {"equivalent_nf"}
                                      else "rf_chain_thermal_v1" if models == {"rf_chain"}
                                      else "mixed_receiver_models_v1")
    config["rssi_reference_by_direction"] = references
    return next(iter(references.values())) if len(set(references.values())) == 1 else "per_direction"


def setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)
    share = Path(get_package_share_directory("comms_sim_pkg"))
    config_path = Path(arg("config_file")).resolve()
    with config_path.open() as stream:
        config = yaml.safe_load(stream)
    for key in ("scenario", "antenna_mode"):
        if arg(key):
            config[key] = arg(key)
    for key, target in (("speed_a_kmh", "drone_a"), ("speed_b_kmh", "drone_b")):
        if arg(key):
            config[target]["speed_kmh"] = float(arg(key))
    if config["scenario"] not in ("parallel", "cross") or config["antenna_mode"] not in ("fixed", "tracking"):
        raise ValueError("scenario / antenna_mode が不正です")
    for key in ("e_plane", "h_plane"):
        config["antenna"][key] = str((config_path.parent / config["antenna"][key]).resolve())
    theory=config.get("theoretical_phy", {})
    if theory.get("enabled",False):
        theory["table"]=str((config_path.parent / theory["table"]).resolve())
    else:
        config["mcs_table"] = str((config_path.parent / config["mcs_table"]).resolve())
    empirical=config.get("empirical_phy", {})
    if empirical.get("enabled",False):
        empirical["table"]=str((config_path.parent / empirical["table"]).resolve())
    output = Path(arg("output_dir") or f"sim_results/drone_{datetime.now():%Y%m%d_%H%M%S}_{uuid.uuid4().hex[:6]}").resolve()
    # 存在する結果・設定を上書きしない。空のディレクトリだけ再利用できる。
    if output.exists() and any(output.iterdir()):
        raise ValueError(f"出力先が空ではありません: {output}")
    output.mkdir(parents=True, exist_ok=True)
    config["output_dir"] = str(output)
    config["geometry_revision"] = "both_scenarios_sideways_fixed_v3"
    config["timeout_counting_revision"] = "setup_only_v2"
    config["protocol_revision"] = "indexed_beacon_until_request_half_duplex_v1"
    config["endpoint_roles"] = {"A":"PRDEV", "B":"PRC"}
    config["data_direction"] = "B_TO_A" if config["protocol"].get("data_from_prc",False) else "A_TO_B"
    rssi_reference = resolve_radio_models(config)
    config["phy_error_model"] = ("uncoded_gray_qpsk_awgn_v1" if config.get("awgn", {}).get("enabled", False)
                                 else "threshold_only")
    if config.get("lab_phy", {}).get("enabled", False):
        config["phy_error_model"] = "lab_iq_rect_v0_uncoded"
        config["protocol_revision"] = "indexed_beacon_lab_mac80_v0"
        config["phy_limitations"] = ["nonstandard_lab_codec", "payload_fec_none",
            "rectangular_pulse_not_rrc", "frame_window_known_no_blind_scan",
            "no_detection_time_jitter", "propagation_ceil_to_sample"]
    if config.get("hrcp", {}).get("enabled", False):
        config["phy_error_model"] = "hrcp_iq_continuous_ldpc_v1"
        config["protocol_revision"] = "hrcp_independent_endpoints_v1"
        config["phy_limitations"] = ["ideal_rectangular_nyquist_pulse_not_spectrum_mask_verified",
            "no_CFO_phase_noise_multipath_or_doppler",
            "AWGN_continuous_absolute_sample_seed" if config.get("awgn",{}).get("enabled",False) else "AWGN_disabled_no_noise_samples",
            "slot_0_1us_and_RSSI_30_minus_code_provisional_user_approved",
            "subheader_crc8_init0_provisional_user_approved", "no_external_interoperability_vector"]
        config["standard_exceptions"] = ["directional_RF_split", "HRCP_frame_at_271_08GHz", "beacon_140us"]
        config["hrcp"].setdefault("expected_rssi_admission_enabled", True)
        if not config["hrcp"]["expected_rssi_admission_enabled"]:
            config["standard_exceptions"].append("expected_RSSI_admission_disabled_to_match_device")
        receiver = 'A' if config["protocol"].get("data_from_prc",False) else 'B'
        config["payload_accounting"] = f"unique_CRC_valid_payload_received_at_{receiver}; acked_data_frames_separate"
        config["hrcp"]["timing_sample_rate_hz"] = 3.52e9
        config["hrcp"]["sifs_us"] = 2.5
        config["hrcp"]["rifs_a_us"] = 19.05
        config["hrcp"]["rifs_b_us"] = 8.01
        config["hrcp"].setdefault("monitor_rssi_window_s",1e-6)
        config["monitor_rssi_measurement"] = {"source":"rx_iq_matched_filter",
            "reference":rssi_reference, "includes_noise":config.get("awgn",{}).get("enabled",False),
            "equivalent_noise_bandwidth_hz":1.76e9, "self_tx":"excluded_missing_if_no_rx_samples"}
    config["legacy_rssi_csv_source"] = "timeseries/events/frames RSSI columns: link_budget_predicted_signal; waveform RSSI: live_signals_rssi.csv"
    if config.get("preview_mode", False):
        config["evaluation_scope"] = "usability_preview_not_standard_phy_validation"
        config["phy_limitations"] = ["statistical_uncoded_QPSK_no_waveform_decode",
            "simplified_MAC_20us_control_10us_IFS", "expected_RSSI_admission_not_evaluated",
            "fixed_RSSI_gate_effectively_disabled_at_minus999dBm"]
    if config.get("preview_mode", False) and config["phy_error_model"] == "threshold_only":
        config["evaluation_scope"] = "received_power_threshold_with_mcs_table"
        config["phy_limitations"] = ["deterministic_received_power_threshold_no_waveform_decode",
            "simplified_MAC_20us_control_10us_IFS", "expected_RSSI_admission_not_evaluated"]
        config["rate_source"] = "MCStable.csv; DATA direction; minimum sampled rate during frame; credited after ACK"
    if empirical.get("enabled",False):
        config["phy_error_model"]="empirical_packet_v1"
        config["evaluation_scope"]="empirical_probability_with_shared_hrcp_mac"
        config["phy_limitations"]=["packet_all_or_nothing_header_and_body",
            "RSSI_held_at_frame_start", "independent_frame_draws", "no_waveform_or_FEC_execution",
            "legacy_control_frame_timing_not_TJX_BPSK_validated",
            "BER_independent_residual_error_approximation" if empirical["metric"]=="ber" else
            "PER_length_policy_"+empirical.get("per_length_policy","exact")]
        config["rate_source"]="MCStable.csv at DATA TX start; mcs_by_rate_gbps maps to measured MCS IDs"
        config["monitor_rssi_measurement"]={"source":"link_budget_predicted_signal",
            "reference":rssi_reference, "includes_noise":False}
        config["calibration_status"]=empirical.get("calibration_status","unvalidated")
        config["reachability_columns"]="timeseries ab_reachable/ba_reachable are legacy threshold references; empirical decisions use the curve only"
        config["frame_rssi_reference"]="frames.csv rssi_at_end_dbm holds TX-start signal power for empirical_packet_v1"
    if theory.get("enabled",False):
        config["phy_error_model"]="tjx_ldpc_awgn_v1"
        config["evaluation_scope"]="simulation_generated_LDPC_LUT_with_shared_hrcp_mac"
        config["phy_limitations"]=["ideal_synchronization_and_known_flat_channel",
            "TJX_BPSK_uses_HRCP_LDPC_and_common_EH_header_assumption",
            "SNR_held_at_frame_start", "independent_codewords_and_frames",
            "finite_Monte_Carlo_counts_no_rare_error_guarantee",
            "no_CFO_phase_noise_multipath_or_doppler"]
        config["rate_source"]="TJX modulation/coding/repetition at 1.76Gchip/s; exact shortening/block padding/pilot timing"
        config["mcs_selection"]="ACK_feedback_10_successes_up_1_failure_down; TJX_0_3_6_8_9_11"
        config["monitor_rssi_measurement"]={"source":"link_budget_predicted_signal",
            "reference":rssi_reference, "includes_noise":False}
        config["reachability_columns"]="legacy_threshold_reference_only_not_used_by_theoretical_PHY"
        config["frame_rssi_reference"]="TX_start_signal_power; minimum_snr_db also holds TX_start_SNR"
        config["calibration_status"]="simulation_only_no_measured_receiver_data"
    runtime = tempfile.TemporaryDirectory(prefix='drone_monitor_') if config.get('monitor_only',False) else None
    if runtime is None:
        inputs = output / "inputs"
        inputs.mkdir()
        provenance = {}
        for label, mapping, key in (("e_plane.csv", config["antenna"], "e_plane"),
                                    ("h_plane.csv", config["antenna"], "h_plane"),
                                    ("MCStable.csv", config, "mcs_table")):
            if key=="mcs_table" and theory.get("enabled",False):
                continue
            source = Path(mapping[key])
            target = inputs / label
            shutil.copyfile(source, target)
            provenance[label] = {"source": str(source), "sha256": hashlib.sha256(target.read_bytes()).hexdigest()}
            mapping[key] = str(target)
        if empirical.get("enabled",False):
            source=Path(empirical["table"]);target=inputs/"receiver_curve.csv"
            shutil.copyfile(source,target)
            provenance["receiver_curve.csv"]={"source":str(source),"sha256":hashlib.sha256(target.read_bytes()).hexdigest()}
            metadata=source.with_suffix(source.suffix+".provenance.json")
            if metadata.exists():
                saved=inputs/"receiver_curve.provenance.json";shutil.copyfile(metadata,saved)
                provenance["receiver_curve.provenance.json"]={"source":str(metadata),"sha256":hashlib.sha256(saved.read_bytes()).hexdigest()}
            empirical["table"]=str(target)
        if theory.get("enabled",False):
            source=Path(theory["table"]);target=inputs/"tjx_theoretical_lut.yaml"
            shutil.copyfile(source,target)
            provenance[target.name]={"source":str(source),"sha256":hashlib.sha256(target.read_bytes()).hexdigest()}
            theory["table"]=str(target)
        provenance["drone_model"] = json.loads((share / "models/drone/fuel_metadata.json").read_text())
        (output / "provenance.json").write_text(json.dumps(provenance, indent=2))
    resolved = output / "config.yaml"
    resolved.write_text(yaml.safe_dump(config, allow_unicode=True, sort_keys=False))
    tree = ET.parse(share / "worlds/drone_world.sdf")
    world = tree.getroot().find("world")
    world.find("physics/max_step_size").text = str(config["simulation"]["physics_step_s"])
    world.find("physics/real_time_factor").text = str(config["simulation"]["real_time_factor"])
    for name in ("drone_a", "drone_b", "antenna_a", "antenna_b"):
        asset = "drone" if name.startswith("drone") else "antenna"
        model = copy.deepcopy(ET.parse(share / f"models/{asset}/model.sdf").getroot().find("model"))
        model.set("name", name)
        # Fuelから保存済みのローカルメッシュだけを使い、実行時の取得を避ける。
        for uri in model.findall(".//mesh/uri"):
            if uri.text.startswith("model://drone/"):
                uri.text = str(share / "models/drone" / uri.text.removeprefix("model://drone/"))
        world.append(model)
    plugin_path = Path(get_package_prefix("comms_sim_pkg")) / "lib/comms_sim_pkg/plugins/DroneEncounterPlugin.so"
    plugin = ET.SubElement(world, "plugin", filename=str(plugin_path), name="comms_sim::DroneEncounterPlugin")
    ET.SubElement(plugin, "config_file").text = str(resolved)
    ET.SubElement(plugin, "pause_at_encounter").text = arg("pause_at_encounter").lower()
    world_path = (Path(runtime.name) if runtime else output) / "world.sdf"
    tree.write(world_path, encoding="unicode")
    env = {"GZ_PARTITION": os.environ.get("GZ_PARTITION") or f"drone_{os.getpid()}_{uuid.uuid4().hex[:6]}"}
    server = ExecuteProcess(cmd=["gz", "sim", "-s", "-r", str(world_path)], additional_env=env, output="screen")

    def finished(event, _context):
        if _context.is_shutdown:
            return []
        if event.returncode != 0 or not (output / "completed").is_file():
            raise RuntimeError(f"実験が完了しませんでした。ログと設定: {output}")
        if runtime: runtime.cleanup()
        print(f"結果保存先: {output}", flush=True)
        if arg("headless").lower() != "true" and arg("keep_gui_open").lower() == "true":
            print("実験完了。GUIは確認用に開いたままにします。", flush=True)
            return []
        return [EmitEvent(event=Shutdown(reason="ドローン実験完了"))]

    actions = [RegisterEventHandler(OnProcessExit(target_action=server, on_exit=finished)), server]
    if arg("bridge").lower() == "true":
        actions.append(Node(package="ros_gz_bridge", executable="parameter_bridge", arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/drone_a/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose",
            "/drone_b/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose"],
            additional_env=env, output="screen"))
    if arg("headless").lower() != "true":
        gui_path = output / "gui.config"
        gui_path.write_text('<window><width>1280</width><height>960</height></window>\n' +
                            "\n".join(ET.tostring(p, encoding="unicode") for p in world.find("gui")))
        gui = ExecuteProcess(cmd=["gz", "sim", "-g", "--gui-config", str(gui_path)], additional_env=env, output="screen")
        actions += [RegisterEventHandler(OnProcessExit(target_action=gui,
                    on_exit=[EmitEvent(event=Shutdown(reason="GUIを閉じました"))])), gui]
    return actions


def generate_launch_description():
    share = get_package_share_directory("comms_sim_pkg")
    defaults = {"config_file": share + "/config/drone_params.yaml", "scenario": "", "antenna_mode": "",
                "speed_a_kmh": "", "speed_b_kmh": "", "output_dir": "", "headless": "true", "bridge": "true", "pause_at_encounter": "false", "keep_gui_open": "false"}
    return LaunchDescription([DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()]
                             + [OpaqueFunction(function=setup)])
