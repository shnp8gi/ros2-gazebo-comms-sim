"""2機のドローン用設定・worldを保存してGazeboを起動する。"""
import copy
import hashlib
import json
import shutil
import os
from pathlib import Path
from datetime import datetime
import uuid
import xml.etree.ElementTree as ET
import yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
    config["mcs_table"] = str((config_path.parent / config["mcs_table"]).resolve())
    output = Path(arg("output_dir") or f"sim_results/drone_{datetime.now():%Y%m%d_%H%M%S}_{uuid.uuid4().hex[:6]}").resolve()
    # 存在する結果・設定を上書きしない。空のディレクトリだけ再利用できる。
    if output.exists() and any(output.iterdir()):
        raise ValueError(f"出力先が空ではありません: {output}")
    output.mkdir(parents=True, exist_ok=True)
    config["output_dir"] = str(output)
    config["geometry_revision"] = "both_scenarios_sideways_fixed_v3"
    config["timeout_counting_revision"] = "setup_only_v2"
    config["protocol_revision"] = "indexed_beacon_until_request_half_duplex_v1"
    chain_fields = ("tx_antenna_gain_dbi", "rx_antenna_gain_dbi", "tx_insertion_loss_db",
                    "rx_insertion_loss_db", "rx_lna_gain_db", "rx_conversion_gain_db", "noise_bandwidth_hz")
    config["power_model_revision"] = ("rf_chain_thermal_v1" if any(
        field in config[f"radio_{side}"] for side in ("ab", "ba") for field in chain_fields)
        else "legacy_antenna_only")
    config["phy_error_model"] = ("uncoded_gray_qpsk_awgn_v1" if config.get("awgn", {}).get("enabled", False)
                                 else "threshold_only")
    if config.get("lab_phy", {}).get("enabled", False):
        config["phy_error_model"] = "lab_iq_rect_v0_uncoded"
        config["protocol_revision"] = "indexed_beacon_lab_mac80_v0"
        config["phy_limitations"] = ["nonstandard_lab_codec", "payload_fec_none",
            "rectangular_pulse_not_rrc", "frame_window_known_no_blind_scan",
            "no_detection_time_jitter", "propagation_ceil_to_sample"]
    inputs = output / "inputs"
    inputs.mkdir()
    provenance = {}
    for label, mapping, key in (("e_plane.csv", config["antenna"], "e_plane"),
                                ("h_plane.csv", config["antenna"], "h_plane"),
                                ("MCStable.csv", config, "mcs_table")):
        source = Path(mapping[key])
        target = inputs / label
        shutil.copyfile(source, target)
        provenance[label] = {"source": str(source), "sha256": hashlib.sha256(target.read_bytes()).hexdigest()}
        mapping[key] = str(target)
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
    world_path = output / "world.sdf"
    tree.write(world_path, encoding="unicode")
    env = {"GZ_PARTITION": os.environ.get("GZ_PARTITION") or f"drone_{os.getpid()}_{uuid.uuid4().hex[:6]}"}
    server = ExecuteProcess(cmd=["gz", "sim", "-s", "-r", str(world_path)], additional_env=env, output="screen")

    def finished(event, _context):
        if _context.is_shutdown:
            return []
        if event.returncode != 0 or not (output / "completed").is_file():
            raise RuntimeError(f"実験が完了しませんでした。ログと設定: {output}")
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
