import os
import re
import glob
import subprocess

sweep_dir = "sim_results/sweep_20260615_151121"
runs_dir = os.path.join(sweep_dir, "runs")

empty_runs = []
pattern = re.compile(r"run_(\d+)_y([-\d\.]+)_a([-\d\.]+)")

for run_folder in sorted(os.listdir(runs_dir)):
    full_path = os.path.join(runs_dir, run_folder)
    if not os.path.isdir(full_path):
        continue
    comms_dir = os.path.join(full_path, "comms")
    if not os.path.exists(comms_dir) or len(os.listdir(comms_dir)) == 0:
        match = pattern.match(run_folder)
        if match:
            run_idx = int(match.group(1))
            y = float(match.group(2))
            angle = float(match.group(3))
            empty_runs.append((run_folder, run_idx, y, angle))

print(f"Found {len(empty_runs)} empty runs.")
if not empty_runs:
    exit()

# Pick the first empty run
run_folder, run_idx, y, angle = empty_runs[0]
print(f"Testing the first empty run: {run_folder} (run_idx={run_idx}, y={y}, angle={angle})")

# Re-run this task using docker-compose exec with worker_id = test_failed
ros_domain_id = 99
gz_partition = "comms_sim_partition_test_failed"
gz_port = 11399
rtf = 5.0
base_station_yaw_deg = -90.0

# Generate the config YAML
import math
world_yaw = math.radians(angle) - math.pi
entity_yaw = math.radians(base_station_yaw_deg)
antenna_yaw = world_yaw - entity_yaw

CONFIG_PATH = "src/comms_sim_pkg/config/sim_params.yaml"
tmp_config_path = "tools/sweep_build/sim_params_tmp_test_failed.yaml"

with open(CONFIG_PATH, 'r', encoding='utf-8') as f:
    content = f.read()

content = re.sub(r'\n\s*summary_filename:\s*["\']?[^"\']*["\']?', '', content)
content = re.sub(r'\n\s*output_subdir:\s*["\']?[^"\']*["\']?', '', content)
content = re.sub(
    r'(simulation:)', 
    rf'\1\n  summary_filename: "sweep_summary_test_failed_run{run_idx}_wtest_failed.csv"\n  output_subdir: "sweep_test_failed"', 
    content
)

content = re.sub(r'headless:\s*false', 'headless: true', content)
content = re.sub(r'real_time_factor:\s*[\d\.]+', f'real_time_factor: {rtf}', content)

content = re.sub(
    r'(antenna\w*:\s*.*?pose:\s*\[\s*)([-\d\.]+),\s*[-\d\.]+,\s*([-\d\.]+),\s*([-\d\.]+),\s*([-\d\.]+),\s*([-\d\.]+)(\s*\])',
    rf'\g<1>\g<2>, {y}, \g<3>, \g<4>, \g<5>, {entity_yaw:.4f}\g<7>',
    content,
    flags=re.DOTALL
)

content = re.sub(
    r'(antenna_relative_rpy:\s*\[\s*[-\d\.]+,\s*[-\d\.]+,\s*)[-\d\.]+(\s*\])',
    rf'\g<1>{antenna_yaw:.4f}\g<2>',
    content
)

ugv_antenna_yaw = world_yaw - math.pi
for ant_name in ["shinkansen_front", "shinkansen_mid", "shinkansen_rear"]:
    content = re.sub(
        rf'(name:\s*"{ant_name}".*?relative_rpy:\s*\[\s*[-\d\.]+,\s*[-\d\.]+,\s*)[-\d\.]+(\s*\])',
        rf'\g<1>{ugv_antenna_yaw:.4f}\g<2>',
        content,
        flags=re.DOTALL
    )

os.makedirs(os.path.dirname(tmp_config_path), exist_ok=True)
with open(tmp_config_path, 'w', encoding='utf-8') as f:
    f.write(content)

# Run the command and capture/stream output
cmd = [
    "docker", "compose", "exec", "sim", "bash", "-c", 
    f"export ROS_DOMAIN_ID={ros_domain_id} && "
    f"export GZ_PARTITION={gz_partition} && "
    f"export GZ_PORT={gz_port} && "
    f"export FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/src/comms_sim_pkg/config/fastdds_no_shm.xml && "
    f"source /opt/ros/humble/setup.bash && source install/setup.bash && "
    f"ros2 launch comms_sim_pkg sim_launch.py config_file:=/workspace/{tmp_config_path}"
]

print("Running command...")
subprocess.run(cmd)
