import os
import re
import math
import subprocess

CONFIG_PATH = "src/comms_sim_pkg/config/sim_params.yaml"
tmp_config_path = "tools/sweep_build/sim_params_tmp_test.yaml"
y = 3.0
angle_deg = 10.0
base_station_yaw_deg = -90.0
rtf = 5.0

world_yaw = math.radians(angle_deg) - math.pi
entity_yaw = math.radians(base_station_yaw_deg)
antenna_yaw = world_yaw - entity_yaw

with open(CONFIG_PATH, 'r', encoding='utf-8') as f:
    content = f.read()

content = re.sub(r'\n\s*summary_filename:\s*["\']?[^"\']*["\']?', '', content)
content = re.sub(r'\n\s*output_subdir:\s*["\']?[^"\']*["\']?', '', content)
content = re.sub(
    r'(simulation:)', 
    r'\1\n  summary_filename: "sweep_summary_test_run1_wtest.csv"\n  output_subdir: "sweep_test"', 
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

print("Temp YAML created at:", tmp_config_path)

ros_domain_id = 99
gz_partition = "comms_sim_partition_test"
gz_port = 11399

cmd = [
    "docker", "compose", "exec", "sim", "bash", "-c", 
    f"export ROS_DOMAIN_ID={ros_domain_id} && "
    f"export GZ_PARTITION={gz_partition} && "
    f"export GZ_PORT={gz_port} && "
    f"export FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/src/comms_sim_pkg/config/fastdds_no_shm.xml && "
    f"source /opt/ros/humble/setup.bash && source install/setup.bash && "
    f"ros2 launch comms_sim_pkg sim_launch.py config_file:=/workspace/{tmp_config_path}"
]

print("Running command:", " ".join(cmd))
subprocess.run(cmd)
