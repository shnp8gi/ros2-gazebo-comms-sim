import numpy as np

# Coordinates
bs_pos = np.array([-5.0, 3.0, 1.0])
ugv_pos = np.array([-3.39, 0.0, 0.0])
mid_offset = np.array([0.0, 1.5, 1.0])

# Vehicle orientation
vehicle_yaw = 0.0
# Antenna relative RPY (at 80 degrees sweep)
world_yaw = np.radians(80.0) - np.pi
entity_yaw = np.radians(-90.0)
antenna_yaw = world_yaw - entity_yaw

print(f"ugv_antenna_yaw (deg): {np.degrees(antenna_yaw)}")

# Compute UGV antenna position
R_veh = np.array([
    [np.cos(vehicle_yaw), -np.sin(vehicle_yaw), 0],
    [np.sin(vehicle_yaw), np.cos(vehicle_yaw), 0],
    [0, 0, 1]
])
ant_pos = ugv_pos + R_veh.dot(mid_offset)
print(f"Antenna position: {ant_pos}")

# Vector to BS
v_world = bs_pos - ant_pos
norm = np.linalg.norm(v_world)
v_world = v_world / norm
print(f"v_world: {v_world}")

# UGV antenna rotation matrix
ant_rpy = np.array([0.0, 0.0, antenna_yaw])
cos_y, sin_y = np.cos(antenna_yaw), np.sin(antenna_yaw)
R_ant = np.array([
    [cos_y, -sin_y, 0],
    [sin_y, cos_y, 0],
    [0, 0, 1]
])

# Project vector onto UGV antenna frame
v_ant = R_ant.T.dot(v_world)
print(f"v_ant: {v_ant}")

az = np.degrees(np.arctan2(v_ant[1], v_ant[0]))
el = np.degrees(np.arcsin(v_ant[2]))
print(f"Azimuth (deg): {az}")
print(f"Elevation (deg): {el}")
