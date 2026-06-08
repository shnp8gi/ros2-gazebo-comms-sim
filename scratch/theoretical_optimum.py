import csv
import math
import numpy as np

# Load H-plane pattern
h_angles = []
h_gains = []
h_plane_path = "src/comms_sim_pkg/config/h_plane.csv"

with open(h_plane_path, 'r', encoding='utf-8') as f:
    for line in f:
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        if 'angle' in line.lower():
            continue
        parts = line.split(',')
        if len(parts) >= 2:
            try:
                angle = float(parts[0].strip())
                gain = float(parts[1].strip().replace('−', '-'))
                h_angles.append(angle)
                h_gains.append(gain)
            except ValueError:
                continue

h_angles = np.array(h_angles)
h_gains = np.array(h_gains)

# Sort
idx = np.argsort(h_angles)
h_angles = h_angles[idx]
h_gains = h_gains[idx]

def get_h_gain(angle_deg):
    # Clamp angle to [-180, 180]
    while angle_deg > 180:
        angle_deg -= 360
    while angle_deg < -180:
        angle_deg += 360
    # Pattern is only defined from -90 to 90
    if angle_deg < -90 or angle_deg > 90:
        return -7.5  # peak (22.5) - max_atten (30)
    return float(np.interp(angle_deg, h_angles, h_gains))

def get_throughput(rssi):
    # MCS table
    # -50.5: 6.0
    # -54.0: 4.6
    # -58.5: 2.7
    # -62.5: 2.15
    # -65.5: 1.1
    # -68.5: 0.5
    if rssi >= -50.5:
        return 6.0
    elif rssi >= -54.0:
        return 4.6
    elif rssi >= -58.5:
        return 2.7
    elif rssi >= -62.5:
        return 2.15
    elif rssi >= -65.5:
        return 1.1
    elif rssi >= -68.5:
        return 0.5
    else:
        return 0.0

def calculate_total_data(angle_deg, y_dist=1.5):
    # Integrate along track X from -100 to 100 meters
    dx = 0.01
    x_range = np.arange(-100, 100, dx)
    total_data_bits = 0.0
    
    world_yaw = math.radians(angle_deg) - math.pi
    
    for x in x_range:
        # Vector from BS (0, y_dist) to Train (x, 0)
        # target_pos - antenna_pos = (x, -y_dist)
        d = math.sqrt(x**2 + y_dist**2)
        
        # angle in world frame
        phi = math.atan2(-y_dist, x)
        
        # relative angle in antenna frame
        rel_angle_rad = phi - world_yaw
        # wrap to [-pi, pi]
        rel_angle_rad = (rel_angle_rad + math.pi) % (2.0 * math.pi) - math.pi
        rel_angle_deg = math.degrees(rel_angle_rad)
        
        # Total gain (both antennas are symmetric)
        g_bs = get_h_gain(rel_angle_deg)
        g_tx = get_h_gain(rel_angle_deg) # since train antenna is aligned at the same angle
        g_total = g_bs + g_tx
        
        # Path loss
        pl = 68.0 + 20 * math.log10(d)
        
        # RSSI
        rssi = -7.0 - pl + g_total
        
        # Throughput (Gbps)
        tp = get_throughput(rssi)
        
        # Data in megabytes: tp * 1e9 bits/s * dt_s / 8 / 1e6
        # dx is distance step. dt = dx / v
        # v = 83.33 m/s
        dt = dx / 83.33
        total_data_bits += tp * 1e9 * dt
        
    return total_data_bits / 8.0 / 1e6 # MB

# Evaluate angles from 10 to 170 with step 1
results = []
for ang in range(10, 171):
    data = calculate_total_data(ang)
    results.append((ang, data))

# Sort to find optimal
results.sort(key=lambda x: x[1], reverse=True)
print("Top 10 optimal angles (Angle, Data MB):")
for r in results[:10]:
    print(f"  {r[0]} deg: {r[1]:.4f} MB")
