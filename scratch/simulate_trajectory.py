import math
import sys

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

def get_h_gain(angle_deg):
    while angle_deg > 180:
        angle_deg -= 360
    while angle_deg < -180:
        angle_deg += 360
    if angle_deg < -90 or angle_deg > 90:
        return -7.5
    # linear interpolation
    for i in range(len(h_angles) - 1):
        if h_angles[i] <= angle_deg <= h_angles[i+1]:
            t = (angle_deg - h_angles[i]) / (h_angles[i+1] - h_angles[i])
            return h_gains[i] + t * (h_gains[i+1] - h_gains[i])
    closest_idx = min(range(len(h_angles)), key=lambda i: abs(h_angles[i] - angle_deg))
    return h_gains[closest_idx]

# E-plane implementation: we can load it, or just use 0.0 since pitch/elevation is small.
# Let's load it to be exact
e_angles = []
e_gains = []
e_plane_path = "src/comms_sim_pkg/config/e_plane.csv"
try:
    with open(e_plane_path, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            if 'angle' in line.lower():
                continue
            parts = line.split(',')
            if len(parts) >= 2:
                angle = float(parts[0].strip())
                gain = float(parts[1].strip().replace('−', '-'))
                e_angles.append(angle)
                e_gains.append(gain)
except Exception:
    pass

def get_e_gain(angle_deg):
    if not e_angles:
        return 0.0
    while angle_deg > 180:
        angle_deg -= 360
    while angle_deg < -180:
        angle_deg += 360
    if angle_deg < -90 or angle_deg > 90:
        return -7.5
    for i in range(len(e_angles) - 1):
        if e_angles[i] <= angle_deg <= e_angles[i+1]:
            t = (angle_deg - e_angles[i]) / (e_angles[i+1] - e_angles[i])
            return e_gains[i] + t * (e_gains[i+1] - e_gains[i])
    closest_idx = min(range(len(e_angles)), key=lambda i: abs(e_angles[i] - angle_deg))
    return e_gains[closest_idx]

def rpy_to_rotmat(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr           ]
    ]

def transpose(mat):
    return [[mat[j][i] for j in range(len(mat))] for i in range(len(mat[0]))]

def mat_vec_mult(mat, vec):
    return [
        mat[0][0]*vec[0] + mat[0][1]*vec[1] + mat[0][2]*vec[2],
        mat[1][0]*vec[0] + mat[1][1]*vec[1] + mat[1][2]*vec[2],
        mat[2][0]*vec[0] + mat[2][1]*vec[1] + mat[2][2]*vec[2]
    ]

def calculate_antenna_frame_angles(ant_pos, target_pos, ant_rpy):
    # Vector from antenna to target
    v = [target_pos[0] - ant_pos[0], target_pos[1] - ant_pos[1], target_pos[2] - ant_pos[2]]
    norm = math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)
    if norm <= 1e-12:
        return 0.0, 0.0
    v = [x / norm for x in v]
    
    r = rpy_to_rotmat(ant_rpy[0], ant_rpy[1], ant_rpy[2])
    r_T = transpose(r)
    v_ant = mat_vec_mult(r_T, v)
    
    az = math.atan2(v_ant[1], v_ant[0])
    el = math.asin(max(-1.0, min(1.0, v_ant[2])))
    return el, az

def get_gain_from_angles(el_rad, az_rad, max_atten=30.0):
    el_deg = math.degrees(el_rad)
    az_deg = math.degrees(az_rad)
    e_gain = get_e_gain(el_deg)
    h_gain = get_h_gain(az_deg)
    g_peak = max(max(e_gains) if e_gains else 0.0, max(h_gains) if h_gains else 0.0)
    
    e_atten = min(g_peak - e_gain, max_atten)
    h_atten = min(g_peak - h_gain, max_atten)
    total = g_peak - e_atten - h_atten
    return e_gain, h_gain, max(total, g_peak - max_atten)

# Simulation Config
y_pos = 3.0
y_dist = 1.5
tx_power = -7.0

def run_simulation(angle_deg):
    world_yaw = math.radians(angle_deg) - math.pi
    entity_yaw = math.radians(-90.0)
    antenna_yaw = world_yaw - entity_yaw
    
    bs_pos = [0.0, y_pos, 1.0]
    bs_rpy = [0.0, 0.0, entity_yaw + antenna_yaw]
    
    ugv_antenna_yaw = world_yaw - math.pi
    
    print(f"\nSimulation for angle = {angle_deg} deg")
    print(f"BS pos: {bs_pos}, BS yaw: {math.degrees(bs_rpy[2]):.2f} deg")
    print(f"Train ant yaw: {math.degrees(ugv_antenna_yaw):.2f} deg")
    print("-" * 115)
    print(f"{'X_ant (m)':>10} | {'Dist (m)':>8} | {'BS_az (deg)':>11} | {'BS_gain':>8} | {'TX_az (deg)':>11} | {'TX_gain':>8} | {'G_total':>8} | {'PL (dB)':>8} | {'RSSI (dBm)':>10}")
    print("-" * 115)
    
    # Sweep X from -2.0 to 2.0 with step 0.1
    # also add exact event points:
    # 1.7344m => X = sqrt(1.7344^2 - 1.5^2) = 0.8707
    # 1.6660m => X = sqrt(1.6660^2 - 1.5^2) = 0.7250
    # 1.5330m => X = sqrt(1.5330^2 - 1.5^2) = 0.3164
    x_points = []
    for i in range(-20, 21):
        x_points.append(i * 0.1)
    
    # Insert specific test points
    for sign in [-1, 1]:
        x_points.extend([
            sign * math.sqrt(1.7344**2 - 1.5**2),
            sign * math.sqrt(1.6660**2 - 1.5**2),
            sign * math.sqrt(1.5330**2 - 1.5**2)
        ])
    x_points = sorted(list(set(x_points)))
    
    for x in x_points:
        tx_pos = [x, 1.5, 1.0]
        tx_rpy = [0.0, 0.0, ugv_antenna_yaw]
        
        bs_el, bs_az = calculate_antenna_frame_angles(bs_pos, tx_pos, bs_rpy)
        tx_el, tx_az = calculate_antenna_frame_angles(tx_pos, bs_pos, tx_rpy)
        
        _, _, g_bs = get_gain_from_angles(bs_el, bs_az)
        _, _, g_tx = get_gain_from_angles(tx_el, tx_az)
        
        g_total = g_bs + g_tx
        d = math.sqrt(x**2 + y_dist**2)
        pl = 68.0 + 20.0 * math.log10(d)
        rssi = tx_power - pl + g_total
        
        # Output lines matching test distances of interest:
        is_highlight = False
        for target_d in [1.7344, 1.6660, 1.5330]:
            if abs(d - target_d) < 0.001:
                is_highlight = True
                
        prefix = "--> " if is_highlight else "    "
        print(f"{prefix}{x:8.4f} | {d:8.4f} | {math.degrees(bs_az):11.2f} | {g_bs:8.3f} | {math.degrees(tx_az):11.2f} | {g_tx:8.3f} | {g_total:8.3f} | {pl:8.2f} | {rssi:10.2f}")

run_simulation(80.0)
print("\n" + "="*80)
run_simulation(100.0)
