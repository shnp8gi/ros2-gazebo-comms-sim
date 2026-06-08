import math

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
    # find closest two angles
    for i in range(len(h_angles) - 1):
        if h_angles[i] <= angle_deg <= h_angles[i+1]:
            # interpolate
            t = (angle_deg - h_angles[i]) / (h_angles[i+1] - h_angles[i])
            return h_gains[i] + t * (h_gains[i+1] - h_gains[i])
    # fallback
    closest_idx = min(range(len(h_angles)), key=lambda i: abs(h_angles[i] - angle_deg))
    return h_gains[closest_idx]

# Constants
antenna_angle_deg = 80.0
world_yaw = math.radians(antenna_angle_deg) - math.pi # world direction of antenna
y_dist = 1.5
tx_power = -7.0

print(f"Antenna angle: {antenna_angle_deg} deg")
print(f"World yaw of antenna: {math.degrees(world_yaw):.1f} deg\n")

print(f"{'X (m)':>8} | {'Dist (m)':>8} | {'Phi (deg)':>10} | {'Rel Ang (deg)':>14} | {'Gain (dBi)':>10} | {'G_total (dB)':>12} | {'PL (dB)':>8} | {'RSSI (dBm)':>10}")
print("-" * 95)

# Test specific distances from events.csv
# d = 1.7344, 1.6660, 1.5330
distances = [1.7344, 1.6660, 1.5330]
for d in distances:
    # d^2 = x^2 + y_dist^2 => x = sqrt(d^2 - y_dist^2)
    # We test both approaching (x < 0) and departing (x > 0)
    for sign in [-1, 1]:
        x = sign * math.sqrt(d**2 - y_dist**2)
        phi = math.atan2(-y_dist, x) # Angle of vector from BS (0, 1.5) to Train (x, 3.0) -> relative vector is (x, 1.5)
        # Note: in theoretical_optimum, vector from BS (0, y_dist) to Train (x, 0) is (x, -y_dist).
        # Here we follow theoretical_optimum's vector:
        phi_theoretical = math.atan2(-y_dist, x)
        rel_angle_rad = phi_theoretical - world_yaw
        # wrap
        rel_angle_rad = (rel_angle_rad + math.pi) % (2.0 * math.pi) - math.pi
        rel_angle_deg = math.degrees(rel_angle_rad)
        
        g_bs = get_h_gain(rel_angle_deg)
        g_tx = get_h_gain(rel_angle_deg)
        g_total = g_bs + g_tx
        
        pl = 68.0 + 20.0 * math.log10(d)
        rssi = tx_power - pl + g_total
        
        print(f"{x:8.4f} | {d:8.4f} | {math.degrees(phi_theoretical):10.1f} | {rel_angle_deg:14.2f} | {g_bs:10.3f} | {g_total:12.3f} | {pl:8.2f} | {rssi:10.2f}")
