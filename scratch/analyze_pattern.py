# Load H-plane pattern without numpy
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
            except ValueError as e:
                print(f"Error parsing line {line}: {e}")
                continue

# Find max/min
max_gain = max(h_gains)
max_gain_idx = h_gains.index(max_gain)
min_gain = min(h_gains)
min_gain_idx = h_gains.index(min_gain)

print(f"Max gain: {max_gain} dBi at {h_angles[max_gain_idx]} degrees")
print(f"Min gain: {min_gain} dBi at {h_angles[min_gain_idx]} degrees")

# Print typical points
print("\nGain at typical angles:")
for a in [-90, -80, -70, -60, -50, -40, -30, -20, -10, 0, 10, 20, 30, 40, 50, 60, 70, 80, 90]:
    # find closest angle
    closest_idx = min(range(len(h_angles)), key=lambda i: abs(h_angles[i] - a))
    print(f"  {a:3d} deg: {h_gains[closest_idx]:.3f} dBi (closest {h_angles[closest_idx]:.1f})")

# Let's inspect local minima (nulls) and maxima (sidelobes)
print("\nLocal extrema:")
for i in range(1, len(h_gains) - 1):
    if h_gains[i] > h_gains[i-1] and h_gains[i] > h_gains[i+1]:
        # Local max
        print(f"  Local Max: {h_gains[i]:.3f} dBi at {h_angles[i]:.1f} deg")
    elif h_gains[i] < h_gains[i-1] and h_gains[i] < h_gains[i+1]:
        # Local min
        print(f"  Local Min (Null): {h_gains[i]:.3f} dBi at {h_angles[i]:.1f} deg")
