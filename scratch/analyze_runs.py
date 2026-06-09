import os
import csv
import glob

sweep_dir = '/workspace/sim_results/sweep_20260608_151724'

def analyze_run(run_dir):
    run_name = os.path.basename(run_dir)
    # Parse angle
    parts = run_name.split('_a')
    if len(parts) < 2:
        return
    angle_str = parts[1]
    
    # We want to check shinkansen_mid, front, rear
    connected_data = []
    
    for ant_name in ['shinkansen_front', 'shinkansen_mid', 'shinkansen_rear']:
        csv_path = os.path.join(run_dir, f'comms/{ant_name}_full.csv')
        if not os.path.exists(csv_path):
            continue
        
        with open(csv_path, 'r') as f:
            reader = csv.reader(f)
            header = next(reader)
            # Find column indices
            time_idx = header.index('time_s')
            grant_idx = header.index('has_link_grant')
            rssi_idx = header.index('rssi_dBm')
            state_idx = header.index('link_state')
            dist_idx = header.index('distance_m')
            
            connected_samples = []
            for row in reader:
                if not row:
                    continue
                if row[state_idx] == 'CONNECTED':
                    connected_samples.append({
                        'time': float(row[time_idx]),
                        'grant': row[grant_idx],
                        'rssi': float(row[rssi_idx]),
                        'dist': float(row[dist_idx]),
                        'ant': ant_name
                    })
            
            if connected_samples:
                connected_data.extend(connected_samples)
                
    if not connected_data:
        print(f"Run {run_name} (Angle {angle_str}): No CONNECTED samples")
        return
        
    connected_data.sort(key=lambda x: x['time'])
    
    # Identify segments (where gap > 0.05s)
    segments = []
    current_seg = [connected_data[0]]
    for item in connected_data[1:]:
        if item['time'] - current_seg[-1]['time'] > 0.05:
            segments.append(current_seg)
            current_seg = [item]
        else:
            current_seg.append(item)
    segments.append(current_seg)
    
    print(f"\n==========================================")
    print(f"Run {run_name} (Angle {angle_str}) Analysis:")
    print(f"Total CONNECTED samples: {len(connected_data)}")
    print(f"Number of connected segments: {len(segments)}")
    for i, seg in enumerate(segments):
        t_start = seg[0]['time']
        t_end = seg[-1]['time']
        duration = t_end - t_start
        rssis = [x['rssi'] for x in seg]
        dists = [x['dist'] for x in seg]
        avg_rssi = sum(rssis) / len(rssis)
        min_rssi = min(rssis)
        max_rssi = max(rssis)
        print(f"  Segment {i+1}: t = {t_start:.4f} to {t_end:.4f} ({duration:.4f}s)")
        print(f"    Antennas used: {set(x['ant'] for x in seg)}")
        print(f"    RSSI: avg={avg_rssi:.2f}, min={min_rssi:.2f}, max={max_rssi:.2f}")
        print(f"    Distance: min={min(dists):.2f}m, max={max(dists):.2f}m")

# Analyze 30.0, 30.1, 30.2, 30.3, 30.4, 30.5
for ang in ['30', '30.1', '30.2', '30.3', '30.4', '30.5', '90']:
    run_dir = os.path.join(sweep_dir, f'runs/run_001_y3_a{ang}')
    if os.path.exists(run_dir):
        analyze_run(run_dir)
