import glob
import os
import csv

# Find the latest sweep directory
sweep_dirs = sorted(glob.glob('/workspace/sim_results/sweep_*'))
if not sweep_dirs:
    print('No sweep directories found')
    exit()

latest_sweep = '/workspace/sim_results/sweep_20260608_151724'
summary_path = os.path.join(latest_sweep, 'sweep_summary.csv')
print('Reading sweep summary:', summary_path)

if not os.path.exists(summary_path):
    print(f'File {summary_path} does not exist')
    exit()

# Parse CSV
data = []
with open(summary_path, 'r', encoding='utf-8') as f:
    reader = csv.reader(f)
    header = next(reader)
    # Header format: type, y_pos, antenna_angle, vehicle_name, total_data_MB, connected_time_s, average_rssi_dBm, handover_count, ...
    for row in reader:
        if not row:
            continue
        # Only process AVERAGE rows
        if row[0] == 'AVERAGE':
            try:
                y_pos = float(row[1])
                angle = float(row[2])
                name = row[3]
                total_data = float(row[4])
                conn_time = float(row[5])
                avg_tp = float(row[6])
                avg_rssi = float(row[7])
                handover = float(row[8])
                data.append({
                    'y_pos': y_pos,
                    'angle': angle,
                    'name': name,
                    'total_data': total_data,
                    'conn_time': conn_time,
                    'avg_rssi': avg_rssi,
                    'handover': handover
                })
            except Exception as e:
                print(f"Error parsing row {row}: {e}")

# Filter for shinkansen
shinkansen_data = [d for d in data if 'shinkansen' in d['name']]
shinkansen_data.sort(key=lambda x: (x['angle'], x['name']))

print(f"{'Angle':>6} | {'Antenna':>17} | {'Conn Time (s)':>14} | {'Avg RSSI (dBm)':>15} | {'Handovers':>10} | {'Total Data (MB)':>16}")
print("-" * 90)
for d in shinkansen_data:
    if 30.0 <= d['angle'] <= 150.0:
        if abs(d['angle'] % 10.0) < 1e-3:
            print(f"{d['angle']:6.1f} | {d['name']:17} | {d['conn_time']:14.4f} | {d['avg_rssi']:15.3f} | {d['handover']:10.1f} | {d['total_data']:16.3f}")
    elif abs(d['angle'] % 30.0) < 1e-3:
        print(f"{d['angle']:6.1f} | {d['name']:17} | {d['conn_time']:14.4f} | {d['avg_rssi']:15.3f} | {d['handover']:10.1f} | {d['total_data']:16.3f}")
