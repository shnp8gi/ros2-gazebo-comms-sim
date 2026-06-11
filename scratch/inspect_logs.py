import pandas as pd
import glob
import os

# Find the latest sweep directory
sweep_dirs = sorted(glob.glob('/workspace/sim_results/sweep_*'))
if not sweep_dirs:
    print('No sweep directories found')
    exit()

latest_sweep = sweep_dirs[-1]
print('Latest sweep dir:', latest_sweep)

# Read the full log for mid antenna
csv_path = os.path.join(latest_sweep, 'runs/run_001_y3_a100/comms/shinkansen_mid_full.csv')
if not os.path.exists(csv_path):
    print(f'File {csv_path} does not exist')
    exit()

df = pd.read_csv(csv_path)

# Filter rows in time range [11.50, 11.70]
df_range = df[(df['time_s'] >= 11.50) & (df['time_s'] <= 11.70)].copy()
if df_range.empty:
    print('No rows in that time range')
else:
    print('All rows in time range [11.50, 11.70]:')
    cols = ['time_s', 'has_link_grant', 'distance_m', 'rssi_dBm', 'throughput_Gbps', 'link_state']
    print(df_range[cols].to_string())
