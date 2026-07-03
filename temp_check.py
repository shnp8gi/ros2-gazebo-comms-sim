import pandas as pd

df = pd.read_csv('sim_results/sweep_20260703_113304/shinkansen_shinkansen_front_log.csv')
min_idx = df['distance_m'].idxmin()
print(df.loc[min_idx][['time_s', 'tx_x', 'bs_x', 'distance_m', 'rssi_dBm', 'path_loss_dB', 'tx_antenna_gain_dB', 'rx_antenna_gain_dB', 'link_state']])
