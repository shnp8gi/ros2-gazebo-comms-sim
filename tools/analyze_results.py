#!/usr/bin/env python3
import pandas as pd
import sys
import numpy as np

def analyze_log(csv_path):
    print(f"Analyzing {csv_path}...")
    try:
        df = pd.read_csv(csv_path)
    except Exception as e:
        print(f"Error reading CSV: {e}")
        return

    print("="*50)
    print("定量分析レポート (Quantitative Analysis)")
    print("="*50)

    # 1. 接続状態の統計
    connected_df = df[df['link_state'] == 'CONNECTED']
    total_time = df['time_s'].iloc[-1] - df['time_s'].iloc[0] if len(df) > 0 else 0
    connected_time = len(connected_df) * 0.001 # Assuming 1ms dt based on Gazebo updates
    
    # Let's actually use timestamps
    if len(connected_df) > 1:
        connected_time = connected_df['time_s'].max() - connected_df['time_s'].min()
    elif len(connected_df) == 1:
        connected_time = 0.001
    else:
        connected_time = 0

    print(f"【接続性能】")
    print(f"  - 合計シミュレーション時間: {total_time:.2f} s")
    print(f"  - 接続維持時間: {connected_time:.2f} s")
    print(f"  - 接続率: {(connected_time/total_time)*100 if total_time > 0 else 0:.2f} %")

    # 2. 物理演算の統計
    print(f"\n【物理演算パラメータ】")
    valid_rssi = df[df['rssi_dBm'] > -900] # Ignore -999 disconnected defaults
    if len(valid_rssi) > 0:
        print(f"  - 最大 RSSI: {valid_rssi['rssi_dBm'].max():.2f} dBm")
        print(f"  - 平均 RSSI (受信範囲内): {valid_rssi['rssi_dBm'].mean():.2f} dBm")
        print(f"  - 最小 Path Loss: {valid_rssi['path_loss_dB'].min():.2f} dB")
        print(f"  - アンテナゲイン (TX): {df['tx_antenna_gain_dB'].mean():.2f} dB")
        print(f"  - アンテナゲイン (RX): {df['rx_antenna_gain_dB'].mean():.2f} dB")
    else:
        print("  - 有効な RSSI データなし (常に圏外または -999 dBm)")
        print(f"  - 記録された平均 RX ゲイン: {df['rx_antenna_gain_dB'].mean():.2f} dB")
        print(f"  - 記録された平均 オフボアサイト角: {df['off_boresight_h_deg'].mean():.2f} deg")

    # 3. スループットとデータ量
    print(f"\n【通信パフォーマンス】")
    print(f"  - 最大スループット: {df['throughput_Gbps'].max():.2f} Gbps")
    print(f"  - 平均スループット (接続時): {connected_df['throughput_Gbps'].mean() if len(connected_df) > 0 else 0:.2f} Gbps")
    print(f"  - 転送完了データ量: {df['total_data_MB'].max():.2f} MB")

    # 4. ハンドオーバーと安定性
    # Find edges where link_state changes from DISCONNECTED to CONNECTED
    transitions = (df['link_state'] != df['link_state'].shift(1)) & (df['link_state'] == 'CONNECTED')
    ho_count = transitions.sum()
    print(f"\n【ハンドオーバー特性】")
    print(f"  - 接続確立回数 (HO/新規接続): {ho_count} 回")
    
    print("="*50)

if __name__ == "__main__":
    if len(sys.argv) > 1:
        analyze_log(sys.argv[1])
    else:
        print("Usage: python3 analyze_results.py <csv_path>")
