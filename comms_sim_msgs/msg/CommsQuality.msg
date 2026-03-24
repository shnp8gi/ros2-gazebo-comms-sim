# CommsQuality.msg
# 通信品質メトリクスのカスタムメッセージ定義

# タイムスタンプ付きヘッダ
std_msgs/Header header

# UGV-基地局間の距離 [m]
float64 distance

# 受信信号強度 (RSSI) [dBm]
float64 rssi

# 瞬時スループット [Gbps]
float64 throughput

# 累計伝送データ量 [Mb]
float64 total_data_transmitted

# UGVのワールド座標位置 [m]
float64 ugv_x
float64 ugv_y
float64 ugv_z

# 基地局のワールド座標位置 [m]
float64 base_station_x
float64 base_station_y
float64 base_station_z

# アンテナゲイン [dBi]
float64 antenna_gain_e_plane
float64 antenna_gain_h_plane

# パスロス [dB]
float64 path_loss

# 通信アクティブフラグ
bool comm_active
