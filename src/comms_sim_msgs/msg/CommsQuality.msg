# CommsQuality.msg
# 通信品質メトリクスのカスタムメッセージ定義

# タイムスタンプ付きヘッダ
std_msgs/Header header

# TX-基地局間の距離 [m]
float64 distance

# 受信信号強度 (RSSI) [dBm]
float64 rssi

# 瞬時スループット [Gbps]
float64 throughput

# 累計伝送データ量 [Mb]
float64 total_data_transmitted

# TXのワールド座標位置 [m]
float64 tx_x
float64 tx_y
float64 tx_z

# 基地局のワールド座標位置 [m]
float64 rx_x
float64 rx_y
float64 rx_z

# アンテナゲイン [dBi]
float64 antenna_gain_e_plane
float64 antenna_gain_h_plane

# パスロス [dB]
float64 path_loss

# 通信アクティブフラグ
bool comm_active

# リンク確立状態 (DISCONNECTED, ESTABLISHING, CONNECTED)
string link_state
