# CommsQuality.msg
# Custom message for communication quality metrics

# Header with timestamp
std_msgs/Header header

# Distance between UGV and base station [m]
float64 distance

# Received Signal Strength Indicator [dBm]
float64 rssi

# Instantaneous throughput [Gbps]
float64 throughput

# Total data transmitted [Mb]
float64 total_data_transmitted

# UGV position in world coordinates [m]
float64 ugv_x
float64 ugv_y
float64 ugv_z

# Base station position in world coordinates [m]
float64 base_station_x
float64 base_station_y
float64 base_station_z

# Antenna gains [dBi]
float64 antenna_gain_e_plane
float64 antenna_gain_h_plane

# Path loss [dB]
float64 path_loss

# Communication active flag
bool comm_active
