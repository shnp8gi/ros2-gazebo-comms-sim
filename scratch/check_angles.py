from comms_sim_pkg.antenna_parser import AntennaPatternParser
p = AntennaPatternParser(
    e_plane_path='/workspace/src/comms_sim_pkg/config/e_plane.csv',
    h_plane_path='/workspace/src/comms_sim_pkg/config/h_plane.csv',
    max_antenna_attenuation=30.0,
    mainlobe_angle_margin_deg=5.0,
    mainlobe_e_half_angle_override_deg=-1.0,
    mainlobe_h_half_angle_override_deg=-1.0
)
print('E mainlobe half angle:', p.e_mainlobe_half_angle)
print('H mainlobe half angle:', p.h_mainlobe_half_angle)
