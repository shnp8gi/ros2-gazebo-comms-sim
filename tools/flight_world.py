"""飛行の条件（flight_*.yaml）から、物理ベースの機体2機を含む Gazebo のワールドを作る。"""
import math
import random

ROTOR_MASS = 0.005
GIMBAL_MASS = 0.01
SLOWDOWN = 10          # ロータの関節は実際の 1/10 の速さで回す（物理計算の安定のため）
ANTENNA_BELOW = 0.06   # アンテナの関節の位置（機体中心からの下向きの距離）[m]
ANTENNA_COM = 0.03     # 関節からアンテナの重心までの距離（正面方向）[m]
ANTENNA_SIZE = 0.05


def box_inertia(mass, x, y, z):
    return mass / 12 * (y * y + z * z), mass / 12 * (x * x + z * z), mass / 12 * (x * x + y * y)


def inertial(mass, ixx, iyy, izz, pose='0 0 0 0 0 0'):
    return (f'<inertial><pose>{pose}</pose><mass>{mass}</mass><inertia><ixx>{ixx}</ixx><iyy>{iyy}</iyy>'
            f'<izz>{izz}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>')


def rotor_positions(arm):
    """quad_controller.hpp の quad_rotors と同じ並び。"""
    l = arm / math.sqrt(2)
    return [(l, -l, 'ccw'), (-l, l, 'ccw'), (l, l, 'cw'), (-l, -l, 'cw')]


def distance_to_center(config):
    """助走を含めて、encounter_time_s までに進む距離。"""
    s = config['scenario']
    ramp = s['speed_mps'] / s['acceleration_mps2']
    return s['speed_mps'] * (s['encounter_time_s'] - s['start_time_s'] - 0.5 * ramp)


def model(name, config, position, heading, elevation_deg, seed):
    v, a, c = config['vehicle'], config['antenna'], config['controller']
    drag, vib = config['drag'], config['vibration']
    spring = a['joint'] == 'spring'
    base_mass = v['mass_kg'] - 4 * ROTOR_MASS - a['mass_kg'] - (GIMBAL_MASS if spring else 0)
    ixx, iyy, izz = v['inertia_kgm2']
    x, y, z = position
    parts = [f'<model name="{name}"><pose>{x} {y} {z} 0 0 {heading}</pose>',
             f'<link name="base_link">{inertial(base_mass, ixx, iyy, izz)}'
             '<visual name="body"><geometry><box><size>0.3 0.3 0.06</size></box></geometry>'
             '<material><diffuse>0.2 0.2 0.25 1</diffuse></material></visual></link>']
    for i, (rx, ry, turning) in enumerate(rotor_positions(v['arm_m'])):
        parts.append(
            f'<link name="rotor_{i}"><pose relative_to="base_link">{rx} {ry} 0.03 0 0 0</pose>'
            f'{inertial(ROTOR_MASS, 9.75e-07, 4.17e-05, 4.26e-05)}'
            '<visual name="disc"><geometry><cylinder><radius>0.1</radius><length>0.005</length></cylinder></geometry>'
            '<material><diffuse>0.6 0.6 0.6 0.6</diffuse></material></visual></link>'
            f'<joint name="rotor_{i}_joint" type="revolute"><parent>base_link</parent><child>rotor_{i}</child>'
            '<axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis></joint>')
    # アンテナ。リンクの +X が正面。既定の取り付けは機体の左真横（方位 +90°）、仰角 elevation_deg。
    mount = f'0 0 {-ANTENNA_BELOW} 0 {-math.radians(elevation_deg)} {math.pi / 2}'
    ax, ay, az = box_inertia(a['mass_kg'], ANTENNA_SIZE, ANTENNA_SIZE, ANTENNA_SIZE)
    parts.append(
        f'<link name="antenna_link"><pose relative_to="base_link">{mount}</pose>'
        f'{inertial(a["mass_kg"], ax, ay, az, f"{ANTENNA_COM} 0 0 0 0 0")}'
        f'<visual name="horn"><pose>{ANTENNA_COM} 0 0 0 0 0</pose><geometry><box><size>{ANTENNA_SIZE} {ANTENNA_SIZE} {ANTENNA_SIZE}</size></box></geometry>'
        '<material><diffuse>0.9 0.5 0.1 1</diffuse></material></visual></link>')
    if spring:
        # ピッチ・ヨーの2軸。固有振動数と減衰比から、ばね定数 k = I ω_n²、減衰 c = 2 ζ I ω_n を決める。
        omega = 2 * math.pi * a['natural_frequency_hz']
        about_joint = ay + a['mass_kg'] * ANTENNA_COM ** 2
        stiffness, damping = about_joint * omega ** 2, 2 * a['damping_ratio'] * about_joint * omega
        dynamics = f'<dynamics><damping>{damping}</damping><spring_stiffness>{stiffness}</spring_stiffness><spring_reference>0</spring_reference></dynamics>'
        parts.append(
            f'<link name="antenna_gimbal"><pose relative_to="base_link">{mount}</pose>'
            f'{inertial(GIMBAL_MASS, 1e-06, 1e-06, 1e-06)}</link>'
            '<joint name="antenna_pitch_joint" type="revolute"><parent>base_link</parent><child>antenna_gimbal</child>'
            f'<axis><xyz>0 1 0</xyz><limit><lower>-0.5</lower><upper>0.5</upper></limit>{dynamics}</axis></joint>'
            '<joint name="antenna_yaw_joint" type="revolute"><parent>antenna_gimbal</parent><child>antenna_link</child>'
            f'<axis><xyz>0 0 1</xyz><limit><lower>-0.5</lower><upper>0.5</upper></limit>{dynamics}</axis></joint>')
    else:
        parts.append('<joint name="antenna_joint" type="fixed"><parent>base_link</parent><child>antenna_link</child></joint>')
    s = config['scenario']
    direction = (math.cos(heading), math.sin(heading), 0)
    parts.append(
        '<plugin filename="QuadControllerPlugin" name="comms_sim::QuadControllerPlugin">'
        f'<mass>{v["mass_kg"]}</mass><arm>{v["arm_m"]}</arm><ixx>{ixx}</ixx><iyy>{iyy}</iyy><izz>{izz}</izz>'
        f'<motor_constant>{v["motor_constant"]}</motor_constant><moment_constant>{v["moment_constant"]}</moment_constant>'
        f'<max_rot_velocity>{v["max_rot_velocity"]}</max_rot_velocity>'
        + ''.join(f'<{key}>{c[key]}</{key}>' for key in sorted(c))
        + f'<line_start>{x} {y} {z}</line_start><line_direction>{direction[0]} {direction[1]} 0</line_direction>'
        f'<cruise_speed>{s["speed_mps"]}</cruise_speed><acceleration>{s["acceleration_mps2"]}</acceleration>'
        f'<start_time>{s["start_time_s"]}</start_time><yaw>{heading}</yaw></plugin>')
    for i, (_, _, turning) in enumerate(rotor_positions(v['arm_m'])):
        parts.append(
            '<plugin filename="gz-sim-multicopter-motor-model-system" name="gz::sim::systems::MulticopterMotorModel">'
            f'<robotNamespace>{name}</robotNamespace><jointName>rotor_{i}_joint</jointName><linkName>rotor_{i}</linkName>'
            f'<turningDirection>{turning}</turningDirection>'
            f'<timeConstantUp>{v["motor_time_constant_up_s"]}</timeConstantUp><timeConstantDown>{v["motor_time_constant_down_s"]}</timeConstantDown>'
            f'<maxRotVelocity>{v["max_rot_velocity"]}</maxRotVelocity><motorConstant>{v["motor_constant"]}</motorConstant>'
            f'<momentConstant>{v["moment_constant"]}</momentConstant><commandSubTopic>rotor_cmd</commandSubTopic>'
            f'<actuator_number>{i}</actuator_number><rotorDragCoefficient>0</rotorDragCoefficient>'
            f'<rollingMomentCoefficient>0</rollingMomentCoefficient><motorSpeedPubTopic>motor_speed/{i}</motorSpeedPubTopic>'
            f'<rotorVelocitySlowdownSim>{SLOWDOWN}</rotorVelocitySlowdownSim><motorType>velocity</motorType></plugin>')
    if drag['enabled']:
        wind = ' '.join(str(w) for w in drag['wind_mps'])
        parts.append('<plugin filename="DragPlugin" name="comms_sim::DragPlugin">'
                     f'<air_density>{drag["air_density"]}</air_density><cd_area>{drag["cd_area_m2"]}</cd_area>'
                     f'<wind>{wind}</wind></plugin>')
    if vib['enabled']:
        rng = random.Random(seed)
        phases = ' '.join(f'{rng.uniform(0, 2 * math.pi):.9f}' for _ in range(8))
        positions = ' '.join(f'{rx} {ry} 0.03' for rx, ry, _ in rotor_positions(v['arm_m']))
        parts.append('<plugin filename="RotorVibrationPlugin" name="comms_sim::RotorVibrationPlugin">'
                     f'<unbalance>{vib["unbalance_kgm"]}</unbalance><thrust_ripple>{vib["thrust_ripple"]}</thrust_ripple>'
                     f'<blades>{v["rotor_blades"]}</blades><motor_constant>{v["motor_constant"]}</motor_constant>'
                     f'<rotor_velocity_slowdown>{SLOWDOWN}</rotor_velocity_slowdown>'
                     f'<rotor_positions>{positions}</rotor_positions><phases>{phases}</phases></plugin>')
    parts.append('</model>')
    return '\n'.join(parts)


def world(config, output_dir, gui=False):
    s = config['scenario']
    offset = s['d_min_m'] / math.sqrt(2)
    run = distance_to_center(config)
    elevation = config['antenna']['elevation_deg']
    seed = config['vibration']['seed']
    # A は +x 向き、B は −x 向き。B は横に offset、上に offset ずらす（解析軌道と同じ配置）。
    a = model('drone_a', config, (-run, 0, s['altitude_a_m']), 0, elevation, seed)
    b = model('drone_b', config, (run, offset, s['altitude_a_m'] + offset), math.pi, -elevation, seed + 1)
    scene = ('<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>'
             '<plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>') if gui else ''
    return f'''<?xml version="1.0"?>
<sdf version="1.9">
<world name="drone_flight">
<physics name="default" type="ignored"><max_step_size>{config["simulation"]["physics_step_s"]}</max_step_size>
<real_time_factor>{1 if gui else 0}</real_time_factor></physics>
<gravity>0 0 -9.81</gravity>
<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
{scene}
<plugin filename="FlightRecorderPlugin" name="comms_sim::FlightRecorderPlugin">
<output_dir>{output_dir}</output_dir><duration>{s["duration_s"]}</duration>
<rotor_velocity_slowdown>{SLOWDOWN}</rotor_velocity_slowdown></plugin>
<light name="sun" type="directional"><pose>0 0 100 0 0 0</pose><direction>-0.3 0.2 -1</direction></light>
{a}
{b}
</world>
</sdf>
'''
