import math

def estimate_expected_duration(config_path, rtf):
    try:
        import yaml
        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
            
        # 1. 起動遅延の取得
        timing = config.get('timing', {})
        tx_delay = float(timing.get('tx_controller_delay', 3.5))
        start_delay_wall = tx_delay
        
        # 2. TX物理パラメータの取得
        tx_common = config.get('tx_controller_common', {})
        if not tx_common:
            tx_common = config.get('tx_controller_node', {}).get('ros__parameters', {})
        a_max = float(tx_common.get('max_acceleration', 0.5))
        
        # 3. 車両ルートに基づく所要時間の計算
        vehicles = config.get('vehicles', [])
        if not vehicles:
            return None
            
        vehicle = vehicles[0]
        spawn_pose = vehicle.get('pose', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        waypoints = vehicle.get('waypoints', [])
        if not waypoints:
            return None
            
        total_distance = 0.0
        last_pt = [float(spawn_pose[0]), float(spawn_pose[1]), float(spawn_pose[2])]
        segment_times = []
        current_vel = 0.0
        
        for wp in waypoints:
            wp_x, wp_y, wp_z, wp_v = float(wp[0]), float(wp[1]), float(wp[2]), float(wp[3])
            dist = math.sqrt((wp_x - last_pt[0])**2 + (wp_y - last_pt[1])**2 + (wp_z - last_pt[2])**2)
            if dist > 0.001:
                # 目標速度への加速時間を計算
                t_acc = abs(wp_v - current_vel) / a_max
                # 加速中の移動距離
                d_acc = 0.5 * (current_vel + wp_v) * t_acc
                
                if d_acc <= dist:
                    d_const = dist - d_acc
                    t_const = d_const / wp_v if wp_v > 0.0 else 0.0
                    t_seg = t_acc + t_const
                    current_vel = wp_v
                else:
                    # 加速区間だけで目的地を通り過ぎてしまう場合
                    a = a_max if wp_v > current_vel else -a_max
                    term = current_vel**2 + 2.0 * a * dist
                    if term >= 0.0:
                        t_seg = (-current_vel + math.sqrt(term)) / a
                        current_vel = current_vel + a * t_seg
                    else:
                        t_seg = dist / max(0.1, current_vel)
                segment_times.append(t_seg)
                last_pt = [wp_x, wp_y, wp_z]
                
        t_sim_movement = sum(segment_times)
        t_expected_wall = start_delay_wall + (t_sim_movement / rtf)
        return t_expected_wall
    except Exception as e:
        print(f"Warning: Failed to estimate expected completion time: {e}")
        return None
