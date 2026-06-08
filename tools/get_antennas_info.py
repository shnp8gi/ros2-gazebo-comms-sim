#!/usr/bin/env python3
import os
import yaml

def get_entities_and_antennas(config_path):
    """
    sim_params.yaml から動的にエンティティ名とアンテナ情報を取得する
    """
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"Config file not found: {config_path}")
        
    with open(config_path, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)
        
    vehicles = config.get('vehicles', [])
    result = {}
    
    for vehicle in vehicles:
        vehicle_name = vehicle.get('name', 'unknown')
        antennas = vehicle.get('antennas', [])
        
        # アンテナ名のリストを取得
        antenna_names = [ant.get('name') for ant in antennas if ant.get('name')]
        
        result[vehicle_name] = {
            'count': len(antennas),
            'antennas': antenna_names
        }
        
    return result

if __name__ == "__main__":
    # ワークスペースルートを自動検出
    script_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_root = os.path.dirname(script_dir)
    config_file = os.path.join(workspace_root, "src/comms_sim_pkg/config/sim_params.yaml")
    
    try:
        info = get_entities_and_antennas(config_file)
        print("=== 検出されたエンティティとアンテナ構成 ===")
        for entity, data in info.items():
            print(f"エンティティ名: {entity}")
            print(f"  アンテナ数  : {data['count']}台")
            print(f"  アンテナ一覧:")
            for ant in data['antennas']:
                print(f"    - {ant}")
            print()
    except Exception as e:
        print(f"エラーが発生しました: {e}")
