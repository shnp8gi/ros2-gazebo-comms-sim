#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import abc
from typing import Dict, List, Optional, Tuple

class LinkSchedulingStrategy(abc.ABC):
    """
    リンク制御（スケジューリング）のためのインターフェース。
    """
    
    @abc.abstractmethod
    def determine_active_link(
        self, 
        rssi_dict: Dict[str, float], 
        mission_complete_dict: Dict[str, bool],
        geometry_info: Dict[str, dict],
        current_active_idx: int,
        vehicle_names: List[str],
        current_time: float
    ) -> Tuple[int, Optional[str]]:
        """
        次に有効なリンク権を持つべき車両のインデックスを決定する。
        
        Args:
            rssi_dict: 各車両の最新RSSI
            mission_complete_dict: 各車両のミッション完了状態
            geometry_info: 幾何学情報（distance, antenna_gain_e_plane, comm_active等を格納した辞書）
            current_active_idx: 現在アクティブな車両インデックス
            vehicle_names: 車両名リスト
            current_time: 現在時刻 [s]
            
        Returns:
            Tuple[int, Optional[str]]: (新しいアクティブインデックス, ログ出力用メッセージ(変更がなければNone))
        """
        pass

class SequentialStrategy(LinkSchedulingStrategy):
    """
    sequential ポリシー:
    現在の車両のミッションが完了したら次の車両に切替。
    """
    def determine_active_link(
        self, 
        rssi_dict, 
        mission_complete_dict,
        geometry_info,
        current_active_idx,
        vehicle_names,
        current_time
    ):
        current_name = vehicle_names[current_active_idx]
        if mission_complete_dict.get(current_name, False):
            next_idx = current_active_idx + 1
            if next_idx < len(vehicle_names):
                msg = f'リンク切替: {current_name} → {vehicle_names[next_idx]} (sequential: ミッション完了)'
                return next_idx, msg
        return current_active_idx, None

class RoundRobinStrategy(LinkSchedulingStrategy):
    """
    round_robin ポリシー:
    一定時間ごとに次の車両に切替。
    """
    def __init__(self, time_slot_duration: float):
        self.time_slot_duration = time_slot_duration
        self._last_slot_switch_time: Optional[float] = None

    def determine_active_link(
        self, 
        rssi_dict, 
        mission_complete_dict,
        geometry_info,
        current_active_idx,
        vehicle_names,
        current_time
    ):
        if self._last_slot_switch_time is None:
            self._last_slot_switch_time = current_time
            return current_active_idx, None

        elapsed = current_time - self._last_slot_switch_time
        if elapsed >= self.time_slot_duration:
            old_name = vehicle_names[current_active_idx]
            new_idx = (current_active_idx + 1) % len(vehicle_names)
            self._last_slot_switch_time = current_time
            msg = f'リンク切替: {old_name} → {vehicle_names[new_idx]} (round_robin: {elapsed:.1f}s 経過)'
            return new_idx, msg
            
        return current_active_idx, None

class RssiPriorityStrategy(LinkSchedulingStrategy):
    """
    rssi_priority ポリシー:
    RSSIが最も高い車両にリンク権を付与。
    しかし閾値を下回っている場合は対象外などをすべきだが、
    シンプルに一番RSSIが高い車両を選ぶ。
    """
    def determine_active_link(
        self, 
        rssi_dict, 
        mission_complete_dict,
        geometry_info,
        current_active_idx,
        vehicle_names,
        current_time
    ):
        if not rssi_dict:
            return current_active_idx, None
            
        best_name = max(rssi_dict, key=rssi_dict.get)
        best_idx = vehicle_names.index(best_name)
        
        if best_idx != current_active_idx:
            old_name = vehicle_names[current_active_idx]
            msg = f'リンク切替: {old_name} → {best_name} (rssi_priority: RSSI={rssi_dict[best_name]:.1f} dBm)'
            return best_idx, msg
            
        return current_active_idx, None

class GeometricBeamPriorityStrategy(LinkSchedulingStrategy):
    """
    geometric_beam_priority ポリシー (案1):
    アンテナゲイン（角度の良さ）が閾値以上の車両を「ビーム幅内にいる」として優先対象とする。
    対象の中で「物理的な直線距離」が最も近い車両に通信権を与える。
    """
    def __init__(self, beam_gain_threshold: float):
        self.beam_gain_threshold = beam_gain_threshold

    def determine_active_link(
        self, 
        rssi_dict, 
        mission_complete_dict,
        geometry_info,
        current_active_idx,
        vehicle_names,
        current_time
    ):
        if not geometry_info:
            return current_active_idx, None

        in_beam_vehicles = []
        out_beam_vehicles = []

        for name, info in geometry_info.items():
            if not info.get('comm_active', False):
                continue
            dist = info.get('distance', float('inf'))
            gain = info.get('antenna_gain_e_plane', -999.0)
            
            if gain >= self.beam_gain_threshold:
                in_beam_vehicles.append((name, dist))
            else:
                out_beam_vehicles.append((name, dist))

        # ビーム内の車両があれば、その中で一番近いものを選択
        if in_beam_vehicles:
            in_beam_vehicles.sort(key=lambda x: x[1])
            best_name = in_beam_vehicles[0][0]
            best_dist = in_beam_vehicles[0][1]
        # いなければ、ビーム外でも一番近いものを選択（フェールセーフ）
        elif out_beam_vehicles:
            out_beam_vehicles.sort(key=lambda x: x[1])
            best_name = out_beam_vehicles[0][0]
            best_dist = out_beam_vehicles[0][1]
        else:
            return current_active_idx, None

        best_idx = vehicle_names.index(best_name)
        if best_idx != current_active_idx:
            old_name = vehicle_names[current_active_idx]
            msg = f'リンク切替: {old_name} → {best_name} (geometric_beam: dist={best_dist:.1f}m)'
            return best_idx, msg
            
        return current_active_idx, None

class GeometricWeightedStrategy(LinkSchedulingStrategy):
    """
    geometric_weighted ポリシー (案3):
    すべての通信可能車両に対し、距離の近さ(0~1)とゲインの良さ(0~1)を正規化し、
    設定した重みで足し合わせた総合スコアが高い車両にリンク権を与える。
    """
    def __init__(self, weight_distance: float, weight_angle: float):
        self.weight_distance = weight_distance
        self.weight_angle = weight_angle

    def determine_active_link(
        self, 
        rssi_dict, 
        mission_complete_dict,
        geometry_info,
        current_active_idx,
        vehicle_names,
        current_time
    ):
        if not geometry_info:
            return current_active_idx, None

        active_info = {n: i for n, i in geometry_info.items() if i.get('comm_active', False)}
        if not active_info:
            return current_active_idx, None

        distances = [i.get('distance', 0) for i in active_info.values()]
        gains = [i.get('antenna_gain_e_plane', 0) for i in active_info.values()]
        
        min_d, max_d = min(distances), max(distances)
        min_g, max_g = min(gains), max(gains)

        best_name = None
        best_score = -float('inf')

        for name, info in active_info.items():
            d = info.get('distance', 0)
            g = info.get('antenna_gain_e_plane', 0)

            # 距離正規化: 近い方が1.0、遠い方が0.0
            norm_d = (max_d - d) / (max_d - min_d) if max_d > min_d else 1.0
            
            # ゲイン正規化: 高い方が1.0、低い方が0.0
            norm_g = (g - min_g) / (max_g - min_g) if max_g > min_g else 1.0

            score = self.weight_distance * norm_d + self.weight_angle * norm_g
            if score > best_score:
                best_score = score
                best_name = name

        if best_name is None:
            return current_active_idx, None

        best_idx = vehicle_names.index(best_name)
        if best_idx != current_active_idx:
            old_name = vehicle_names[current_active_idx]
            msg = f'リンク切替: {old_name} → {best_name} (geometric_weighted: score={best_score:.2f})'
            return best_idx, msg
            
        return current_active_idx, None
