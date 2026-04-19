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
        current_active_idx: int,
        vehicle_names: List[str],
        current_time: float
    ) -> Tuple[int, Optional[str]]:
        """
        次に有効なリンク権を持つべき車両のインデックスを決定する。
        
        Args:
            rssi_dict: 各車両の最新RSSI
            mission_complete_dict: 各車両のミッション完了状態
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
