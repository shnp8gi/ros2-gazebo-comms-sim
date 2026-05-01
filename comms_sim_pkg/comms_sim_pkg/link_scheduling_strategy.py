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
    geometric_beam_priority ポリシー:
    
    地上局アンテナ角度・車両移動方向・車両アンテナ角度が形成する三角形において、
    地上局側の角度と車両側の角度がともに最小となる（アライメントが良い）車両を選択する。
    
    アンテナゲイン(E面/H面)はoff-boresight角の関数であり、ゲインが高い = 角度が小さい。
    したがって min(E面ゲイン, H面ゲイン) を最大化することで、
    両端の角度を最小化し、双方向で最もアライメントが良い車両を選ぶ。
    （パスロスによる距離の減衰は意図的に考慮せず、ビームへの入り具合のみを評価する）
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

        candidates = []

        for name, info in geometry_info.items():
            if not info.get('comm_active', True):
                continue
            
            e_gain = info.get('antenna_gain_e_plane', -999.0)
            h_gain = info.get('antenna_gain_h_plane', -999.0)
            dist = info.get('distance', float('inf'))
            
            if e_gain < self.beam_gain_threshold or h_gain < self.beam_gain_threshold:
                continue
                
            # アライメント品質 = 三角形の両端のうち悪い方の角度に対応するゲイン
            alignment_quality = min(e_gain, h_gain)
            
            candidates.append((name, alignment_quality, e_gain, h_gain, dist))

        if not candidates:
            return current_active_idx, None

        # アライメント品質が最も高い車両を選択
        # 同スコアの場合は距離が近い方を優先
        candidates.sort(key=lambda x: (-x[1], x[4]))
        best_name = candidates[0][0]
        best_quality = candidates[0][1]
        best_dist = candidates[0][4]

        best_idx = vehicle_names.index(best_name)
        if best_idx != current_active_idx:
            old_name = vehicle_names[current_active_idx]
            msg = (
                f'リンク切替: {old_name} → {best_name} '
                f'(alignment: min_gain={best_quality:.1f}dBi, dist={best_dist:.1f}m)'
            )
            return best_idx, msg
            
        return current_active_idx, None

class PhysicalScorePriorityStrategy(LinkSchedulingStrategy):
    """
    physical_score_priority ポリシー:
    
    基地局側で計算可能な幾何学情報（アンテナゲインと距離）を用いて、
    仮想的な受信電力をスコア化し、電波物理的に最適な車両を選択する。
    実システム（IEEE 802.15.3e PNC）で実現可能な方式。
    
    物理スコア = E面ゲイン(θ_BS) + H面ゲイン(θ_V) - パスロス(距離)
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

        candidates = []

        for name, info in geometry_info.items():
            if not info.get('comm_active', True):
                continue
            
            e_gain = info.get('antenna_gain_e_plane', -999.0)
            h_gain = info.get('antenna_gain_h_plane', -999.0)
            path_loss = info.get('path_loss', 999.0)
            dist = info.get('distance', float('inf'))
            
            if e_gain < self.beam_gain_threshold or h_gain < self.beam_gain_threshold:
                continue
            
            # 物理スコア: アンテナアライメントと距離の統合指標
            phys_score = e_gain + h_gain - path_loss
            
            candidates.append((name, phys_score, dist, e_gain, h_gain))

        if not candidates:
            return current_active_idx, None

        # 物理スコアが最も高い車両を選択
        candidates.sort(key=lambda x: -x[1])
        best_name = candidates[0][0]
        best_score = candidates[0][1]
        best_dist = candidates[0][2]

        best_idx = vehicle_names.index(best_name)
        if best_idx != current_active_idx:
            old_name = vehicle_names[current_active_idx]
            msg = (
                f'リンク切替: {old_name} → {best_name} '
                f'(phys_score={best_score:.1f}, dist={best_dist:.1f}m)'
            )
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

        active_info = {n: i for n, i in geometry_info.items() if i.get('comm_active', True)}
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
