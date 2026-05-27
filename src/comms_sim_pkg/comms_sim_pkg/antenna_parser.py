#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
# アンテナパターン解析モジュール
# E面・H面アンテナゲインCSVファイルの読み込みと補間
# =============================================================================
"""
通信シミュレーション用アンテナパターン解析モジュール。
CSVファイルからアンテナゲインパターンを読み込み、
任意の角度に対する補間ゲイン値を提供する。
"""

from typing import Tuple, Optional
import numpy as np
from scipy.interpolate import interp1d


class AntennaPatternParser:
    """
    アンテナ放射パターンCSVファイルの解析クラス。

    E面（垂直面）・H面（水平面）パターンに基づいて、
    角度からゲイン値を線形補間により取得する。

    Attributes:
        e_plane_interp: E面ゲインの補間関数
        h_plane_interp: H面ゲインの補間関数
    """

    def __init__(
        self,
        e_plane_path: Optional[str] = None,
        h_plane_path: Optional[str] = None,
        max_antenna_attenuation: float = 30.0
    ) -> None:
        """
        アンテナパターン解析クラスを初期化する。

        Args:
            e_plane_path: E面ゲインCSVファイルのパス
            h_plane_path: H面ゲインCSVファイルのパス
            max_antenna_attenuation: 各面の最大減衰量 [dB]。
                パターン範囲外等で極端に低いゲイン値が得られた場合に
                減衰をこの値でクランプする。旧C++実装と同等。
        """
        self.e_plane_interp: Optional[interp1d] = None
        self.h_plane_interp: Optional[interp1d] = None
        self.e_plane_peak: float = 0.0
        self.h_plane_peak: float = 0.0
        self.max_antenna_attenuation: float = max_antenna_attenuation

        if e_plane_path:
            self.load_e_plane(e_plane_path)
        if h_plane_path:
            self.load_h_plane(h_plane_path)

    def _load_pattern(self, filepath: str) -> Tuple[interp1d, float]:
        """
        CSVファイルからアンテナパターンを読み込む。

        想定されるCSVフォーマット:
        - 2列: angle,gain（ヘッダ行はオプション）
        - 角度は度数（例: -90 ～ +90）、ゲインは dBi
        - '#' で始まる行はコメントとして扱う
        - 0.1度刻み（-90 ～ +90 で1801行）

        Args:
            filepath: CSVファイルのパス

        Returns:
            (補間関数, ピークゲイン [dBi]) のタプル

        Raises:
            FileNotFoundError: ファイルが存在しない場合
            ValueError: ファイルフォーマットが不正な場合
        """
        angles = []
        gains = []

        with open(filepath, 'r', encoding='utf-8') as f:
            header_found = False
            for line in f:
                line = line.strip()

                # 空行とコメント行をスキップ
                if not line or line.startswith('#'):
                    continue

                # ヘッダ行をスキップ
                if not header_found and 'angle' in line.lower():
                    header_found = True
                    continue

                # データ行の解析
                parts = line.split(',')
                if len(parts) >= 2:
                    try:
                        angle = float(parts[0].strip())
                        # 全角マイナス記号の対応
                        gain_str = parts[1].strip().replace('−', '-')
                        gain = float(gain_str)
                        angles.append(angle)
                        gains.append(gain)
                    except ValueError:
                        continue

        if not angles:
            raise ValueError(f"有効なデータが見つかりません: {filepath}")

        # numpy配列に変換
        angles = np.array(angles)
        gains = np.array(gains)

        # 角度でソート
        sort_idx = np.argsort(angles)
        angles = angles[sort_idx]
        gains = gains[sort_idx]

        peak_gain = float(np.max(gains))

        # 補間関数を作成
        # 範囲外の角度には非常に大きな負のゲイン (-1000 dBi) を割り当て、
        # アンテナパターンの角度範囲外では事実上通信不可能であることを表現。
        interp_func = interp1d(
            angles, gains,
            kind='linear',
            bounds_error=False,
            fill_value=-1000.0
        )
        return interp_func, peak_gain

    def load_e_plane(self, filepath: str) -> None:
        """
        E面（仰角方向）アンテナパターンを読み込む。

        Args:
            filepath: E面CSVファイルのパス
        """
        interp, peak = self._load_pattern(filepath)
        self.e_plane_interp = interp
        self.e_plane_peak = peak

    def load_h_plane(self, filepath: str) -> None:
        """
        H面（方位角方向）アンテナパターンを読み込む。

        Args:
            filepath: H面CSVファイルのパス
        """
        interp, peak = self._load_pattern(filepath)
        self.h_plane_interp = interp
        self.h_plane_peak = peak

    def get_e_plane_gain(self, angle_deg: float) -> float:
        """
        指定角度でのE面アンテナゲインを取得する。

        Args:
            angle_deg: 角度 [度] (-180 ～ 180)

        Returns:
            ゲイン [dBi]
        """
        if self.e_plane_interp is None:
            return 0.0
        return float(self.e_plane_interp(angle_deg))

    def get_h_plane_gain(self, angle_deg: float) -> float:
        """
        指定角度でのH面アンテナゲインを取得する。

        Args:
            angle_deg: 角度 [度] (-180 ～ 180)

        Returns:
            ゲイン [dBi]
        """
        if self.h_plane_interp is None:
            return 0.0
        return float(self.h_plane_interp(angle_deg))

    def get_combined_gain(
        self,
        elevation_deg: float,
        azimuth_deg: float
    ) -> Tuple[float, float, float]:
        """
        E面（仰角）・H面（方位角）の合成ゲインを取得する。

        Args:
            elevation_deg: 仰角 [度]（E面）
            azimuth_deg: 方位角 [度]（H面）

        Returns:
            (E面ゲイン, H面ゲイン, 合成ゲイン) [dBi] のタプル
        """
        e_gain = self.get_e_plane_gain(elevation_deg)
        h_gain = self.get_h_plane_gain(azimuth_deg)

        # 合成ゲイン（簡易モデル: dB領域での加算）
        # より正確なモデルでは3Dパターンデータを使用する
        combined = e_gain + h_gain

        return e_gain, h_gain, combined

    @staticmethod
    def _rpy_to_rotmat(roll: float, pitch: float, yaw: float) -> np.ndarray:
        """回転行列 R = Rz(yaw) * Ry(pitch) * Rx(roll) を返す。

        アンテナ/ボディフレーム → ワールドフレームへの変換に使用。
        """
        cr, sr = float(np.cos(roll)), float(np.sin(roll))
        cp, sp = float(np.cos(pitch)), float(np.sin(pitch))
        cy, sy = float(np.cos(yaw)), float(np.sin(yaw))

        rx = np.array(
            [[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]],
            dtype=float,
        )
        ry = np.array(
            [[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]],
            dtype=float,
        )
        rz = np.array(
            [[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]],
            dtype=float,
        )
        return rz @ ry @ rx

    @staticmethod
    def _wrap_pi(angle_rad: float) -> float:
        """角度を [-π, π] の範囲に正規化する。"""
        return float((angle_rad + np.pi) % (2.0 * np.pi) - np.pi)

    def calculate_antenna_frame_angles(
        self,
        antenna_pos_world: np.ndarray,
        target_pos_world: np.ndarray,
        antenna_rpy_world: np.ndarray,
    ) -> Tuple[float, float]:
        """アンテナフレームにおけるターゲット方向の (仰角, 方位角) を計算する。

        - 仰角: 上方向が正 (asin(z))
        - 方位角: atan2(y, x)

        antenna_rpy_world はアンテナフレームのワールド座標系に対する姿勢。
        """
        # ワールド座標系での方向ベクトル
        v_world = np.asarray(target_pos_world, dtype=float) - np.asarray(antenna_pos_world, dtype=float)
        norm = float(np.linalg.norm(v_world))
        if norm <= 1e-12:
            return 0.0, 0.0

        # 単位ベクトルに正規化
        v_world /= norm

        # 回転行列を生成
        r = self._rpy_to_rotmat(
            float(antenna_rpy_world[0]),
            float(antenna_rpy_world[1]),
            float(antenna_rpy_world[2]),
        )

        # ワールド → アンテナフレーム: 逆回転（転置）
        v_ant = r.T @ v_world

        az = float(np.arctan2(v_ant[1], v_ant[0]))
        el = float(np.arcsin(np.clip(v_ant[2], -1.0, 1.0)))
        return el, self._wrap_pi(az)

    def get_gain_from_angles(self, elevation_rad: float, azimuth_rad: float) -> Tuple[float, float, float]:
        """アンテナフレーム角度 [ラジアン] から (E面/H面/合成) ゲインを取得する。

        合成ゲインは旧C++実装と同等の3Dパターン近似で推定:
            各面の減衰 = g_peak - G_plane （max_antenna_attenuation でクランプ）
            G(el, az) = g_peak - atten_E - atten_H  （g_peak - max_attenuation を下限）

        ボアサイトではピークゲインに等しく、オフアクシスでは各面のロールオフが寄与する。
        パターン範囲外で極端に低い値が得られた場合でも、減衰は max_antenna_attenuation で
        クランプされるため RSSI が破綻しない。
        """
        elevation_deg = float(np.degrees(elevation_rad))
        azimuth_deg = float(np.degrees(azimuth_rad))

        e_gain = float(self.get_e_plane_gain(elevation_deg))
        h_gain = float(self.get_h_plane_gain(azimuth_deg))
        g_peak = max(self.e_plane_peak, self.h_plane_peak)

        # 各面の減衰量を算出し、max_antenna_attenuation でクランプ
        e_atten = min(g_peak - e_gain, self.max_antenna_attenuation)
        h_atten = min(g_peak - h_gain, self.max_antenna_attenuation)

        # 合成ゲイン = ピーク - E面減衰 - H面減衰
        total = g_peak - e_atten - h_atten

        # 合成ゲインの下限もクランプ
        min_gain = g_peak - self.max_antenna_attenuation
        total = max(total, min_gain)

        return e_gain, h_gain, total

    def get_tx_rx_gains(
        self,
        tx_pos_world: np.ndarray,
        tx_rpy_world: np.ndarray,
        rx_pos_world: np.ndarray,
        rx_rpy_world: np.ndarray,
    ) -> Tuple[float, float, float, float, float, float]:
        """送信側・受信側のアンテナゲインを個別に計算する。

        Returns:
            (tx_e, tx_h, tx_total, rx_e, rx_h, rx_total) [dB]
        """
        # 送信側: アンテナから受信側方向の角度を計算
        tx_el, tx_az = self.calculate_antenna_frame_angles(tx_pos_world, rx_pos_world, tx_rpy_world)
        # 受信側: アンテナから送信側方向の角度を計算
        rx_el, rx_az = self.calculate_antenna_frame_angles(rx_pos_world, tx_pos_world, rx_rpy_world)

        tx_e, tx_h, tx_total = self.get_gain_from_angles(tx_el, tx_az)
        rx_e, rx_h, rx_total = self.get_gain_from_angles(rx_el, rx_az)

        return tx_e, tx_h, tx_total, rx_e, rx_h, rx_total

    def calculate_angles_from_orientation(
        self,
        ugv_position: np.ndarray,
        base_station_position: np.ndarray,
        ugv_orientation_euler: np.ndarray
    ) -> Tuple[float, float]:
        """後方互換API。

        注意: この旧ヘルパーはyawのみで方位角補正を行い、
        roll/pitchは完全には反映しない。
        `calculate_antenna_frame_angles()` の使用を推奨。
        """
        # UGVから基地局への方向ベクトル
        direction = base_station_position - ugv_position

        # XY平面での水平距離
        horizontal_dist = np.sqrt(direction[0]**2 + direction[1]**2)

        # 仰角（基地局への垂直角度）
        elevation_rad = np.arctan2(direction[2], horizontal_dist)

        # 方位角（基地局への水平角度）
        azimuth_to_bs = np.arctan2(direction[1], direction[0])

        # UGVのyawを考慮した相対方位角
        yaw = ugv_orientation_euler[2]
        relative_azimuth = azimuth_to_bs - yaw

        # -180 ～ 180度に正規化
        elevation_deg = np.degrees(elevation_rad)
        azimuth_deg = np.degrees(relative_azimuth)

        # 方位角を -180 ～ 180 にラップ
        while azimuth_deg > 180:
            azimuth_deg -= 360
        while azimuth_deg < -180:
            azimuth_deg += 360

        return elevation_deg, azimuth_deg
