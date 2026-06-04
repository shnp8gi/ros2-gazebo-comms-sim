#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
# 通信品質計算モジュール
# Strategyパターンによる伝搬路モデルの実装
# =============================================================================
"""
通信品質（RSSI・スループット）を計算するモジュール。
Strategyパターンにより伝搬路モデルを差し替え可能にしている。
"""

from abc import ABC, abstractmethod
from typing import Tuple, Optional, List
import numpy as np
import csv
import os


# =============================================================================
# Strategyインターフェース: 伝搬路モデル
# =============================================================================
class PropagationModel(ABC):
    """
    伝搬路モデルの抽象基底クラス（Strategyパターン）。

    新しい伝搬路モデル（NLOS、反射波モデル等）を追加する場合、
    このインターフェースを実装する。
    """

    @abstractmethod
    def calculate_path_loss(
        self,
        distance: float,
        frequency_ghz: float = 60.0
    ) -> float:
        """
        指定距離でのパスロスを計算する。

        Args:
            distance: 送受信間距離 [m]
            frequency_ghz: 使用周波数 [GHz]

        Returns:
            パスロス [dB]
        """
        pass

    @property
    @abstractmethod
    def model_name(self) -> str:
        """伝搬路モデルの名称を返す。"""
        pass


# =============================================================================
# 具象Strategy: 対数距離減衰モデル
# =============================================================================
class LogDistancePathLossModel(PropagationModel):
    """
    対数距離減衰モデル (Log-Distance Path Loss Model)。

    PL(d) = PL(d₀) + 10 × n × log₁₀(d / d₀)

    各変数:
        PL(d₀): 基準距離d₀でのパスロス [dB]
        n: パスロス指数
        d₀: 基準距離 [m]（通常1m）
        d: 距離 [m]

    パスロス指数 n の典型値:
        - 自由空間: 2.0
        - 都市部: 2.7 - 3.5
        - 屋内LOS: 1.6 - 1.8
        - 屋内NLOS: 4 - 6
    """

    def __init__(
        self,
        frequency: float = 6.0e10,
        c: float = 299792458,
        exponent: float = 2.0,
        d0: float = 1.0,
        pl_d0: float = -1.0,
    ) -> None:
        """
        対数距離減衰モデルを初期化する。

        Args:
            frequency: 使用周波数 [Hz]（pl_d0自動計算用）
            c: 光速 [m/s]
            exponent: パスロス指数 (n)
            d0: 基準距離 [m]
            pl_d0: 基準距離でのパスロス [dB]
                   -1.0の場合は自由空間理論値 20log₁₀(4πd₀/λ) を自動計算
        """
        self.exponent = exponent
        self.frequency = frequency
        self.c = c
        self.d0 = d0

        if pl_d0 < 0:
            # 自由空間理論値を自動計算
            wavelength = c / frequency
            self.pl_d0 = 20.0 * np.log10(4.0 * np.pi * d0 / wavelength)
        else:
            self.pl_d0 = pl_d0

    def calculate_path_loss(
        self,
        distance: float,
        frequency_ghz: float = None,
    ) -> float:
        """
        対数距離モデルによるパスロスを計算する。

        PL(d) = PL(d₀) + 10 × n × log₁₀(d / d₀)

        Args:
            distance: 距離 [m]
            frequency_ghz: 未使用（インターフェース互換のため残す）

        Returns:
            パスロス [dB]
        """
        # 距離が基準距離以下の場合のガード
        if distance <= self.d0:
            return self.pl_d0

        path_loss = self.pl_d0 + 10.0 * float(self.exponent) * np.log10(float(distance) / self.d0)
        return float(path_loss)

    @property
    def model_name(self) -> str:
        return "Log-Distance Path Loss Model"


# =============================================================================
# 具象Strategy: 二波モデル（地面反射モデル）
# =============================================================================
class TwoRayGroundModel(PropagationModel):
    """
    二波モデル（Two-Ray Ground Reflection Model）。

    地面反射が顕著になる長距離通信に適用。

    PL(d) = 40 * log10(d) - 10 * log10(Gt * Gr * ht^2 * hr^2)

    各変数:
        d: 距離 [m]
        Gt, Gr: アンテナゲイン（リニア値）
        ht, hr: アンテナ高さ [m]
    """

    def __init__(
        self,
        tx_height: float = 10.5,
        rx_height: float = 1.3,
        tx_gain_db: float = 0.0,
        rx_gain_db: float = 0.0
    ) -> None:
        """
        二波モデルを初期化する。

        Args:
            tx_height: 送信アンテナ高さ [m]
            rx_height: 受信アンテナ高さ [m]
            tx_gain_db: 送信アンテナゲイン [dBi]
            rx_gain_db: 受信アンテナゲイン [dBi]
        """
        self.tx_height = tx_height
        self.rx_height = rx_height
        # dBからリニア値へ変換
        self.tx_gain_linear = 10 ** (tx_gain_db / 10)
        self.rx_gain_linear = 10 ** (rx_gain_db / 10)

    def calculate_path_loss(
        self,
        distance: float,
        frequency: float = 6.0e10,
        c: float = 299792458,
    ) -> float:
        """
        二波モデルによるパスロスを計算する。

        Args:
            distance: 距離 [m]
            frequency: 使用周波数 [Hz]
            c: 光速 [m/s]

        Returns:
            パスロス [dB]
        """
        if distance <= 0:
            return 0.0

        wavelength = c / frequency

        # クロスオーバー距離: 自由空間モデルと二波モデルの切り替え点
        crossover = (4 * np.pi * self.tx_height * self.rx_height) / wavelength

        if distance < crossover:
            # クロスオーバー距離未満: 自由空間パスロス (FSPL) を使用
            fspl = 20.0 * np.log10(4.0 * np.pi * float(distance) / wavelength)
            return float(fspl)

        # クロスオーバー距離以上: 二波モデルを使用
        gain_term = 10.0 * np.log10(
            self.tx_gain_linear * self.rx_gain_linear *
            (self.tx_height ** 2) * (self.rx_height ** 2)
        )
        path_loss = 40.0 * np.log10(float(distance)) - gain_term
        return float(max(path_loss, 0.0))

    @property
    def model_name(self) -> str:
        return "Two-Ray Ground Reflection Model"


# =============================================================================
# 通信品質計算クラス（Contextクラス）
# =============================================================================
class CommsCalculator:
    """
    通信品質計算クラス。

    伝搬路モデル・アンテナゲイン・雑音に基づいて
    RSSIとスループットを計算する。
    Strategyパターンにより伝搬路モデルの差し替えが可能。
    """

    def __init__(
        self,
        propagation_model: Optional[PropagationModel] = None,
        tx_power_dbm: float = 20.0,
        noise_variance: float = 2.0,
        mcs_table_path: Optional[str] = None
    ) -> None:
        """
        通信品質計算クラスを初期化する。

        Args:
            propagation_model: 伝搬路モデル（デフォルト: 対数距離減衰）
            tx_power_dbm: 送信電力 [dBm]
            noise_variance: AWGN分散 [dB]
            mcs_table_path: MCSテーブルCSVファイルのパス
        """
        self.propagation_model = propagation_model or LogDistancePathLossModel()
        self.tx_power_dbm = tx_power_dbm
        self.noise_variance = noise_variance
        self._rng = np.random.default_rng()

        # MCSテーブル
        self.mcs_rssi: List[float] = []
        self.mcs_throughput: List[float] = []

        # RSSI閾値（MCSテーブルから設定される）
        self.rssi_min: float = 0.0
        self.rssi_max: float = 0.0

        if mcs_table_path:
            self._load_mcs_table(mcs_table_path)
        else:
            # MCSテーブル未指定時のデフォルト値
            self._init_default_mcs_table()

        # 読み込んだMCSテーブルからRSSI閾値を設定
        if self.mcs_rssi:
            self.rssi_min = min(self.mcs_rssi)
            self.rssi_max = max(self.mcs_rssi)
        else:
            raise ValueError("MCSテーブルが空です。RSSI閾値を決定できません。")

    def _load_mcs_table(self, csv_path: str) -> None:
        """
        CSVファイルからMCSテーブルを読み込む。

        CSVフォーマット:
            RSSI [dBm], Throughput [Gbps]
            -50.5, 6.000
            -54, 4.600
            ...

        Args:
            csv_path: MCSテーブルCSVファイルのパス

        Raises:
            FileNotFoundError: CSVファイルが存在しない場合
            ValueError: CSVフォーマットが不正な場合
        """
        if not os.path.exists(csv_path):
            raise FileNotFoundError(
                f"\n{'='*70}\n"
                f"エラー: MCSテーブルファイルが見つかりません\n"
                f"{'='*70}\n"
                f"パス: {csv_path}\n"
                f"configディレクトリにMCStable.csvが存在するか確認してください。\n"
                f"{'='*70}\n"
            )

        rssi_list = []
        throughput_list = []

        try:
            with open(csv_path, 'r', encoding='utf-8') as f:
                reader = csv.reader(f)
                for row in reader:
                    # 空行とコメント行をスキップ
                    if not row or row[0].strip().startswith(('#', '//')):
                        continue

                    # 列数不足の行をスキップ
                    if len(row) < 2:
                        continue

                    try:
                        rssi = float(row[0].strip())
                        throughput = float(row[1].strip())
                        rssi_list.append(rssi)
                        throughput_list.append(throughput)
                    except ValueError:
                        continue

            if not rssi_list:
                raise ValueError("MCSテーブルに有効なデータがありません")

            # RSSI昇順でソート（searchsorted用）
            sorted_pairs = sorted(zip(rssi_list, throughput_list))
            self.mcs_rssi = [pair[0] for pair in sorted_pairs]
            self.mcs_throughput = [pair[1] for pair in sorted_pairs]

        except Exception as e:
            raise ValueError(
                f"\n{'='*70}\n"
                f"エラー: MCSテーブルの読み込みに失敗しました\n"
                f"{'='*70}\n"
                f"ファイル: {csv_path}\n"
                f"エラー: {e}\n"
                f"{'='*70}\n"
            )

    def _init_default_mcs_table(self) -> None:
        """デフォルトMCSテーブルを初期化する（フォールバック用）。"""
        self.mcs_rssi = [-61, -58, -55, -51, -45, -39]
        self.mcs_throughput = [2.5813, 3.2853, 5.1627, 6.5707, 9.856, 13.1413]

    def set_propagation_model(self, model: PropagationModel) -> None:
        """
        伝搬路モデルを設定する（Strategyパターン）。

        Args:
            model: 新しい伝搬路モデル
        """
        self.propagation_model = model

    def set_noise_variance(self, variance: float) -> None:
        """
        AWGN分散を更新する（動的雑音制御用）。

        Args:
            variance: 新しい雑音分散 [dB]
        """
        self.noise_variance = variance

    def calculate_distance(
        self,
        tx_position: np.ndarray,
        rx_position: np.ndarray
    ) -> float:
        """
        3Dユークリッド距離を計算する。

        Args:
            tx_position: TX位置 [x, y, z]
            rx_position: 基地局位置 [x, y, z]

        Returns:
            距離 [m]
        """
        return float(np.linalg.norm(tx_position - rx_position))

    def calculate_rssi(
        self,
        distance: float,
        antenna_gain_db: float = 0.0,
        add_noise: bool = True
    ) -> Tuple[float, float]:
        """
        RSSIを計算する（オプションでAWGNによる劣化を付加）。

        RSSI = 送信電力 - パスロス + アンテナゲイン - 雑音劣化

        劣化量は半正規分布 |N(0, σ)| に従う。

        Args:
            distance: 距離 [m]
            antenna_gain_db: 合成アンテナゲイン [dBi]
            add_noise: AWGN劣化を付加するかどうか

        Returns:
            (RSSI [dBm], パスロス [dB]) のタプル
        """
        # パスロス計算
        path_loss = self.propagation_model.calculate_path_loss(distance)

        # ベースRSSI
        rssi = self.tx_power_dbm - path_loss + antenna_gain_db

        # AWGNによる劣化
        if add_noise and self.noise_variance > 0:
            noise_degradation = abs(self._rng.normal(0, np.sqrt(self.noise_variance)))
            rssi -= noise_degradation

        return rssi, path_loss

    def calculate_throughput(self, rssi: float) -> float:
        """
        MCSテーブルのステップ関数によりスループットを決定する。

        RSSIが各MCSレベルの閾値を満たす最大レベルのスループットを適用。
        MCSレベル間の補間は行わない（ステップ関数方式）。

        ルール:
        - RSSI <= rssi_min: 0 Gbps（通信不可）
        - rssi_min < RSSI < rssi_max: ステップ関数（MCSテーブル参照）
        - RSSI >= rssi_max: 最大スループット（飽和）

        Args:
            rssi: 受信信号強度 [dBm]

        Returns:
            スループット [Gbps]
        """
        # 最低閾値未満: 通信不可
        if rssi <= self.rssi_min:
            return 0.0

        # 最大閾値以上: 最大スループットで飽和
        if rssi >= self.rssi_max:
            return self.mcs_throughput[-1]

        # ステップ関数: RSSI閾値を満たす最大のMCSレベルを検索
        # mcs_rssi は昇順ソート済み;
        # searchsorted(side='right') - 1 で rssi 以下の最大インデックスを取得
        idx = int(np.searchsorted(self.mcs_rssi, rssi, side='right')) - 1
        return float(self.mcs_throughput[idx])

    def calculate_all(
        self,
        tx_position: np.ndarray,
        rx_position: np.ndarray,
        antenna_gain_db: float = 0.0,
        add_noise: bool = True
    ) -> dict:
        """
        全通信メトリクスを一括計算する。

        Args:
            tx_position: TX位置 [x, y, z]
            rx_position: 基地局位置 [x, y, z]
            antenna_gain_db: 合成アンテナゲイン [dBi]
            add_noise: AWGNを付加するかどうか

        Returns:
            全メトリクスを含む辞書
        """
        distance = self.calculate_distance(tx_position, rx_position)
        rssi, path_loss = self.calculate_rssi(distance, antenna_gain_db, add_noise)
        throughput = self.calculate_throughput(rssi)

        return {
            'distance': distance,
            'rssi': rssi,
            'path_loss': path_loss,
            'throughput': throughput,
            'model_name': self.propagation_model.model_name
        }
