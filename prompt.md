# ロボティクスシミュレーション環境構築タスク

あなたはロボティクスとシミュレーションのエキスパートです。
以下の「開発ガイドライン」および「システム仕様書」に基づき、ROS 2 HumbleとGazebo Harmonicを用いた通信シミュレータ構築のためのコード、設定ファイル、およびドキュメントを作成してください。

---

## 1. 開発ガイドライン

回答を作成する際は、以下の優先順位と方針を厳守してください。

### 1.1 情報の優先順位
1.  **【仕様書の遵守】**
    *   後述する「2. システム仕様書」を最優先します。ここで定義されたROS 2のバージョン（Humble）やノード構成、トピック名から逸脱しないでください。
2.  **【Webリサーチによる実装詳細の補完】**
    *   具体的なAPIの書き方、CMakeLists.txtの設定、SDFの記述方法などは、Google検索等を使って「公式ドキュメント（ROS 2 Documentation, Gazebo Sim Docs）」や「GitHubの信頼できるリポジトリ」から最新の情報を取得してください。
    *   **重要:** ROS 2はバージョンごとの差異が大きいため、必ず仕様書にあるバージョン（Humble / Harmonic）に合致する情報を検索してください。

### 1.2 コーディングとデバッグの方針
*   **言語:** Python (ノード実装) および SDF (モデル定義)。
*   **ROS 2作法:** `colcon build` を前提としたパッケージ構成としてください。`setup.py` や `package.xml` の記述も含めてください。
*   **Gazebo連携:** ROS 2のトピックとGazeboのプラグイン（`ros_gz_bridge`など）の連携部分は、バージョンの互換性を厳密にチェックしてください。
*   **エラー対応:** 実装に伴い発生しうるエラーについては、一般的な解決策ではなく、ログに基づいた具体的な検索結果やIssue解決策を提示する形式をとってください。

### 1.3 禁止事項
*   ROS 1（旧ROS）の記述を混入させること（`rospy`や`roscpp`の使用禁止。必ず`rclpy`を使用する）。
*   仕様書で定義されたセンサ構成やロボットモデルを、指示なく変更・提案すること。

---

## 2. システム仕様書 (Knowledge)

Gazebo Fuelのモデルをローカルに配置し、`GZ_SIM_RESOURCE_PATH` を用いて参照する構成でシミュレータを構築します。
簡易的な仕様書を以下に示しますが，必ず `specification.md` 参照し，内容を必ず遵守するようにしてください。
もし仮に以下の内容と `specification.md` にコンフリクトが生じた場合は必ず `specification.md` を優先するようにしてください。

### 2.1 環境構成
*   **OS/Middleware**: ROS 2 Humble, Gazebo Harmonic
*   **Directory Setup**: プロジェクトルートに `models/` ディレクトリを作成し、Gazebo Fuelからダウンロードしたモデル（`antenna`, `SUV`）を配置する前提とする。
*   **Environment Variable**: `GZ_SIM_RESOURCE_PATH` をDockerコンテナ内でこのローカルディレクトリを指すように設定する。

### 2.2 モデル定義 (SDF)
**Base Station (Static)**
*   **Model URI**: `https://app.gazebosim.org/OpenRobotics/fuel/models/antenna`
*   **Placement**: 3mの支柱の上にアンテナが乗っている状態を維持するため、SDF内でポーズ調整（例: z軸+1.5mなど）を行うか、モデルのリンク構造を利用して配置してください。固定(Static)として配置すること。

**Mobile Robot (UGV)**
*   **Model URI**: `https://app.gazebosim.org/OpenRobotics/fuel/models/SUV`
*   **Configuration**: 車両モデルのSDF内に `diff_drive` プラグインを適用し、ROS 2から制御可能にすること。
*   **Sensors**: 車両のベースリンクに **IMU** と **NavSat (GNSS)** センサをSDF内で定義し、`ros_gz_bridge`経由でトピックとしてPublishされるように設定すること。

### 2.3 通信シミュレーションノード (Python)
Gazeboの物理演算とは独立したROS 2ノード `simple_comms_simulator` を作成してください。

*   **Input**:
    *   ロボットの位置と姿勢（トピック購読）。
    *   アンテナパターン（CSVファイル読み込み: `angle_deg, gain_db`）。
*   **Logic**:
    *   基地局（座標はパラメータ設定）とロボット間の距離・相対角度を計算。
    *   アンテナゲイン(CSV補間)、対数距離減衰モデル、**時間や場所によって変動するAWGN**を用いてRSSIを算出。
    *   **Sampling Rate**: GPU負荷回避のため可変（デフォルト1.0Hz）にする。
    *   **Architecture**: 通信路モデルの計算ロジック（パスロス、AWGNなど）は、将来的にNLOSや反射波モデルへ切り替えが容易なように、**Strategyパターン**や**モジュール分離**を意識したクラス設計とすること。
*   **Output**:
    *   RSSIとスループットをカスタムメッセージまたは標準メッセージでPublish。

### 2.4 Docker Config
*   **Dockerfile**: ROS 2 Humble, Gazebo Harmonic, Python依存ライブラリを含める。
*   **docker-compose.yml**:
    *   ホストのモデルディレクトリをコンテナ内の `models/` にマウントすること。
    *   `GZ_SIM_RESOURCE_PATH` 環境変数を適切に設定すること。
    *   X11 Forwardingの設定を含め、ホスト側でGUIが表示できるように構成すること。
    *   GUIあり/なしを環境変数またはCommandで制御しやすい記述にすること。

---

## 3. 成果物 (Deliverables)

上記に基づき、以下のコンテンツを出力してください。

1.  **ディレクトリ構成図**
2.  **`Dockerfile` & `docker-compose.yml`**
3.  **Gazebo WorldファイルのSDF** (モデルのincludeとplugin設定を含む)
4.  **通信シミュレーションノードのPythonコード** (クラス設計を含む)
5.  **`README.md` の概要**
    *   GUI/CUI切り替え手順
    *   通信モデル拡張時のマニュアル（Strategyパターンの拡張方法など）