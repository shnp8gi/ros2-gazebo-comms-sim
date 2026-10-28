#!/bin/bash

# =============================================================================
# 開発用コンテナを起動する（GPU利用だけを判定）
# シミュレーションは tools/start_headless.sh / tools/start_gui.sh で起動する。
# =============================================================================

set -euo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$repo_root"

echo "NVIDIA GPUの利用可否を確認します。"

if command -v nvidia-smi &> /dev/null && nvidia-smi &> /dev/null; then
    echo "NVIDIA GPUを利用する開発用コンテナを起動します。"
    # ベース設定とGPU設定を両方読み込んで起動
    if ! docker compose -f docker-compose.yml -f docker-compose.gpu.yml up -d sim; then
        echo "GPUを利用するコンテナの起動に失敗しました。"
        echo "nvidia-container-toolkitの導入・設定を確認してください。"
        echo "CPUのみで起動し直します。"
        docker compose -f docker-compose.yml up -d sim
    fi
else
    echo "CPUのみの開発用コンテナを起動します。"
    # ベース設定のみで起動
    docker compose -f docker-compose.yml up -d sim
fi

echo "コンテナを起動しました。入るには docker compose -f docker-compose.yml exec sim bash を実行してください。"
echo "シミュレーションのGUI有無は、ros2 launch の headless:=true/false で指定します。"
