#!/bin/bash

# Xサーバーへのアクセスを許可（ローカル）
xhost +local:root > /dev/null

echo "Checking system for NVIDIA GPU..."

# nvidia-smiコマンドが存在し、実行可能かチェック
if command -v nvidia-smi &> /dev/null; then
    echo "✅ NVIDIA GPU detected. Starting simulation WITH hardware acceleration..."
    # ベース設定とGPU設定を両方読み込んで起動
    docker compose -f docker-compose.yml -f docker-compose.gpu.yml up -d sim
else
    echo "⚠️ No NVIDIA GPU detected. Starting simulation with CPU only..."
    # ベース設定のみで起動
    docker compose -f docker-compose.yml up -d sim
fi

echo "Container started. Run 'docker compose exec sim bash' to enter."
