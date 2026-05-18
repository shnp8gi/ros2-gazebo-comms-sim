#!/bin/bash

# =============================================================================
# 起動スクリプト（自動GPU/CPUモード判定）
# =============================================================================

# Xサーバーへのアクセスを許可（DISPLAY が設定されている場合のみ）
if [ -n "$DISPLAY" ]; then
    xhost +local:root > /dev/null 2>&1 || true
fi

echo "Checking system for NVIDIA GPU..."

if command -v nvidia-smi &> /dev/null && nvidia-smi &> /dev/null; then
    echo "✅ NVIDIA GPU detected. Starting simulation WITH hardware acceleration..."
    # ベース設定とGPU設定を両方読み込んで起動
    docker compose -f docker-compose.yml -f docker-compose.gpu.yml up -d sim
else
    echo "⚠️  No NVIDIA GPU detected. Starting simulation with CPU only..."
    # ベース設定のみで起動
    docker compose -f docker-compose.yml up -d sim
fi

# DISPLAY の状況を表示
if [ -z "$DISPLAY" ]; then
    echo "ℹ️  DISPLAY is not set (remote/headless connection). Gazebo will run in headless mode automatically."
else
    echo "ℹ️  DISPLAY=${DISPLAY} is set. Gazebo GUI will use GPU (ogre2) rendering."
fi

echo "Container started. Run 'docker compose exec sim bash' to enter."
