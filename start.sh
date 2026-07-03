#!/bin/bash

# =============================================================================
# 襍ｷ蜍輔せ繧ｯ繝ｪ繝励ヨ・郁・蜍膝PU/CPU繝｢繝ｼ繝牙愛螳夲ｼ・
# =============================================================================

# X繧ｵ繝ｼ繝舌・縺ｸ縺ｮ繧｢繧ｯ繧ｻ繧ｹ繧定ｨｱ蜿ｯ・・ISPLAY 縺瑚ｨｭ螳壹＆繧後※縺・ｋ蝣ｴ蜷医・縺ｿ・・
if [ -n "$DISPLAY" ]; then
    xhost +local:root > /dev/null 2>&1 || true
fi

echo "Checking system for NVIDIA GPU..."

if command -v nvidia-smi &> /dev/null && nvidia-smi &> /dev/null; then
    echo "笨・NVIDIA GPU detected. Starting simulation WITH hardware acceleration..."
    # 繝吶・繧ｹ險ｭ螳壹→GPU險ｭ螳壹ｒ荳｡譁ｹ隱ｭ縺ｿ霎ｼ繧薙〒襍ｷ蜍・
    if ! docker compose -f docker-compose.yml -f docker-compose.gpu.yml up -d sim; then
        echo "笞・・ Failed to start container with NVIDIA GPU acceleration."
        echo "   Please check if 'nvidia-container-toolkit' is installed and configured for Docker on this machine."
        echo "   Falling back to CPU mode..."
        docker compose -f docker-compose.yml up -d sim
    fi
else
    echo "笞・・ No NVIDIA GPU detected. Starting simulation with CPU only..."
    # 繝吶・繧ｹ險ｭ螳壹・縺ｿ縺ｧ襍ｷ蜍・
    docker compose -f docker-compose.yml up -d sim
fi

# DISPLAY 縺ｮ迥ｶ豕√ｒ陦ｨ遉ｺ
if [ -z "$DISPLAY" ]; then
    echo "邃ｹ・・ DISPLAY is not set (remote/headless connection). Gazebo will run in headless mode automatically."
else
    echo "邃ｹ・・ DISPLAY=${DISPLAY} is set. Gazebo GUI will use GPU (ogre2) rendering."
fi

echo "Container started. Run 'docker compose exec sim bash' to enter."
