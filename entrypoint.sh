#!/bin/bash
set -e

# Disable FastDDS Shared Memory transport to eliminate RTPS_TRANSPORT_SHM segment errors
export FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/config/fastdds_no_shm.xml

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Source workspace if built
if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
fi

# Execute command
exec "$@"
