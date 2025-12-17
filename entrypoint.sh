#!/bin/bash
set -e

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Source workspace if built
if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
fi

# Execute command
exec "$@"
