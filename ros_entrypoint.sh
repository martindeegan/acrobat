#!/bin/bash
set -e

# setup ros2 environment
source "$ROS2_WS/install/setup.bash"
mkdir -p log_files
cd log_files
ros2 bag record -a &
cd ..
exec "$@"
