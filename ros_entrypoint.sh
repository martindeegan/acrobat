#!/bin/bash
set -e

export LD_LIBRARY_PATH="/usr/local/lib:$LD_LIBRARY_PATH"

# setup ros2 environment
source "$ROS2_WS/install/setup.bash"

# Start rosbag logging
cd /logs
ros2 bag record -a &
cd ..

exec "$@"
