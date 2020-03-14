#!/bin/bash
set -e

# setup ros2 environment
source "$ROS2_WS/install/setup.bash"

# Start rosbag logging
cd /logs
ros2 bag record -a &
cd ..

exec "$@"
