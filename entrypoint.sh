#!/bin/bash
set -e

# Source ROS 2
source /opt/ros/humble/setup.bash || true

# Source workspace if built
if [ -f "/home/ros/colcon_ws/install/setup.bash" ]; then
  source /home/ros/colcon_ws/install/setup.bash
fi

# Allow GUI apps to connect (X11)
export QT_X11_NO_MITSHM=1

exec "$@"
