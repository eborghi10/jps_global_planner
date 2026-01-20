#!/bin/bash
set -e

# Setup ROS environment
source /opt/ros/noetic/setup.bash

# Source catkin workspace if it exists
if [ -f /catkin_ws/devel/setup.bash ]; then
    source /catkin_ws/devel/setup.bash
fi

# Export TurtleBot3 model
export TURTLEBOT3_MODEL=burger

exec "$@"
