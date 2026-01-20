#!/bin/bash
set -e

echo "=== Post-create script starting ==="

# Source ROS
source /opt/ros/noetic/setup.bash

# Navigate to catkin workspace
cd /catkin_ws

# Build the workspace
echo "=== Building catkin workspace ==="
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
source /catkin_ws/devel/setup.bash

echo "=== Post-create script completed ==="
echo ""
echo "To run the demo:"
echo "  Terminal 1: roslaunch jps_global_planner demo.launch"
echo "  Terminal 2: Use RViz to set 2D Nav Goal"
