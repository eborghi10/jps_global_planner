# jps_global_planner : Jump Point Search

A global planner for ROS based on Jump Point Search algorithm.

- [How does JPS work?](https://zerowidth.com/2013/a-visual-explanation-of-jump-point-search.html)

- This plugin makes use of [`jps3d`](https://github.com/KumarRobotics/jps3d) library

- Author: [Emiliano Borghi](https://github.com/eborghi10)

## Build status

![Build Status](https://api.travis-ci.org/eborghi10/jps_global_planner.svg?branch=master)
[![GitHub issues](https://img.shields.io/github/issues-raw/eborghi10/jps_global_planner)](https://github.com/eborghi10/jps_global_planner/issues)
[![GitHub](https://img.shields.io/github/license/eborghi10/jps_global_planner)](https://github.com/eborghi10/jps_global_planner/blob/kinetic-devel/LICENSE)

## Install dependencies

```bash
/catkin_ws/src $ git clone https://github.com/KumarRobotics/jps3d.git

cd /catkin_ws && catkin_make_isolated --install --install-space /opt/ros/$ROS_DISTRO -DCMAKE_BUILD_TYPE=Release
```

## Usage

To buid this ROS package, just compile it normally with `catkin_make`.

Create a planner parameter file for `move_base` with this information:

```bash
base_global_planner: "jps/JumpPointSearchROS"

JumpPointSearchROS:
  debug: true
```

## Backtrace with gdb

- Build `move_base` from source

- Install gdb:

```bash
sudo apt-get install -y gdb
```

- Compile the workspace with debug symbols:

```bash
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

- Execute `move_base` with this plugin and [follow these steps to use GDB](http://wiki.ros.org/roslaunch/Tutorials/Roslaunch%20Nodes%20in%20Valgrind%20or%20GDB).

- Add a breakpoint (Shift + Insert to paste the following code):

```bash
break /catkin_ws/src/jps_global_planner/src/jps_ros.cpp:180
```

[GDB Cheat sheet](https://darkdust.net/files/GDB%20Cheat%20Sheet.pdf)

## Testing

```bash
rosrun jps_global_planner test_jps_global_planner `rospack find jps_global_planner`/test/data/corridor.yaml
```

```bash
catkin_make roslint
```
