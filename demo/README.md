# JPS Global Planner Demo

This demo shows the Jump Point Search (JPS) global planner working with ROS Navigation stack.

## Prerequisites

- Docker with Dev Containers support
- VS Code with Remote - Containers extension (or any Dev Container compatible IDE)
- X11 forwarding for GUI (RViz)

## Quick Start

### 1. Open in Dev Container

Open this project in VS Code and when prompted, click "Reopen in Container". This will:
- Build a Docker image with ROS Noetic
- Install all dependencies including the `jps3d` library
- Build the catkin workspace

### 2. Run the Demo

Once the container is built and the workspace is compiled:

```bash
# Terminal 1 - Launch the demo
roslaunch jps_global_planner demo.launch
```

### 3. Set Navigation Goals

In RViz:
1. Click the "2D Nav Goal" button in the toolbar
2. Click and drag on the map to set a goal pose
3. Watch the JPS planner compute the path!

## What You'll See

- **Green path**: The final smoothed path from JPS planner
- **Yellow path**: The raw JPS path (before DMP smoothing)
- **Blue path**: Local planner trajectory
- **Map**: A demo environment with walls and obstacles

## Configuration

### JPS Planner Parameters

Edit `demo/config/jps_planner_params.yaml`:

```yaml
debug: false  # Set to true for verbose output

dmp:  # Distance Map Planner parameters
  potential_radius:
    x: 0.5
    y: 0.5
  search_radius:
    x: 0.5
    y: 0.5
  potential_map_range:
    x: 2.0
    y: 2.0
```

### Using Your Own Map

1. Place your map files (`.pgm` and `.yaml`) in `demo/maps/`
2. Update the `map_file` argument in the launch file:

```bash
roslaunch jps_global_planner demo.launch map_file:=/path/to/your/map.yaml
```

## Troubleshooting

### X11 Display Issues

If RViz doesn't open, you may need to allow X11 connections:

```bash
# On the host machine
xhost +local:docker
```

### Build Errors

If the build fails, try rebuilding:

```bash
cd /catkin_ws
catkin clean -y
catkin build
```

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                      move_base                          │
├─────────────────────────────────────────────────────────┤
│  Global Planner: jps/JumpPointSearchROS                 │
│  ├── Jump Point Search (JPS) for initial path          │
│  └── Distance Map Planner (DMP) for path smoothing     │
├─────────────────────────────────────────────────────────┤
│  Local Planner: TrajectoryPlannerROS                    │
│  └── Handles local obstacle avoidance                   │
├─────────────────────────────────────────────────────────┤
│  Costmaps: Global (static) + Local (rolling window)    │
└─────────────────────────────────────────────────────────┘
```
