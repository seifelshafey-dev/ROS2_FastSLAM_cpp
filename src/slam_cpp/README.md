# slam_cpp

The SLAM package: a ROS-free core (occupancy grid, scan matcher, Rao-Blackwellized particle filter) and one ROS 2 node over it. The module layout is in [module-map.md](../../docs/architecture/module-map.md).

## Build

From the repository root, after sourcing ROS 2 Humble:

```bash
colcon build --packages-select slam_cpp
source install/setup.bash
```

To build the core without ROS, see [testing.md](../../docs/guide/testing.md).

## Run

The node is `src/ros/slam_node.cpp`. Its launch file is [`launch/slam_cpp.launch.py`](launch/slam_cpp.launch.py) and its parameter file is [`config/slam_params.yaml`](config/slam_params.yaml).

```bash
ros2 launch slam_cpp slam_cpp.launch.py use_sim_time:=true
```

Every parameter is described in [config.md](../../docs/architecture/config.md). Topics and frames are in [ros.md](../../docs/architecture/ros.md).

## More

- [Root README](../../README.md)
- [Getting started](../../docs/guide/getting-started.md)
- [Architecture overview](../../docs/architecture/overview.md)
