# Getting started

This page takes you from a fresh clone to a map in the simulator. I have run the package only in simulation, never on a physical robot. What it assumes about your robot is in [ros.md](../architecture/ros.md).

## Requirements

- Ubuntu 22.04 and ROS 2 Humble (`ros-humble-ros-base` or the desktop variant). The packages below add the rest; `rclcpp`, the message packages, `ament_cmake`, `ament_cmake_gtest` and `ros2launch` come with Humble.
- Ignition Fortress, for the simulator in `robot_bringup`. On Humble, the `ros-humble-ros-gz-sim` package below brings it in. I checked this in the `sim` image: `/usr/bin/ign` is version 6.18.0. I took the package list from the `sim` stage of [`docker/Dockerfile`](../../docker/Dockerfile) and have not tried a bare host install.
- A C++17 compiler. I built with GCC 11.4 and GCC 12 ([environment.md](../architecture/environment.md)). I have not tried Clang.

To build and test the SLAM package alone:

```bash
sudo apt update
sudo apt install build-essential cmake git python3-colcon-common-extensions \
  libeigen3-dev libgtest-dev \
  ros-humble-tf2-ros ros-humble-ament-lint-auto ros-humble-ament-lint-common \
  doxygen graphviz
```

`doxygen` and `graphviz` are only for the Doxygen command in [testing.md](testing.md).

The simulator and the reference robot need these as well:

```bash
sudo apt install libgl1-mesa-dri mesa-utils xterm \
  ros-humble-controller-manager ros-humble-diff-drive-controller \
  ros-humble-ign-ros2-control ros-humble-joint-state-broadcaster \
  ros-humble-robot-localization ros-humble-robot-state-publisher \
  ros-humble-ros-gz-bridge ros-humble-ros-gz-sim ros-humble-rviz2 \
  ros-humble-xacro
```

The first build downloads RapidCheck with `git` for the property tests. See [testing.md](testing.md) to skip it.

## Build

The repository root works as the workspace. `colcon` finds the two packages under `src/`.

```bash
git clone https://github.com/seifelshafey-dev/ROS2_FastSLAM_cpp.git
cd ROS2_FastSLAM_cpp
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

`colcon build` without arguments builds both packages. Without a build type the library builds as Release. To build only the SLAM package, add `--packages-select slam_cpp`.

## Run

Run these in a shell where you sourced `install/setup.bash`.

### The simulator

```bash
ros2 launch robot_bringup sim.launch.py
```

This starts Ignition Fortress with the warehouse world, the robot, its controllers, the EKF that publishes `odom` to `base_link`, RViz and a keyboard teleop in an `xterm`. It does not start the SLAM node. Its ROS nodes run on the simulation clock.

| Argument | Default | What it does |
|---|---|---|
| `gui` | `true` | Starts the Ignition GUI client. The simulation server runs without it. |
| `rviz` | `true` | Starts RViz. |
| `teleop` | `true` | Starts the keyboard teleop in its own `xterm`. |

With `gui`, `rviz` and `teleop` all `false`, nothing needs a display:

```bash
ros2 launch robot_bringup sim.launch.py gui:=false rviz:=false teleop:=false
```

### The SLAM node

In a second shell, with the simulator running:

```bash
ros2 launch slam_cpp slam_cpp.launch.py use_sim_time:=true
```

| Argument | Default | What it does |
|---|---|---|
| `use_sim_time` | `false` | Runs the node on the simulation clock. Set it to `true` with the simulator. |

The launch file loads [`slam_params.yaml`](../../src/slam_cpp/config/slam_params.yaml). Every setting is described in [config.md](../architecture/config.md). To change a setting, edit that file and rebuild.

### Both at once

```bash
ros2 launch robot_bringup slam_sim.launch.py
```

This includes `sim.launch.py` and starts the SLAM node with the same parameter file, always on the simulation clock. It takes the same three arguments with the same defaults and passes them on.

| Argument | Default |
|---|---|
| `gui` | `true` |
| `rviz` | `true` |
| `teleop` | `true` |

## What to look at

Drive the robot with the teleop window: it prints its keys when it starts. The SLAM node publishes `/slam/map` and the transform on a timer, but `/slam/best_pose` and `/slam/particles` only after a filter update, which needs the robot to move.

| Topic | Type | What it is |
|---|---|---|
| `/slam/map` | `nav_msgs/msg/OccupancyGrid` | The map of the best particle. |
| `/slam/best_pose` | `geometry_msgs/msg/PoseStamped` | The best particle's pose in the map frame. Published once per filter update, stamped with the scan stamp. |
| `/slam/particles` | `geometry_msgs/msg/PoseArray` | The pose of every particle. Published once per filter update, stamped with the scan stamp. |

The node also broadcasts the `map` to `odom` transform. The topics it reads, the frames and the QoS are in [ros.md](../architecture/ros.md).

```bash
ros2 topic list
ros2 topic hz /slam/map
```

`ros2 topic hz /slam/map` should show about 2 Hz with the robot standing still. A pose appears on `/slam/best_pose` once the robot has moved 0.05 m or turned 0.05 rad, for example with the teleop. `ros2 topic echo /slam/best_pose --once` prints nothing before that.

The RViz configuration I ship shows the robot and `/scan`. Its fixed frame is already `map`, so RViz reports a missing frame until the SLAM node runs. It has no map display, so add a Map display for `/slam/map`.

The node warns once if the scan is too sparse for the cell size. [tuning.md](tuning.md) explains the warning.

## The container route

If you would rather not install anything on the host, the repository has a container for building and testing. From the repository root:

```bash
docker compose -f docker/compose.yaml run --rm dev
```

This opens a shell in the `ci` image with `src/` mounted at `/ws/src`. The build, install and log directories live in Docker volumes, so they persist between runs. The image has GCC 12.3.0 and CMake 3.22.1. Inside the shell:

```bash
cd /ws
colcon build --packages-select slam_cpp
colcon test --packages-select slam_cpp
colcon test-result --verbose
```

In that shell the build finished in 2 min 16 s, and `colcon test-result` printed 0 errors, 0 failures. The image has `doxygen` and `graphviz`, but the compose file mounts only `src`, so I run the Doxygen command on the host ([testing.md](testing.md)).

Both images build from `docker/Dockerfile`. The `ci` image is about 1.7 GB on disk and the `sim` image about 3.7 GB.

The `sim` service is the same with the simulator packages added, software rendering only. I do not give a recipe for showing its windows on your display.

```bash
docker compose -f docker/compose.yaml run --rm sim
```

The entrypoint sources only `/opt/ros/humble/setup.bash`, so build and source the workspace first. The launch below turns off the Ignition GUI, RViz and the teleop window:

```bash
cd /ws && colcon build && source install/setup.bash
ros2 launch robot_bringup slam_sim.launch.py gui:=false rviz:=false teleop:=false
```

In the `sim` container I ran that launch for 100 s with the image's software rendering. Ignition Fortress (`ign gazebo` 6.18.0, which comes with `ros-humble-ros-gz-sim`) started, `/scan` and `/slam/best_pose` were published and the simulation clock ran at about real time. I did not drive the robot in the container. I have not tried the GUI or RViz from the container.
