# Module map

Which file does what. Paths are from the repository root. Links are relative to this page.

## `src/slam_cpp`: the SLAM package

The core library is `slam_core`, built from the `.cpp` files in the first table. The node is built on top of it and is the only part that needs ROS.

| File | Responsibility |
|---|---|
| [`include/slam_cpp/slam_types.hpp`](../../src/slam_cpp/include/slam_cpp/slam_types.hpp) | `kPi`, `normalize_angle`, and the result and view structs the matcher and filter share (`GridView`, `BilinearResult`, `JacobianResult`, `LMResult`, `ScanMatchResult`). |
| [`include/slam_cpp/check.hpp`](../../src/slam_cpp/include/slam_cpp/check.hpp) | `SLAM_CHECK`, an always-on check that throws `std::invalid_argument`. |
| [`include/slam_cpp/slam_config.hpp`](../../src/slam_cpp/include/slam_cpp/slam_config.hpp), [`src/slam_config.cpp`](../../src/slam_cpp/src/slam_config.cpp) | `SlamConfig` and `validate`. |
| [`include/slam_cpp/sigmoid_lut.hpp`](../../src/slam_cpp/include/slam_cpp/sigmoid_lut.hpp) | Table from a stored cell value to an occupancy probability. |
| [`include/slam_cpp/occupancy_grid.hpp`](../../src/slam_cpp/include/slam_cpp/occupancy_grid.hpp), [`src/occupancy_grid.cpp`](../../src/slam_cpp/src/occupancy_grid.cpp) | The grid: cell indexing, ray casting, copy on write, map quality. |
| [`include/slam_cpp/scan_matching.hpp`](../../src/slam_cpp/include/slam_cpp/scan_matching.hpp), [`src/scan_matching.cpp`](../../src/slam_cpp/src/scan_matching.cpp) | Scan extraction, interpolation, joint cost, coarse search, Levenberg-Marquardt, weight, and `scan_match_single`. |
| [`include/slam_cpp/particle_filter.hpp`](../../src/slam_cpp/include/slam_cpp/particle_filter.hpp), [`src/particle_filter.cpp`](../../src/slam_cpp/src/particle_filter.cpp) | `ParticleFilter`: motion model, sampling, weights, resampling, map updates. |
| [`include/slam_cpp/odometry.hpp`](../../src/slam_cpp/include/slam_cpp/odometry.hpp), [`src/odometry.cpp`](../../src/slam_cpp/src/odometry.cpp) | Motion decomposition and planar pose algebra (`compose`, `inverse`, `map_to_odom`). |
| [`src/ros/slam_node.cpp`](../../src/slam_cpp/src/ros/slam_node.cpp) | The ROS 2 node. |
| [`config/slam_params.yaml`](../../src/slam_cpp/config/slam_params.yaml) | Parameters the launch file loads. |
| [`launch/slam_cpp.launch.py`](../../src/slam_cpp/launch/slam_cpp.launch.py) | Starts the node. |
| [`CMakeLists.txt`](../../src/slam_cpp/CMakeLists.txt) | Builds `slam_core`, the node and the tests. Option `SLAM_BUILD_ROS` (default ON); OFF builds the core and its tests without ROS. |
| [`README.md`](../../src/slam_cpp/README.md) | Package readme. |

## Tests

All in [`src/slam_cpp/test/`](../../src/slam_cpp/test/). GoogleTest, plus RapidCheck for the files named `*_property_test.cpp`.

| File | Covers |
|---|---|
| `slam_types_test.cpp` | `normalize_angle`. |
| `slam_config_test.cpp`, `slam_config_property_test.cpp` | `validate` and the grid size checks. |
| `occupancy_grid_test.cpp` | Indexing, ray casting, clamping, copy on write, map quality. |
| `scan_matching_test.cpp`, `scan_matching_property_test.cpp` | Scan extraction, interpolation, cost, coarse search, Levenberg-Marquardt, point spacing. |
| `scan_matching_integration_test.cpp` | A match on a map written at a known pose. |
| `particle_filter_test.cpp` | Prediction, the first update, the best particle, thread-count independence. |
| `particle_filter_integration_test.cpp` | The filter around a circle in a synthetic room. |
| `odometry_test.cpp`, `odometry_property_test.cpp` | Motion decomposition and pose algebra. |
| `support/` | The synthetic room that generates scans for the tests, with its own tests. |

## `src/robot_bringup`: the reference robot

| Path | Responsibility |
|---|---|
| [`src/robot_bringup/urdf/`](../../src/robot_bringup/urdf/) | Robot model (xacro): links, joints, `ros2_control`, lidar, IMU. |
| [`src/robot_bringup/config/my_controllers.yaml`](../../src/robot_bringup/config/my_controllers.yaml) | Controller manager and differential drive settings. |
| [`src/robot_bringup/config/ekf.yaml`](../../src/robot_bringup/config/ekf.yaml) | The EKF from `robot_localization`. |
| [`src/robot_bringup/worlds/warehouse.sdf`](../../src/robot_bringup/worlds/warehouse.sdf) | Simulator world. |
| [`src/robot_bringup/launch/sim.launch.py`](../../src/robot_bringup/launch/sim.launch.py) | Simulator, robot, bridge, controllers, EKF, RViz, teleop. |
| [`src/robot_bringup/launch/slam_sim.launch.py`](../../src/robot_bringup/launch/slam_sim.launch.py) | `sim.launch.py` plus the SLAM node. |
| [`src/robot_bringup/src/teleop_node.cpp`](../../src/robot_bringup/src/teleop_node.cpp) | Keyboard teleop. |
| [`src/robot_bringup/rviz/my_robot.rviz`](../../src/robot_bringup/rviz/my_robot.rviz) | RViz layout. |

See [reference-robot.md](reference-robot.md).

## Elsewhere

| Path | What |
|---|---|
| [`Doxyfile`](../../Doxyfile) | Generates the API documentation from `src/slam_cpp/include` and `src/slam_cpp/src/ros`. Warnings fail the run. |
| [`docker/`](../../docker/) | The Dockerfile with a `ci` and a `sim` stage, a compose file, and a lock check script. See [environment.md](environment.md). |
| [`.github/workflows/`](../../.github/workflows/) | `ci.yml` builds and tests; `image.yml` builds the `ci` image. |
| [`prototype/`](../../prototype/) | The Python scripts I started from. Not maintained. |
