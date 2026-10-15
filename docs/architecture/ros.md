# The ROS node

Source: [`slam_node.cpp`](../../src/slam_cpp/src/ros/slam_node.cpp). It is the only file in the package that includes a ROS header. No test covers the node: the tests build the core without ROS. I checked the behaviour below by reading the code.

The class comment of `SlamNode` in that file is the owner of the topic list, the QoS and the full parameter table. I do not copy them here. If this page and that comment disagree, the comment is right.

## Topics

| Direction | Default name | Type |
|---|---|---|
| in | `/odometry/filtered` (parameter `odom_topic`) | `nav_msgs/msg/Odometry` |
| in | `/scan` (parameter `scan_topic`) | `sensor_msgs/msg/LaserScan` |
| out | `/slam/map` | `nav_msgs/msg/OccupancyGrid` |
| out | `/slam/particles` | `geometry_msgs/msg/PoseArray` |
| out | `/slam/best_pose` | `geometry_msgs/msg/PoseStamped` |

The QoS of each is in the class comment. The output topic names are fixed in the code. All output messages carry `map_frame` as frame. `/slam/map` and the `map` to `odom` transform carry the node clock. `/slam/particles` and `/slam/best_pose` carry the stamp of the scan that triggered the update.

The map has `map_size / map_resolution` cells per side and its origin at `(-map_size / 2, -map_size / 2)` in the map frame. A cell is -1 for unknown, otherwise the occupancy probability in percent, 0 to 100. The map is the grid of the best particle.

## Frames and the transform

The node reads the odometry pose from the message as `(x, y, yaw)`. An odometry message whose position or orientation is not finite is dropped with a warning, at most one per 5 s, and changes nothing. It does not look at `child_frame_id` or `header.frame_id`. It reads no transform. It assumes the lidar is at the `base_link` origin with no rotation, so the scan header frame is also ignored.

The node broadcasts `map_frame` to `odom_frame` (defaults `map` and `odom`). It does not publish `odom` to `base_link`. Something else has to, in my setup the EKF.

The transform is built from one moment. At each SLAM update the node stores the odometry pose of that update, and the transform is `map_to_odom(best particle pose, stored odometry pose)`, from [`odometry.hpp`](../../src/slam_cpp/include/slam_cpp/odometry.hpp). With `T` the pose of `odom` in `map`, it satisfies `compose(T, odom_pose) = slam_pose`. So the transform stays constant between updates. Checked by `Odometry.MapToOdomOfEqualPosesIsIdentity`, `Odometry.MapToOdomHandComputedWith90DegreeHeadingDifference` and `OdometryProperty.ComposeMapToOdomWithOdomGivesSlamPose` (`../../src/slam_cpp/test/odometry_test.cpp`, `odometry_property_test.cpp`). The node's use of it is not tested.

## What each callback does

**Odometry.** The first message sets the reference pose and nothing else. Each later message adds the change in x, y and heading to a running total. The heading total is wrapped to `[-pi, pi)`.

**Scan.**

- A scan that arrives before the first odometry message is ignored.
- The total is split into a turn, a move and a second turn by `decompose_motion`. A net move under 1 cm is treated as pure rotation, and a motion more than 90 degrees from the heading is taken as reverse. Checked by `Odometry.SidewaysBelowThresholdIgnoresBearing`, `Odometry.SidewaysAboveThresholdUsesBearing` and `Odometry.StraightReverse`.
- After the first scan, the scan is used only if the move exceeds `motion_threshold_trans` or the heading total exceeds `motion_threshold_rot`. Otherwise it is dropped and the totals keep growing.
- The scan is turned into points with `extract_local_scan`, using `scan_stride`. If no beam is valid the scan is dropped and the totals are kept.
- At the first scan that is used, the node logs a warning once if the kept points are more than 2 cells apart at the median range. See [limits.md](limits.md).
- The first used scan goes to `update_and_resample` without `predict`. Later scans call `predict` first.
- After an update the totals are reset and the odometry pose is stored for the transform.

**Publish timer.** At `map_publish_rate` (default 2 Hz) the node publishes the map and the transform, both stamped with the node clock. Nothing is published before the first update. If the clock has gone backwards, as when a simulator restarts, the publish is skipped with a warning. The timer uses the node clock, so `use_sim_time` works.

**Pose and particles.** `/slam/best_pose` and `/slam/particles` are published once per filter update, from the scan callback, and are stamped with the scan's header stamp. The transform is stamped with the node clock, so its stamp differs from the pose stamp. Between updates (below the motion threshold) no new pose or particle message is published. The first update is made at the first valid scan, so its pose is published at startup; a subscriber that connects later sees the next pose only after the robot has moved past the threshold.

The node runs on a single-threaded executor, so its callbacks never run at the same time.

## Parameters

The table is in the class comment of `SlamNode`. [config.md](config.md) says what each setting trades.

Parameters that become `SlamConfig` fields are checked by `validate` when the node starts. A bad value makes construction throw `std::invalid_argument`. `main` catches it, logs one fatal line with the message, which names the field and the rule, and the process exits with status 1. The five that are not `SlamConfig` fields:

| Parameter | Default | Check |
|---|---|---|
| `odom_topic` | `/odometry/filtered` | none |
| `scan_topic` | `/scan` | none |
| `map_frame` | `map` | not empty, different from `odom_frame` |
| `odom_frame` | `odom` | not empty, different from `map_frame` |
| `map_publish_rate` | 2.0 Hz | greater than 0, and the timer period `1 / rate` must be between 1 ns and 9e9 s (a rate of about 1.1e-10 Hz to 1e9 Hz) |

## Launch

[`slam_cpp.launch.py`](../../src/slam_cpp/launch/slam_cpp.launch.py) starts the node with [`slam_params.yaml`](../../src/slam_cpp/config/slam_params.yaml) and a `use_sim_time` argument (default `false`). The yaml holds the same values as the defaults in `SlamConfig`. No test reads the yaml. `SlamConfig.ShippedYamlValuesAreValid` (`slam_config_test.cpp`) validates a hand-typed copy of the yaml values, so a change to the yaml is not caught by it.

[`slam_sim.launch.py`](../../src/robot_bringup/launch/slam_sim.launch.py) in `robot_bringup` starts the simulator, the robot and the node together with `use_sim_time` true. See [reference-robot.md](reference-robot.md).
