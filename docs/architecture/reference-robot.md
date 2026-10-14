# The reference robot

`robot_bringup` is a small differential-drive robot in an Ignition Fortress world. I made it so the SLAM package can be tried without hardware. It is not part of the SLAM library. The package does not depend on it, and another robot works if it meets the assumptions in [ros.md](ros.md).

It has only been run in simulation. I have not run it on a physical robot.

## Model

Files: [`src/robot_bringup/urdf/`](../../src/robot_bringup/urdf/).

- `base_link` is a 0.5 m by 0.5 m by 0.25 m box of 2.0 kg ([`robot_properties.xacro`](../../src/robot_bringup/urdf/robot_properties.xacro)).
- Two drive wheels, radius 0.075 m, width 0.025 m, 0.5 kg each. Their joints sit at `x = 0` of `base_link`, so the robot turns about the middle of the chassis. Their centres are 0.525 m apart ([`my_robot_joints.xacro`](../../src/robot_bringup/urdf/my_robot_joints.xacro)).
- Two ball casters at `x = -0.2 m` and `x = +0.2 m`.
- The lidar is fixed to `base_link` at `(0, 0, 0.15)` with no rotation. That is the placement the SLAM node assumes: at the `base_link` origin in x and y, no yaw. A 2D scan does not depend on height.
- The lidar is a GPU lidar: 360 samples over `-pi` to `pi`, range 0.15 m to 10 m, Gaussian noise with a standard deviation of 0.01 m, 30 Hz ([`lidar.xacro`](../../src/robot_bringup/urdf/lidar.xacro)). It is bridged to `/scan`.
- The IMU is fixed at `(0, 0, 0.255)`, 100 Hz, bridged to `/imu/data` ([`imu.xacro`](../../src/robot_bringup/urdf/imu.xacro)).

The simulator publishes `/scan` with the frame id `my_bot/base_link/laser`. The SLAM node does not read the frame id of a scan.

## Controllers

File: [`src/robot_bringup/config/my_controllers.yaml`](../../src/robot_bringup/config/my_controllers.yaml). Wheel velocity is commanded through `ros2_control` ([`ros2_control.xacro`](../../src/robot_bringup/urdf/ros2_control.xacro), limit 10 rad/s per wheel).

- Controller manager at 30 Hz with a joint state broadcaster and `diff_drive_controller`.
- `diff_drive_controller` publishes `/diff_cont/odom` at 30 Hz and does not publish `odom` to `base_link`. The EKF does.
- `velocity_rolling_window_size: 1`. The default averages the velocity over 10 samples, which is about 0.15 s late at 30 Hz. In two simulator runs of the same drive script, the wall position read through the fused odometry at the scan stamp was off by -0.0437 m with the default window and by 0.0010 m with a window of 1 (81 scans on the first straight each).

**Wheel separation calibration.** `wheel_separation` is 0.525 m, the distance between the wheel centres, and `wheel_separation_multiplier` is 0.953. The multiplier is a calibration: in the simulator the robot turns as if its wheels were about 0.500 m apart. I measured it from a run in which the odometry yaw ended 1.2353 rad off the true yaw after 26.4 rad of turning (about 4.7 percent), and the ratio of the two net yaws gave 0.5004 m. With the multiplier in place, one run of the same drive script ended with the fused odometry yaw 0.0033 rad off and the position 0.001 m off. That is one run. I did not repeat it. If you change the wheel geometry or the simulator, measure it again.

## EKF

File: [`src/robot_bringup/config/ekf.yaml`](../../src/robot_bringup/config/ekf.yaml). `robot_localization` at 30 Hz, two-dimensional mode, world frame `odom`. It publishes `odom` to `base_link` and the topic `/odometry/filtered` that the SLAM node reads by default.

As configured, it fuses:

- from `/diff_cont/odom`: the forward speed and the yaw rate;
- from `/imu/data`: the yaw, the yaw rate and the acceleration along x.

It fuses no position or yaw directly from the wheel odometry.

I have not measured what the IMU contributes. In the recordings I checked, the net yaw of `/odometry/filtered` and of `/diff_cont/odom` agreed to four decimal places in the two recordings I checked (-25.1328 in both for one of them), and the simulator logged that it did not find the IMU sensor in the controller's hardware list. I have not recorded the rate or frame id of `/imu/data`. So I make no claim that the reference robot's odometry uses the IMU. See [limits.md](limits.md).

## World

File: [`src/robot_bringup/worlds/warehouse.sdf`](../../src/robot_bringup/worlds/warehouse.sdf). A closed room, walls at `x = ±5 m` and `y = ±4 m` (0.15 m thick, 2 m high), with a divider, a table, a shelf, a pillar and a crate inside. Physics step 1 ms. The robot spawns at the world origin.

## Launch and teleop

[`sim.launch.py`](../../src/robot_bringup/launch/sim.launch.py) starts the robot state publisher, the simulator server with headless rendering, an optional Ignition GUI, the spawn of the robot into the world (`ros_gz_sim create`, name `my_bot`, `z` 0.2 m), the bridge (`/scan`, `/imu/data`, `/clock`, and the true poses of the models remapped to `/ground_truth/poses`), the two controller spawners in sequence, the EKF, RViz and teleop. Arguments `gui`, `rviz` and `teleop` default to true. [`slam_sim.launch.py`](../../src/robot_bringup/launch/slam_sim.launch.py) adds the SLAM node and passes the three arguments through.

The teleop node ([`teleop_node.cpp`](../../src/robot_bringup/src/teleop_node.cpp)) reads keys in an `xterm` and publishes `geometry_msgs/Twist` to `/diff_cont/cmd_vel_unstamped`, at most 1.0 m/s and 0.75 rad/s.

No test covers `robot_bringup` beyond the linters that `colcon test` runs on it.
