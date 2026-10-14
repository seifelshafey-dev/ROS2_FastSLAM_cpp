# How I got here

This is the order in which I found things and what I changed. It is not complete. Each number comes with the conditions I measured it under. Most are from offline runs on a synthetic room, a few from the simulator, none from a physical robot. What is still open is in [limits.md](architecture/limits.md).

## The Python prototype

I started in November 2025 with a differential-drive robot in a simulator and a recorded bag. I wrote FastSLAM 1.0, then FastSLAM 2.0, as Python scripts that replay the bag offline and plot the result. They are still in [`prototype/`](../prototype/README.md). I do not maintain them.

Two defects I know of in the 2.0 script, both fixed in the C++ package: the coarse search never replaces its start pose, and the matcher returns its start pose instead of the optimised one.

## The C++ port

In April 2026 I ported the method to C++ as the `slam_cpp` package: an occupancy grid with copy-on-write sharing, a scan matcher, a particle filter with adaptive resampling, a ROS 2 node and unit tests. I kept the structure of the prototype. The tests I wrote then were thin. Two of the filter tests asserted nothing.

## The first measurements

In October 2026 I went back through the code with probes: small programs on a synthetic ray-cast room that run the core with no ROS. The first build was clean, and the tests passed. The probes showed the faults the tests missed.

**Cell values read at the wrong place.** The grid writes a point into the cell that contains it, so a cell's value belongs to the whole cell. The matcher interpolated as if the value sat at the cell's lower corner. Every match was pulled half a cell toward negative x and y. On a map built from known poses, the mean match error went from (-0.050, -0.049) m to (-0.0002, +0.0010) m once the matcher read values at cell centres. On a 24 m circle in the synthetic room (400 updates, 80 particles, exact odometry), the error of the whole filter before my core fixes was an rms of 0.432 m with a final error of (-0.419, -0.417) m (seed 42, 180 points, 1 cm range noise). With the half-cell change alone it was 0.078 m, in a scratch copy. With all the core fixes it was 0.037 m for seed 42 and 0.034 to 0.035 m for seeds 1 to 5. I did not measure the other fixes one at a time. The derivation is in [grid.md](design/grid.md).

**The first scan was matched against an empty map.** The matcher had nothing to align to, and the first update sampled every particle from the wide prior. The first scan is now written into every map at the current pose without matching. Matching starts with the second scan.

**The best particle was index 0.** Resampling resets every weight to 1/N, so after each resampling the largest weight was simply the first particle, and the published pose switched to it. Each particle now carries a running sum of its log weights, inherited through resampling, and the best is the largest sum. The two tests that were meant to cover the filter asserted nothing. The new ones check the best particle across resampling.

**The optimiser threw away good answers.** The Levenberg-Marquardt step ended when no damped step lowered the cost, and reported that as a failure, so the match was discarded. Started from the true pose on its own map, the optimiser reported failure in three of the four cases I tried. Running out of tries now returns the current pose. On the room test, no match falls back any more (five in 395 at the start, one after the cell-centre change). Separately, the coarse search sometimes jumps 0.1 to 0.3 m along a wall; the optimiser now starts from whichever of the coarse pose and the odometry pose costs less. In one case found by a property test, the cost dropped from 9568 to 4915 and the position error from 0.13 m to under 0.03 m.

## The simulator and the reference robot

The first simulator run, before these fixes, gave an slam rms of 0.391 m against ground truth. After the core fixes, a second run gave 0.056 m rms over the moving part (path about 26 m, same drive script), computed from the stored recording. I had first written 0.030 m for it, could not reproduce that figure, and dropped it. The wheel separation, the launch file and the real-time factor differed between the two runs, so they are not the same experiment, and I ran each once.

**`base_link` was 0.10 m behind the wheel axle.** The wheel odometry is the motion of the axle centre, but it was published as the motion of `base_link`, where the lidar is. Over 16 turns in place the true `base_link` swung on a circle of 0.097 to 0.100 m radius while the odometry reported 0.001 m. I moved the wheels to `x = 0` of `base_link` and added a second caster. The radius from ground truth went to 0.00 m. This was a real fault, but it was not the main source of the pose error at rest, which was 0.039 m in the next run, against 0.046 and 0.047 m in the two before (one run each).

**The odometry lagged the robot by 0.15 s.** At the first straight, the fused odometry read 0.044 m behind ground truth while driving at 0.3 m/s and caught up at rest. Scans and ground truth agreed in time: over 81 scans, the wall ahead computed from scan range plus true position was off by -0.0009 m. The fused odometry was off by -0.0437 m, and a shift of 0.145 s removed it. The drive controller averaged its velocity over its default 10 samples, which at 30 Hz delays it by (10 - 1) / 2 / 30 = 0.15 s, and the EKF built position from that velocity. I set the window to 1 and measured again: the fused odometry error at the scan stamp was 0.0010 m.

That did not remove the SLAM error at rest. Over four runs of the same drive script it was 0.046, 0.039, 0.011 and 0.035 m. These were not repeats of one configuration: the robot model and the odometry settings changed between some of them, so this is not a clean run-to-run spread. I could not explain it. The filter is seeded, so equal inputs give equal outputs, and the inputs differ between simulator runs in timing. I have not measured how much comes from which scans trigger an update. The accuracy of the current code is in [accuracy.md](architecture/accuracy.md): 0.022 to 0.038 m rms over three live runs of the same drive. The node now stamps its pose with the scan time instead of the clock at publish time, which at the default 2 Hz map rate could be up to about half a second after the scan. My first recording started the bag 50 s after the launch and missed the first update, so I start it before the launch now.

**Odometry yaw read 4.7 percent low.** The simulated robot turned as if the wheels were 0.500 m apart, not the 0.525 m between their centres. I added a multiplier to the controller. Before it, the yaw error after 26.4 rad of turning was 1.2353 rad. After it, over 25.2 rad of turning, it was 0.0033 rad. That is one run each.

## A scan point outside the grid

The joint cost skipped any scan point whose surrounding cells were not all on the grid, so a pose that pushed points off the map was cheaper for free. Such a point now adds the cost of an unknown cell. Nothing changes while the scan stays inside the grid.

It is not finished. With a wall just past the grid edge the match error in x was still 0.059 to 0.122 m rms and the worst position error 0.18 to 0.35 m (grid edge at 4.10 m, walls at 4.13 to 4.30 m, synthetic room, map built from the true poses, each match started 0.02 m off), and I did not find the cause. The particle weight also still penalises an off-grid point about 30 times as much as an unknown cell, and I did not measure what that does. The advice I can give is to make the map larger than the area ([tuning.md](guide/tuning.md)).

## Thin scans and the first map

In the simulator, the pose error at rest varied between 0.01 and 0.05 m from one run to the next, though the robot model and odometry settings changed between those runs. I looked for the same spread offline.

I first measured this with a test program that had a bug: it fed the filter a tenth of each straight step. It showed the best particle stuck at the start for 1.2 m and blamed the map. I corrected this when the public tool `filter_run_dump` gave a different answer, and the mechanism I had written down is withdrawn.

What the tool shows, offline in an empty room with exact odometry: with 180 points at 0.05 m cells the rms error is 0.017 to 0.074 m over four seeds, and with every beam 0.005 m. At the old default (0.1 m cells, 180 points) seven seeds gave 0.009 to 0.014 m, and with every beam 0.007 to 0.008 m. My reading is that a seed spread of this kind is what I saw in the simulator. I did not test that.

A separate probe still stands. On a map holding only the first scan, at 0.05 m cells, after 0.06 m of driving, the start pose cost 7258.6 and the true pose 7777.2 (7319.4 against 7577.8 and 7382.0 against 7584.3 after 0.12 and 0.18 m). The second scan laid back on the first scan's cells scored better than the true position, whose far-wall points fall between those cells. That probe did not use the odometry step. I have not shown that it causes the errors above.

Across the tool's runs, kept points about three cells apart on the walls or more gave the large, seed-dependent errors, and the spacing-table rows under two cells apart gave 0.005 to 0.010 m at noise 0.01 m, and the seven-seed row at the old default (180 points, 0.1 m cells, 10 m room) gave 0.0091 to 0.0141 m (other cases under two cells were worse). One case at about three cells (360 points, 0.025 m cells, 0.0169 m) did not fit, so it is a guide from empty rectangular rooms. The default `scan_stride` is now 1, and the node logs a warning once when the kept points are more than two cells apart at the median range. The matcher itself is unchanged, so a sparse lidar can still give these errors. The tables and the argument are in [scan-density.md](design/scan-density.md).

## Smaller fixes

- `normalize_angle` returned `+pi` for the double just below `-pi`, because the wrap rounded up. It now returns a value in `[-pi, pi)`.
- The pose stored for the map update gate came from the best particle before resampling, which resampling can drop. It is now read after resampling.
- The node registers the first scan at once, builds `map` to `odom` from one moment, and rejects empty or equal frame names at start.
- A configuration is validated when the filter is built. The map must be a whole, even number of cells per side.

## What is still open

- No loop closure, so a double wall on return. I have not measured its size.
- The lidar is assumed to be at the `base_link` origin.
- Motion noise is added twice and the prior covariance is floored. On the circle with walls all round, that made no difference I could measure. I did not measure a corridor.
- The reference robot's IMU adds nothing I could measure to its yaw.
- I have not run any of this on a physical robot, and the only accuracy figures are the simulator runs in [accuracy.md](architecture/accuracy.md).

These are described, with what I measured, in [limits.md](architecture/limits.md).
