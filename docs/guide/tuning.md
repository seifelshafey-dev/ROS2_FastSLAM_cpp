# Tuning

This page gives advice where I have a measurement behind it, and says so where I do not. Every setting and its range is in [config.md](../architecture/config.md). The failures I know of are in [limits.md](../architecture/limits.md). I have run the package only in simulation and in offline runs against a synthetic room, never on a physical robot, so none of this is tested on yours.

The numbers come from empty rectangular rooms with exact or simulated odometry. A room with furniture, a corridor, or a different lidar may behave differently. I did not measure those.

## Cell size against the spacing of scan points

The one rule I can back is about how far apart the scan points fall on a wall, measured in cells.

```
spacing = range * angle between kept beams     (metres)
cells   = spacing / map_resolution
```

The angle between kept beams is the lidar's `angle_increment` times `scan_stride`. For a lidar with 360 beams over a full turn and `scan_stride` 1, the angle is 0.01745 rad. A wall 5 m away then has points 0.087 m apart, which is 0.87 cells at 0.1 m cells and 1.7 cells at 0.05 m cells. That is arithmetic, not a measurement.

**What I measured.** Offline with `filter_run_dump` (see [`tools/README.md`](../../src/slam_cpp/tools/README.md)), empty rectangular room, exact odometry, range noise 0.01 m, 80 particles, seed 42 unless a row lists seeds, rms position error over the whole run. When the kept points were about three cells apart or more on the walls, the error was large for the cell size and changed with the seed. In the spacing-table rows under two cells apart, at noise 0.01 m with the walls off the cell boundary, the error was 0.005 to 0.010 m, and the seven-seed row at the old default (180 points, 0.1 m cells, 10 m room, 1.4 to 1.7 cells) gave 0.0091 to 0.0141 m; other cases under two cells were worse (noise 0.03 m gave 0.0628 m with 360 points).

| Kept points | Cell | Walls at | Spacing on the wall | rms error |
|---|---|---|---|---|
| 360 | 0.10 m | 7.9 to 9.9 m | 1.4 to 1.7 cells | 0.0068 m |
| 180 | 0.10 m | 7.9 to 9.9 m | 2.7 to 3.4 cells | 0.0518 m |
| 360 | 0.05 m | 3.9 to 4.9 m | 1.4 to 1.7 cells | 0.0046 m |
| 180 | 0.05 m | 3.9 to 4.9 m | 2.7 to 3.4 cells | 0.0738, 0.0217, 0.0170, 0.0419 m (seeds 1, 4, 5, 42) |

In the 10 m room the old default (180 points, 0.1 m cells) was at 1.4 to 1.7 cells, under the two-cell guide, yet the error fell again from 180 to 360 points (0.009 to 0.014 m against 0.007 to 0.008 m, seven seeds). So the spacing guide does not explain that gain. Every beam lowered the error in each sparse case I measured. The full table and the sweep over cell sizes are in [scan-density.md](../design/scan-density.md).

**What I did about it.** The default `scan_stride` is 1, so every beam is used. The node measures the spacing at the first scan it uses, as the median range times the angle between kept beams, and logs one warning if it is above 2 cells. Nothing stops the node. If you see the warning, lower `scan_stride` or raise `map_resolution`.

**What I did not measure.**

- The two-cell limit is a guide from empty rectangular rooms. One row does not fit a cut at three cells: 360 points with 0.025 m cells (2.7 to 3.4 cells) gave 0.0169 m.
- I did not measure a cluttered room, a lidar other than the simulated 360-beam one, or cell sizes below 0.025 m.
- With 0.1 m cells, 360 points and an empty room, a wall face exactly on a cell boundary left a steady offset of 0.050 m in x (`filter_run_dump`). I did not measure it at other cell sizes.

## Scan stride

`scan_stride` thins the scan, and so makes the points sparser. The reason to raise it is time. Measured with `filter_run_dump`, Release -O3, OpenMP on, 12 cores, 80 particles, 0.1 m cells, 714 updates, whole-program wall time (scan generation and CSV writing included), on an otherwise idle machine: 3.5 to 3.8 s with 180 points and 6.0 to 6.2 s with 360, two runs each, about 5 ms and 8.5 ms per update. I have not measured the node's load in the simulator. Check the spacing rule above before you raise it.

## Map size against the mapped area

The grid is a fixed square of `map_size` metres, centred on the start pose. Set it so that the whole area you will map, measured from the start in every direction, fits with room to spare. The default is 50 m with 0.1 m cells, which is a 500 by 500 grid of one byte per cell, 250 KB per particle before copies share storage. With 80 particles that is at most 20 MB, by arithmetic.

**From the code.** A scan point off the grid is not written. The matcher charges it the same as a point in an unknown cell. The particle weight does not: it charges the smallest probability it allows, about 30 times the penalty of an unknown cell (log of 1e-9 is -20.7, log of 0.5 is -0.69, before the weight gain). I did not measure what that does to the weights.

**What I measured.** With a wall just past the grid edge, the pose in x was still disturbed in offline tests: grid edge at 4.10 m, walls at 4.13 to 4.30 m, synthetic room. The matcher alone ran on a map built from the true poses, each match started 0.02 m off. The match error in x was 0.059 to 0.122 m rms, and the worst position error 0.18 to 0.35 m. I did not find the cause.

**What I did not measure.** How much margin is enough, and a large grid's cost in time. I did not test a map near the 20000 cells per side limit.

## Particle count

The default is 80. Scan matching runs once per particle, and each particle holds a grid. So both time and memory grow with the count. I have not measured a sweep over `num_particles`, so I have no accuracy figure for any other value and no measured rate of growth.

What I have: with 80 particles, the full filter on a 24 m circle in a synthetic room (180 points, exact odometry, seeds 1 to 5 and 42) gave an rms position error of 0.034 to 0.037 m. In a probe before the fixes on this page (20 m map, `-O3`), the same filter took 3.1 ms per update on 12 threads and 9.9 ms on one thread. The cores and compiler flags matter; `-O0` took 51.7 ms.

## Motion noise

`alpha1` to `alpha4` scale the noise added to each particle's motion. Larger values spread the particles more. I have not measured their effect on accuracy, and I have no best value for any robot.

What I measured is narrower. Motion noise is added twice per update, and the matcher raises each diagonal of the prior covariance to at least 0.01. On the 24 m circle above, replacing the propagated covariance by a constant gave the same rms error to four decimal places (mean rms 0.0350 m). Lowering the floor to 1e-4 or 1e-6 moved it by 1 mm or less. On the drive-script room, in scratch copies of the core with the floor constant changed (`filter_run_dump` has no floor option) and run through the same loop (360 points, 0.1 m cells, seeds 1, 4, 42) floors of 0.01, 1e-4 and 1e-5 gave 0.0075 to 0.0082 m, and with 180 points 0.0093 to 0.0111 m, so there too the floor made no measurable difference. The circle room had walls in every direction. I did not measure a corridor, where the scan does not constrain one axis and the prior decides.

Odometry that lags the robot is a fault to fix at the source, not a reason for larger noise. In the reference robot a velocity average of 10 control periods delayed the odometry by about 0.15 s and left it 0.044 m behind while driving at 0.3 m/s. See [dev-journey.md](../dev-journey.md).

## Motion thresholds and the map update gate

`motion_threshold_trans` (0.05 m) and `motion_threshold_rot` (0.05 rad) decide when a scan is used. `map_update_dist` (0.20 m) and `map_update_angle` (0.25 rad) decide when the map is written.

**What I measured.** Offline with `filter_run_dump`, exact odometry, 0.1 m cells, seed 42, writing the map at every update (the distance gate set to 0.0001 m) made it worse with 180 points (0.0341 against 0.0098 m, mean error in x -0.033 m) and made no change with 360 (0.0072 against 0.0075 m). I have not measured it at 0.05 m cells with the tool.

**What I did not measure.** The cost in time of a lower gate, and the effect of the motion thresholds on accuracy. Lower thresholds update more often and cost more time. I know that from the code, not from a run.

## Resampling

`obs_sigma_gain` flattens the weights. In one offline probe (synthetic rectangular room, 180 points, 1 cm range noise, a 24 m circle of 400 updates, 80 particles, seed 42) the filter resampled 2 times and the lowest effective sample size I read after an update was 40.1. A resample resets it first, so the true minimum was lower and I did not record it. In one simulator run the lowest logged value was 40.0. I have not repeated this with 360 points and I have not measured other gains.
