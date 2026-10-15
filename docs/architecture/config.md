# Configuration

Every setting is a field of `SlamConfig` in [`slam_config.hpp`](../../src/slam_cpp/include/slam_cpp/slam_config.hpp). The rules are in `validate`, [`slam_config.cpp`](../../src/slam_cpp/src/slam_config.cpp). The node exposes each field as a ROS parameter of the same name. The parameter table with units and ranges is in the class comment of `SlamNode` ([`slam_node.cpp`](../../src/slam_cpp/src/ros/slam_node.cpp)). The shipped values are in [`slam_params.yaml`](../../src/slam_cpp/config/slam_params.yaml) and equal the defaults below.

`validate` throws `std::invalid_argument` naming the field, its value and the rule. Every floating-point field must be finite, so NaN and infinity are rejected. Both `ParticleFilter` constructors and the `OccupancyGrid` constructor call it. The node does not call it itself; it builds the filter at start, so the check runs there. Checked by the `SlamConfig.Rejects*` tests, `SlamConfig.RejectsNonFiniteInEveryDoubleField`, `SlamConfig.LogOddsExtremaMustRoundToAtLeastOneCell`, `SlamConfig.LogOddsExtremaProductBoundIs127Point5`, `SlamConfig.OccupancyGridValidatesConfig` and `SlamConfigProperty.ConfigInsideValidRangesIsAccepted` (`../../src/slam_cpp/test/slam_config_test.cpp`, `slam_config_property_test.cpp`).

What each setting trades is below. A range here is what `validate` accepts. It does not say the extreme is a good choice.

## Particles

| Name | Default | Range | What it trades |
|---|---|---|---|
| `num_particles` | 80 | 1 or more | More particles keep more hypotheses alive. Matching runs once per particle, and each particle holds a grid. |

## Map

| Name | Unit | Default | Range | What it trades |
|---|---|---|---|---|
| `map_size` | m | 50.0 | above 0 | Side of the square map, centred on the start. The mapped area must fit inside, see [limits.md](limits.md). Memory per particle grows with the square of the side. |
| `map_resolution` | m per cell | 0.1 | above 0 | Smaller cells give a finer map but need more scan points per metre of wall. See `scan_stride`. |

`map_size / map_resolution` must be a whole, even number of cells from 2 to 20000. One cell is one byte, so the default 500 by 500 grid is 250 KB per particle before sharing.

## Cell values

| Name | Default | Range |
|---|---|---|
| `log_odds_scale` | 20.0 | above 0, at most 127 |
| `lo_occupied` | 0.85 | above 0 |
| `lo_free` | -0.4 | below 0 |
| `lo_max` | 5.0 | above 0 |
| `lo_min` | -5.0 | below 0 |

A cell stores `log-odds * log_odds_scale` as an `int8`. The four log-odds values times the scale must round into `[-127, 127]`, `lo_occupied` and `lo_free` times the scale must not round to 0, `lo_max` times the scale must round to at least 1, and `lo_min` times the scale to at most -1 (otherwise no cell could reach the clamp). A larger scale resolves smaller log-odds steps but leaves less headroom before the clamp. `lo_max` and `lo_min` decide how many consistent observations it takes to overturn a cell: at the defaults the limits are 100 and -100 stored units and a free observation is -8, so a cell at the occupied limit goes below zero after 13 free observations and reaches the free limit after 25.

## Motion noise

| Name | Unit | Default | Range |
|---|---|---|---|
| `alpha1` | none | 0.075 | 0 or more |
| `alpha2` | rad per m | 0.03 | 0 or more |
| `alpha3` | none | 0.075 | 0 or more |
| `alpha4` | m per rad | 0.03 | 0 or more |

These scale the noise added to each particle's motion. The formulas are in [core.md](core.md). `alpha1` and `alpha2` set the spread of the turns, `alpha3` and `alpha4` the spread of the move. Larger values spread the particles more. I have not measured the effect on accuracy, or a best value for any robot.

## Scan matching and weights

| Name | Unit | Default | Range | What it trades |
|---|---|---|---|---|
| `sigma_hit` | none | 0.05 | above 0 | Scale of the scan residual in the matcher. Smaller values trust the map over the odometry prior. The particle weight does not use it. |
| `score_stride` | points | 2 | 1 or more | Weight uses every n-th scan point. Larger scores fewer points, so it is faster. I have not measured its effect on accuracy. |
| `lm_max_iters` | iterations | 10 | 1 or more | Cap on refinement steps. |
| `lm_max_sub_iters` | tries | 3 | 1 or more | Cap on damping retries within one step. |
| `lm_lambda_init` | none | 0.01 | `lm_lambda_min` to `lm_lambda_max` | Starting damping. |
| `lm_lambda_min` | none | 1e-7 | above 0 | Smallest damping. |
| `lm_lambda_max` | none | 1e4 | `lm_lambda_init` or more | Largest damping. |
| `lm_lambda_up` | factor | 10.0 | above 1 | Damping growth after a rejected step. |
| `lm_lambda_down` | factor | 0.1 | between 0 and 1, exclusive | Damping shrink after an accepted step. |
| `obs_sigma_gain` | none | 3.0 | above 0 | Log weights are divided by `obs_sigma_gain * num_particles`. A larger value flattens the weights, so the filter resamples less. |

On resampling: with the default gain the filter does resample. In an offline probe (synthetic rectangular room, 180 points, 1 cm range noise, a 24 m circle of 400 updates, 80 particles) the lowest effective sample size I read after an update was 40 and it resampled 2 or 3 times. A resample resets the weights before I read it, so the true minimum was lower and I did not record it. In one simulator run the lowest logged value was also 40.0. I have not repeated this with 360 points.

## Update gates

| Name | Unit | Default | Range | What it trades |
|---|---|---|---|---|
| `motion_threshold_trans` | m | 0.05 | 0 or more | Move since the last update that triggers a new one. The node applies it. |
| `motion_threshold_rot` | rad | 0.05 | 0 or more | Turn since the last update that triggers a new one. The node applies it. |
| `map_update_dist` | m | 0.20 | 0 or more | The map is written only after the best pose has moved this far from the last write. |
| `map_update_angle` | rad | 0.25 | 0 or more | The same, for turning. |

Lower thresholds update more often and cost more time. The map gate saves the time of writing every map at every update. While the robot has not moved past the gate, the map stays as it was, which is why a thin first map can hold the pose at the start. See [limits.md](limits.md).

## Scan and cache

| Name | Unit | Default | Range | What it trades |
|---|---|---|---|---|
| `scan_stride` | beams | 1 | 1 or more | Use every n-th beam. Applied by `extract_local_scan`, not by the filter. |
| `quality_cache_interval` | updates | 5 | 1 or more | How often each particle's map quality is recomputed. |

`scan_stride` and `map_resolution` are linked through the gap between neighbouring scan points on a wall: the arc length at the wall's range, which is range times the angle between kept beams. The node warns once when that gap, at the median range of the first used scan, exceeds 2 cells. The bound comes from offline runs in empty rectangular rooms (details in [limits.md](limits.md)); it is a guide, not a law.

Cost of thinning, measured with `filter_run_dump`, Release -O3, OpenMP on, 12 cores, 80 particles, 0.1 m cells, 714 updates, whole-program wall time (scan generation and CSV writing included), on an otherwise idle machine: 3.5 to 3.8 s with 180 points and 6.0 to 6.2 s with 360, two runs each, about 5 ms and 8.5 ms per update.

## Node-only settings

`odom_topic`, `scan_topic`, `map_frame`, `odom_frame` and `map_publish_rate` are not in `SlamConfig`. See [ros.md](ros.md).

## Fixed in the code

These are constants, not settings:

- the floor of 0.01 on each diagonal entry of the prior covariance;
- the coarse search windows and the 20 percent rule;
- the minimum of 10 scan points on the grid for a match to succeed;
- the clamp of 1e-9 on a point's probability in the weight;
- the effective sample size threshold of half the particle count.

They are described in [core.md](core.md).
