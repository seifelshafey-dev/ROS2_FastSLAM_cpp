# The core: grid, scan matcher, particle filter

The core is plain C++17 with Eigen. Only `slam_node.cpp` includes a ROS header. The test paths below are relative to [`../../src/slam_cpp/test/`](../../src/slam_cpp/test/).

## Conventions

- A pose is `(x, y, theta)` in the map frame. Metres and radians.
- `normalize_angle` in [`slam_types.hpp`](../../src/slam_cpp/include/slam_cpp/slam_types.hpp) returns an angle in `[-pi, pi)`. An input of exactly `pi` returns `-pi`. Checked by `SlamTypes.NormalizeAnglePiReturnsMinusPi` and three more `SlamTypes.NormalizeAngle*` tests (`slam_types_test.cpp`).
- Scan points are in the base frame, in metres. The lidar is assumed to sit at the `base_link` origin with no rotation.
- The map frame is the robot pose at the first update. Every particle starts at `(0, 0, 0)`.

## Occupancy grid

Source: [`occupancy_grid.hpp`](../../src/slam_cpp/include/slam_cpp/occupancy_grid.hpp), [`occupancy_grid.cpp`](../../src/slam_cpp/src/occupancy_grid.cpp).

**Indexing.** The grid is square and centred on the map origin. A point maps to the cell `(col, row) = (floor(x / res) + cols / 2, floor(y / res) + rows / 2)`. A cell covers `[i, i + 1)` in cell units, so `x = -0.05 m` at 0.1 m cells lies in the cell left of the origin cell. Row is y, column is x, storage is row-major. A point outside the grid gives `std::nullopt`. Checked by `OccupancyGridTest.WorldToGridCenter` and `OccupancyGridTest.WorldToGridOutOfBounds` (`occupancy_grid_test.cpp`).

**Cell value.** A cell is an `int8` equal to `log-odds * log_odds_scale`, clamped to `[lo_min, lo_max] * log_odds_scale`. Zero means unknown. Each hit adds `lo_occupied`, each ray passing through a cell adds `lo_free`, both scaled and rounded. Checked by `OccupancyGridTest.LogOddsClamping`.

**Ray casting.** `update` takes a pose and the scan points in the base frame. For each point it moves the point to the map frame, finds its cell, and walks a Bresenham line from the robot's cell to that cell. Cells on the line get the free increment and the end cell gets the occupied increment. A point whose cell is off the grid is skipped. If the robot's own cell is off the grid, nothing is written. Checked by `OccupancyGridTest.BresenhamDrawsCorrectLine` and `OccupancyGridTest.UpdateIntegratesFullScan`.

A beam that returns nothing (range at or beyond `range_max`, or not finite) never becomes a scan point, so it writes nothing, not even free cells. See [limits.md](limits.md).

**Copy on write.** A copy of a grid shares its cells. `update` is the only writer and makes the cells unique first. The filter copies grids at resampling, so this keeps a resample cheap. Checked by `OccupancyGridTest.COWSharesUntilWrite`.

**Map quality.** `map_quality()` is the number of cells above `+log_odds_scale` stored units (log-odds 1) divided by the number of cells beyond either `+log_odds_scale` or `-log_odds_scale`. It is 0 when no cell is confident. The matcher uses it to choose how wide to search. Checked by `OccupancyGridTest.MapQualityIncrementalCounters`.

## Scan matcher

Source: [`scan_matching.hpp`](../../src/slam_cpp/include/slam_cpp/scan_matching.hpp), [`scan_matching.cpp`](../../src/slam_cpp/src/scan_matching.cpp). The functions take a `GridView`, a read-only view of one grid (see [`slam_types.hpp`](../../src/slam_cpp/include/slam_cpp/slam_types.hpp)).

**Reading the map.** A cell's value is taken to sit at the cell centre. The view's `sample_origin` is `cols / 2 - 0.5`, and a point in metres becomes the sample coordinate `s = x / res + sample_origin`. The occupancy probability at a point is the bilinear blend of the four cells whose centres surround it, each converted with a sigmoid lookup table ([`sigmoid_lut.hpp`](../../src/slam_cpp/include/slam_cpp/sigmoid_lut.hpp)). A point is usable only if all four cells are on the grid. Checked by `ScanMatchingTest.InterpolationAtCellCentreFromWorldReturnsCellValue`, `ScanMatchingTest.InterpolationOnCellBoundaryFromWorldIsMeanOfCells` and `ScanMatchingProperty.WrittenHitIsReadAtCellCentre` (`scan_matching_test.cpp`, `scan_matching_property_test.cpp`).

**Scan points.** `extract_local_scan` keeps every `stride`-th beam from index 0. A beam is kept when its range is finite, above zero, at least `range_min` and below `range_max`. Its point is `r * (cos a, sin a)` with `a = angle_min + i * angle_increment`. Checked by the `ScanMatchingTest.ExtractLocalScan*` tests and `ScanMatchingProperty.ExtractLocalScanKeepsExactlyInWindowBeams`.

**Cost of a pose.** For scan point `k`, with `p_k` the interpolated occupancy probability at its place in the map frame:

- residual `r_k = (1 - p_k) / sigma_hit`;
- scan term `0.5 * sum(r_k^2)`;
- odometry term `0.5 * d^T * odom_info * d`, with `d = pose - odom_pose` and the angle part wrapped to `[-pi, pi)`.

The joint cost is the sum of the two terms. Lower is better. A point with any of its four cells off the grid is charged the residual of an unknown cell, `0.5 / sigma_hit`. Checked by `ScanMatchingTest.JointCostChargesOffGridPointAsUnknownCell` and `ScanMatchingProperty.InGridCostIsSumOfInterpolatedResiduals`.

**Coarse search.** `coarse_search` tries poses around the odometry pose and keeps the one with the highest summed occupancy probability on a thinned copy of the scan. It ignores the odometry term. The window depends on `map_quality`:

| Map quality | Shifts in x and y (cells of `map_resolution`) | Turns (rad) | Scan thinning |
|---|---|---|---|
| below 0.2 | -3 to +3 | 0, +0.1, -0.1 | every 5th point |
| 0.2 or more | -1, 0, +1 | 0, +0.05, -0.05 | every 3rd point |

Once `map_quality` is above 0.01, a candidate with fewer than 20 percent of the thinned points on the grid is skipped. If every candidate is skipped the odometry pose is returned. Checked by `ScanMatchingTest.CoarseSearchFindsCorrectPose`.

**Refinement.** `scan_match_single` compares the joint cost of the coarse result with that of the odometry pose and starts `lm_optimize` from the lower. `lm_optimize` runs Levenberg-Marquardt on the joint cost. Each iteration builds residuals and a Jacobian over the points that are on the grid, forms `H = J^T J + odom_info`, and solves `(H + lambda * trace(H) / 3 * I) * delta = -b`. A step is kept if it lowers the joint cost over the same points. Damping `lambda` goes down by `lm_lambda_down` after a kept step, not below `lm_lambda_min`, and up by `lm_lambda_up` after a rejected step, capped at `lm_lambda_max`. After a failed solve or a non-finite step it is multiplied by `lm_lambda_up` with no cap. It stops after `lm_max_iters`, when no damped step lowers the cost, or when the step is under 1e-4. It fails only when fewer than 10 scan points are on the grid at the pose being linearised. Checked by `ScanMatchingTest.LmFromTruePoseOnOwnMapSucceeds`, `ScanMatchingTest.LmHessianIsEvaluatedAtReturnedPose` and `ScanMatchingProperty.MatchIsNeverWorseThanOdometry`.

**Prior.** `scan_match_single` takes the particle's predicted pose and its covariance. It raises each diagonal entry of the covariance to at least 0.01 and inverts it to get `odom_info`. With that floor the prior is never tighter than a standard deviation of 0.1 in each of x (m), y (m) and theta (rad). If the floored covariance cannot be inverted, the information matrix is `10 * I`. No test targets the floor or this fallback on its own.

**Proposal.** After refinement the proposal covariance is the inverse of `H` plus `1e-5` on the diagonal. If the match fails, the pose stays at the predicted pose and the covariance is `1e-8` on the diagonal. The covariance is also `1e-8` on the diagonal when `H` is not positive definite.

**Weight.** `calculate_log_weight` takes every `score_stride`-th scan point, sums `log(p)` with each `p` limited to `[1e-9, 1]`, and multiplies by `1 / (obs_sigma_gain * num_particles)`. A point with a surrounding cell off the grid counts as `1e-9`. This differs from the joint cost, which charges such a point as an unknown cell. See [limits.md](limits.md). No test checks the weight value on its own.

## Particle filter

Source: [`particle_filter.hpp`](../../src/slam_cpp/include/slam_cpp/particle_filter.hpp), [`particle_filter.cpp`](../../src/slam_cpp/src/particle_filter.cpp).

Both constructors call `validate` and throw `std::invalid_argument` on a bad configuration. Checked by `SlamConfig.BothParticleFilterConstructorsValidate` (`slam_config_test.cpp`).

**Motion model.** `predict(trans, rot1, rot2, direction)` moves every particle by a turn, a move and a turn, each with noise. With `alpha1` to `alpha4` from the configuration, the noise standard deviations are:

- first turn: `alpha1 * |rot1| + alpha2 * trans`;
- move: `alpha3 * trans + alpha4 * (|rot1| + |rot2|)`;
- second turn: `alpha1 * |rot2| + alpha2 * trans`.

Each particle's covariance is propagated through the motion Jacobians. The model follows the odometry motion model in Probabilistic Robotics (Thrun, Burgard, Fox), chapter 5. The alphas here are standard-deviation coefficients, not the book's variance coefficients. Checked by `ParticleFilterTest.PredictZeroNoiseExactMotion`, `ParticleFilterTest.PredictWithNoiseSpreadsPoses`, `ParticleFilterTest.PredictCovarianceGrows` and `ParticleFilterTest.PredictReverseDirection` (`particle_filter_test.cpp`).

**First update.** The first call to `update_and_resample` writes the scan into every grid at the particle's current pose. It does not match, and poses and weights are unchanged. Checked by `ParticleFilterTest.FirstUpdateRegistersScanWithoutMovingParticles`.

**Later updates**, in this order:

1. Matching. For each particle, `scan_match_single` gives a refined pose, a proposal covariance and a log weight. Map quality is cached and starts at 0 for every particle. The refresh counter starts at -5 and the first call counts as one update, so with the default `quality_cache_interval` of 5 the cache is refreshed at the second call, before the first match, from the map of the one registered scan. It is then refreshed every `quality_cache_interval` updates (the seventh call, and so on). The 20 percent rule applies whenever the cached quality is above 0.01. This loop reads shared data and writes only its own particle's slots, so it runs in parallel with OpenMP.
2. Sampling. A new pose is drawn for each particle from a Gaussian around its refined pose, using a Cholesky factor of the proposal covariance. If the covariance is not positive definite, each axis is drawn on its own from the diagonal. All random numbers are drawn on the calling thread from one seeded generator, so a seed gives the same result for any thread count. Checked by `ParticleFilterTest.SameSeedSameResultAcrossThreadCounts`, which only builds when OpenMP is found.
3. Weights. Each weight is multiplied by `exp(log_w - max log_w)` and the weights are normalised. If their sum is below 1e-200 or not finite they are reset to equal weights.
4. Map update decision. Before resampling, the filter takes the best particle, using the cumulative log weights as updated in step 3, and compares its sampled pose with the pose remembered at the last write. The grids are written only if that pose is more than `map_update_dist` away or turned more than `map_update_angle`. Otherwise the scan is not added to any map.
5. Resampling. If the effective sample size `1 / sum(w^2)` is below half the particle count, systematic resampling picks the survivors. Weights then reset to equal. Surviving particles share grids through copy on write. If step 4 said to write, only one copy per surviving ancestor is written, before the copies are made.
6. Remembered pose. After a write, the filter remembers the pose of the best particle that survived resampling.

No test targets the sampling, the map update gate or the resampling step on its own. `ParticleFilterIntegration.CircleWithExactOdometryTracksTruth` (`particle_filter_integration_test.cpp`) runs the whole sequence on a synthetic room and checks the pose error against a bound.

**Best particle.** Each particle carries a cumulative log weight, summed along its ancestry and inherited through resampling. The best particle is the one with the largest. Among copies of one ancestor the lowest index wins. Checked by `ParticleFilterTest.BestParticleSurvivesResampling` and `ParticleFilterTest.BestParticleIsACopyOfTheHeaviestAncestorAfterResampling`.
