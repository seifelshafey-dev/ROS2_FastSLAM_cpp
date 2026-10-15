// Copyright 2026 Seif ElShafey
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SLAM_CPP__SLAM_CONFIG_HPP_
#define SLAM_CPP__SLAM_CONFIG_HPP_

/**
 * @file slam_config.hpp
 * @brief The SLAM configuration struct and its validation.
 */

#include <cmath>
#include <cstdint>

namespace slam
{

/// All tunable SLAM parameters. Check a configuration with `validate` before use; the
/// ranges below are the ones `validate` enforces.
struct SlamConfig
{
  // Particles
  /// Number of particles; >= 1. Default 80.
  int num_particles = 80;

  // Occupancy grid
  /// Side length of the square map in metres; > 0. `map_size / map_resolution` must be an
  /// even whole number of cells between 2 and 20000. Default 50.0.
  double map_size = 50.0;
  /// Edge length of one map cell in metres; > 0. Default 0.1.
  double map_resolution = 0.1;

  // Log-odds scaling
  /// Factor by which log-odds are multiplied before being stored in an int8 cell (rounded
  /// to nearest); in (0, 127]. Default 20.0.
  double log_odds_scale = 20.0;
  /// Log-odds added to a cell for each hit; > 0. Default 0.85.
  double lo_occupied = 0.85;
  /// Log-odds added to a cell for each ray passing through it; < 0. Default -0.4.
  double lo_free = -0.4;
  /// Upper clamp of a cell's log-odds; > 0. Default 5.0.
  double lo_max = 5.0;
  /// Lower clamp of a cell's log-odds; < 0. Default -5.0.
  double lo_min = -5.0;
  // The four values above, times log_odds_scale and rounded, must lie in [-127, 127], and
  // lo_occupied and lo_free must not round to 0 (see the accessors below).

  // Motion model noise, after the odometry motion model of Probabilistic Robotics. The
  // alphas are standard-deviation coefficients, not the book's variance coefficients.
  // For a motion of translation `trans` and rotations `rot1`, `rot2`, the standard
  // deviation of each rotation is alpha1 * |rot| + alpha2 * trans (radians) and that of
  // the translation is alpha3 * trans + alpha4 * (|rot1| + |rot2|) (metres).
  /// Rotation noise per radian rotated (standard-deviation coefficient). Dimensionless;
  /// >= 0. Default 0.075.
  double alpha1 = 0.075;
  /// Rotation noise per metre translated (standard-deviation coefficient). Rad per m;
  /// >= 0. Default 0.03.
  double alpha2 = 0.03;
  /// Translation noise per metre translated (standard-deviation coefficient).
  /// Dimensionless; >= 0. Default 0.075.
  double alpha3 = 0.075;
  /// Translation noise per radian rotated (standard-deviation coefficient). M per rad;
  /// >= 0. Default 0.03.
  double alpha4 = 0.03;

  // Scan matching
  /// Standard deviation of the scan residual `1 - p`, where p is the interpolated
  /// occupancy probability at a scan point; dimensionless; > 0. Smaller values weigh map
  /// mismatch more against the odometry prior. Not used for the particle weight.
  /// Default 0.05.
  double sigma_hit = 0.05;
  /// Particle weighting uses every `score_stride`-th scan point of the scan passed to the
  /// filter (after `scan_stride` and invalid-beam removal); >= 1. Default 2.
  int score_stride = 2;
  // Levenberg-Marquardt
  /// Maximum Levenberg-Marquardt iterations per scan match; >= 1. Default 10.
  int lm_max_iters = 10;
  /// Maximum damping retries within one Levenberg-Marquardt iteration; >= 1. Default 3.
  int lm_max_sub_iters = 3;

  /// Initial Levenberg-Marquardt damping; in [lm_lambda_min, lm_lambda_max]. Default 0.01.
  double lm_lambda_init = 0.01;
  /// Smallest damping; > 0. Default 1e-7.
  double lm_lambda_min = 1e-7;
  /// Largest damping; >= lm_lambda_init. Default 1e4.
  double lm_lambda_max = 1e4;
  /// Factor by which damping is multiplied when a step is rejected; > 1. Default 10.0.
  double lm_lambda_up = 10.0;
  /// Factor by which damping is multiplied when a step is accepted; in (0, 1). Default 0.1.
  double lm_lambda_down = 0.1;

  // Motion thresholds for triggering a SLAM update
  /// Translation since the last update, in metres, that triggers a new SLAM update; >= 0.
  /// Applied by the node (or by a caller through extract_local_scan()); the filter itself
  /// does not read it. Default 0.05.
  double motion_threshold_trans = 0.05;
  /// Rotation since the last update, in radians, that triggers a new SLAM update; >= 0.
  /// Applied by the node (or by a caller through extract_local_scan()); the filter itself
  /// does not read it. Default 0.05.
  double motion_threshold_rot = 0.05;

  // Sparse map update gating
  /// Distance, in metres, the pose must move from the last map update before the
  /// map is updated again; >= 0. Default 0.20.
  double map_update_dist = 0.20;
  /// Rotation, in radians, the pose must turn from the last map update before the
  /// map is updated again; >= 0. Default 0.25.
  double map_update_angle = 0.25;

  /// Observation gain after Grisetti et al. (2007): log-likelihoods are scaled by
  /// 1 / (obs_sigma_gain * num_particles). Dimensionless; > 0. A larger value flattens the
  /// particle weights and makes weight collapse less likely. Default 3.0.
  double obs_sigma_gain = 3.0;

  // Scan and cache
  /// Use every `scan_stride`-th beam of each scan; >= 1. Applied by extract_local_scan(),
  /// which the node or a caller uses to build the scan; the filter itself does not read
  /// it. Thinning the scan saves time but widens the gap between scan points on distant
  /// walls; with points more than about two cells apart the first matches can fail.
  /// Default 1.
  int scan_stride = 1;
  /// The cached map quality is refreshed every `quality_cache_interval` updates; >= 1.
  /// Default 5.
  int quality_cache_interval = 5;

  // The four scaled values below are the log-odds field times `log_odds_scale`,
  // rounded to nearest. Valid once `validate` has accepted the configuration.
  /// `lo_occupied` as a stored cell value.
  ///
  /// @return `lo_occupied * log_odds_scale`, rounded to nearest, as an int8 cell value.
  int8_t lo_occupied_scaled() const
  {
    return static_cast<int8_t>(std::lround(lo_occupied * log_odds_scale));
  }
  /// `lo_free` as a stored cell value.
  ///
  /// @return `lo_free * log_odds_scale`, rounded to nearest, as an int8 cell value.
  int8_t lo_free_scaled() const
  {
    return static_cast<int8_t>(std::lround(lo_free * log_odds_scale));
  }
  /// `lo_max` as a stored cell value.
  ///
  /// @return `lo_max * log_odds_scale`, rounded to nearest, as an int8 cell value.
  int8_t lo_max_scaled() const
  {
    return static_cast<int8_t>(std::lround(lo_max * log_odds_scale));
  }
  /// `lo_min` as a stored cell value.
  ///
  /// @return `lo_min * log_odds_scale`, rounded to nearest, as an int8 cell value.
  int8_t lo_min_scaled() const
  {
    return static_cast<int8_t>(std::lround(lo_min * log_odds_scale));
  }
};

/**
 * @brief Reject a configuration that cannot run.
 *
 * Enforces:
 * - counts: `num_particles`, the strides, the LM iteration counts and
 *   `quality_cache_interval` are >= 1.
 * - map: `map_size` and `map_resolution` are > 0; `map_size / map_resolution` is an even
 *   whole number of cells, at most 20000.
 * - log-odds: 0 < `log_odds_scale` <= 127; `lo_free` < 0 < `lo_occupied`;
 *   `lo_min` < 0 < `lo_max`; every log-odds value times the scale rounds into [-127, 127],
 *   and the increments do not round to 0.
 * - noise and gain: `alpha1` to `alpha4` >= 0; `sigma_hit` and `obs_sigma_gain` > 0.
 * - LM damping: 0 < `lm_lambda_min` <= `lm_lambda_init` <= `lm_lambda_max`;
 *   `lm_lambda_up` > 1; 0 < `lm_lambda_down` < 1.
 * - thresholds: `motion_threshold_trans`, `motion_threshold_rot`, `map_update_dist` and
 *   `map_update_angle` >= 0.
 *
 * A NaN in any field is rejected.
 *
 * @param config Configuration to check.
 * @throws std::invalid_argument naming the field, its value and the rule that failed.
 */
void validate(const SlamConfig & config);

}  // namespace slam

#endif  // SLAM_CPP__SLAM_CONFIG_HPP_
