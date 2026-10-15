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

#include "slam_cpp/slam_config.hpp"

#include <cmath>
#include <sstream>
#include <string>

#include "slam_cpp/check.hpp"

namespace slam
{

namespace
{

// "name must be <rule>, got <value>"
template<typename T>
std::string msg(const char * name, const char * rule, const T & value)
{
  std::ostringstream os;
  os << name << " must be " << rule << ", got " << value;
  return os.str();
}

}  // namespace

void validate(const SlamConfig & c)
{
  SLAM_CHECK(c.num_particles >= 1, msg("num_particles", ">= 1", c.num_particles));
  SLAM_CHECK(c.scan_stride >= 1, msg("scan_stride", ">= 1", c.scan_stride));
  SLAM_CHECK(c.score_stride >= 1, msg("score_stride", ">= 1", c.score_stride));
  SLAM_CHECK(c.lm_max_iters >= 1, msg("lm_max_iters", ">= 1", c.lm_max_iters));
  SLAM_CHECK(c.lm_max_sub_iters >= 1, msg("lm_max_sub_iters", ">= 1", c.lm_max_sub_iters));
  SLAM_CHECK(
    c.quality_cache_interval >= 1,
    msg("quality_cache_interval", ">= 1", c.quality_cache_interval));

  SLAM_CHECK(std::isfinite(c.map_size), msg("map_size", "finite", c.map_size));
  SLAM_CHECK(std::isfinite(c.map_resolution), msg("map_resolution", "finite", c.map_resolution));
  SLAM_CHECK(std::isfinite(c.log_odds_scale), msg("log_odds_scale", "finite", c.log_odds_scale));
  SLAM_CHECK(std::isfinite(c.lo_occupied), msg("lo_occupied", "finite", c.lo_occupied));
  SLAM_CHECK(std::isfinite(c.lo_free), msg("lo_free", "finite", c.lo_free));
  SLAM_CHECK(std::isfinite(c.lo_max), msg("lo_max", "finite", c.lo_max));
  SLAM_CHECK(std::isfinite(c.lo_min), msg("lo_min", "finite", c.lo_min));
  SLAM_CHECK(std::isfinite(c.alpha1), msg("alpha1", "finite", c.alpha1));
  SLAM_CHECK(std::isfinite(c.alpha2), msg("alpha2", "finite", c.alpha2));
  SLAM_CHECK(std::isfinite(c.alpha3), msg("alpha3", "finite", c.alpha3));
  SLAM_CHECK(std::isfinite(c.alpha4), msg("alpha4", "finite", c.alpha4));
  SLAM_CHECK(std::isfinite(c.sigma_hit), msg("sigma_hit", "finite", c.sigma_hit));
  SLAM_CHECK(std::isfinite(c.lm_lambda_init), msg("lm_lambda_init", "finite", c.lm_lambda_init));
  SLAM_CHECK(std::isfinite(c.lm_lambda_min), msg("lm_lambda_min", "finite", c.lm_lambda_min));
  SLAM_CHECK(std::isfinite(c.lm_lambda_max), msg("lm_lambda_max", "finite", c.lm_lambda_max));
  SLAM_CHECK(std::isfinite(c.lm_lambda_up), msg("lm_lambda_up", "finite", c.lm_lambda_up));
  SLAM_CHECK(std::isfinite(c.lm_lambda_down), msg("lm_lambda_down", "finite", c.lm_lambda_down));
  SLAM_CHECK(
    std::isfinite(c.motion_threshold_trans),
    msg("motion_threshold_trans", "finite", c.motion_threshold_trans));
  SLAM_CHECK(
    std::isfinite(c.motion_threshold_rot),
    msg("motion_threshold_rot", "finite", c.motion_threshold_rot));
  SLAM_CHECK(std::isfinite(c.map_update_dist), msg("map_update_dist", "finite", c.map_update_dist));
  SLAM_CHECK(
    std::isfinite(c.map_update_angle), msg("map_update_angle", "finite", c.map_update_angle));
  SLAM_CHECK(std::isfinite(c.obs_sigma_gain), msg("obs_sigma_gain", "finite", c.obs_sigma_gain));

  SLAM_CHECK(c.map_resolution > 0.0, msg("map_resolution", "> 0", c.map_resolution));
  SLAM_CHECK(c.map_size > 0.0, msg("map_size", "> 0", c.map_size));
  const double n = c.map_size / c.map_resolution;
  SLAM_CHECK(
    n >= 2.0 - 1e-6 && n <= 20000.0 + 1e-6,
    msg("map_size / map_resolution", "between 2 and 20000 cells", n));
  const int64_t cells = std::lround(n);
  SLAM_CHECK(
    std::fabs(n - static_cast<double>(cells)) <= 1e-6,
    msg("map_size / map_resolution", "a whole number of cells", n));
  SLAM_CHECK(cells % 2 == 0, msg("map_size / map_resolution", "an even number of cells", n));

  SLAM_CHECK(c.log_odds_scale > 0.0, msg("log_odds_scale", "> 0", c.log_odds_scale));
  SLAM_CHECK(c.log_odds_scale <= 127.0, msg("log_odds_scale", "<= 127", c.log_odds_scale));
  SLAM_CHECK(c.lo_free < 0.0, msg("lo_free", "< 0", c.lo_free));
  SLAM_CHECK(c.lo_occupied > 0.0, msg("lo_occupied", "> 0", c.lo_occupied));
  SLAM_CHECK(c.lo_min < 0.0, msg("lo_min", "< 0", c.lo_min));
  SLAM_CHECK(c.lo_max > 0.0, msg("lo_max", "> 0", c.lo_max));
  // |value * scale| < 127.5 rounds into [-127, 127]; NaN fails the comparison.
  SLAM_CHECK(
    std::fabs(c.lo_occupied * c.log_odds_scale) < 127.5,
    msg(
      "lo_occupied * log_odds_scale", "within [-127, 127] after rounding",
      c.lo_occupied * c.log_odds_scale));
  SLAM_CHECK(
    std::fabs(c.lo_free * c.log_odds_scale) < 127.5,
    msg(
      "lo_free * log_odds_scale", "within [-127, 127] after rounding",
      c.lo_free * c.log_odds_scale));
  SLAM_CHECK(
    std::fabs(c.lo_max * c.log_odds_scale) < 127.5,
    msg(
      "lo_max * log_odds_scale", "within [-127, 127] after rounding",
      c.lo_max * c.log_odds_scale));
  SLAM_CHECK(
    std::fabs(c.lo_min * c.log_odds_scale) < 127.5,
    msg(
      "lo_min * log_odds_scale", "within [-127, 127] after rounding",
      c.lo_min * c.log_odds_scale));
  SLAM_CHECK(
    c.lo_max_scaled() >= 1,
    msg("lo_max * log_odds_scale", "rounded to at least 1", c.lo_max * c.log_odds_scale));
  SLAM_CHECK(
    c.lo_min_scaled() <= -1,
    msg("lo_min * log_odds_scale", "rounded to at most -1", c.lo_min * c.log_odds_scale));
  SLAM_CHECK(
    c.lo_occupied_scaled() != 0,
    msg(
      "lo_occupied * log_odds_scale", "at least 0.5 so it does not round to 0",
      c.lo_occupied * c.log_odds_scale));
  SLAM_CHECK(
    c.lo_free_scaled() != 0,
    msg(
      "lo_free * log_odds_scale", "at most -0.5 so it does not round to 0",
      c.lo_free * c.log_odds_scale));

  SLAM_CHECK(c.alpha1 >= 0.0, msg("alpha1", ">= 0", c.alpha1));
  SLAM_CHECK(c.alpha2 >= 0.0, msg("alpha2", ">= 0", c.alpha2));
  SLAM_CHECK(c.alpha3 >= 0.0, msg("alpha3", ">= 0", c.alpha3));
  SLAM_CHECK(c.alpha4 >= 0.0, msg("alpha4", ">= 0", c.alpha4));
  SLAM_CHECK(c.sigma_hit > 0.0, msg("sigma_hit", "> 0", c.sigma_hit));
  SLAM_CHECK(c.obs_sigma_gain > 0.0, msg("obs_sigma_gain", "> 0", c.obs_sigma_gain));

  SLAM_CHECK(c.lm_lambda_min > 0.0, msg("lm_lambda_min", "> 0", c.lm_lambda_min));
  SLAM_CHECK(
    c.lm_lambda_init >= c.lm_lambda_min,
    msg("lm_lambda_init", ">= lm_lambda_min", c.lm_lambda_init));
  SLAM_CHECK(
    c.lm_lambda_max >= c.lm_lambda_init,
    msg("lm_lambda_max", ">= lm_lambda_init", c.lm_lambda_max));
  SLAM_CHECK(c.lm_lambda_up > 1.0, msg("lm_lambda_up", "> 1", c.lm_lambda_up));
  SLAM_CHECK(
    c.lm_lambda_down > 0.0 && c.lm_lambda_down < 1.0,
    msg("lm_lambda_down", "between 0 and 1 (exclusive)", c.lm_lambda_down));

  SLAM_CHECK(
    c.motion_threshold_trans >= 0.0,
    msg("motion_threshold_trans", ">= 0", c.motion_threshold_trans));
  SLAM_CHECK(
    c.motion_threshold_rot >= 0.0,
    msg("motion_threshold_rot", ">= 0", c.motion_threshold_rot));
  SLAM_CHECK(c.map_update_dist >= 0.0, msg("map_update_dist", ">= 0", c.map_update_dist));
  SLAM_CHECK(c.map_update_angle >= 0.0, msg("map_update_angle", ">= 0", c.map_update_angle));
}

}  // namespace slam
