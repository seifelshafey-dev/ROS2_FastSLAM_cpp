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

#include <gtest/gtest.h>
#include <rapidcheck/gtest.h>

#include <exception>
#include <string>

#include "slam_cpp/slam_config.hpp"

namespace
{

// Integer in [lo, hi] times unit.
rc::Gen<double> scaled(int lo, int hi, double unit)
{
  return rc::gen::map(rc::gen::inRange(lo, hi + 1), [unit](int v) {return v * unit;});
}

}  // namespace

// A config drawn inside every valid range never throws.
RC_GTEST_PROP(SlamConfigProperty, ConfigInsideValidRangesIsAccepted, ())
{
  slam::SlamConfig c;
  c.num_particles = *rc::gen::inRange(1, 301);
  c.scan_stride = *rc::gen::inRange(1, 11);
  c.score_stride = *rc::gen::inRange(1, 11);
  c.lm_max_iters = *rc::gen::inRange(1, 51);
  c.lm_max_sub_iters = *rc::gen::inRange(1, 11);
  c.quality_cache_interval = *rc::gen::inRange(1, 21);

  // Map size is an even cell count times the resolution.
  const double res = *scaled(1, 50, 0.01);
  const int cells = 2 * *rc::gen::inRange(1, 2001);
  c.map_resolution = res;
  c.map_size = cells * res;

  // Each log-odds value is chosen as a whole scaled step divided by the scale,
  // so it rounds back to that step: in [1, 127] and never to 0.
  c.log_odds_scale = *scaled(1, 127, 1.0);
  c.lo_occupied = *rc::gen::inRange(1, 128) / c.log_odds_scale;
  c.lo_free = -*rc::gen::inRange(1, 128) / c.log_odds_scale;
  c.lo_max = *rc::gen::inRange(1, 128) / c.log_odds_scale;
  c.lo_min = -*rc::gen::inRange(1, 128) / c.log_odds_scale;

  c.alpha1 = *scaled(0, 1000, 1e-3);
  c.alpha2 = *scaled(0, 1000, 1e-3);
  c.alpha3 = *scaled(0, 1000, 1e-3);
  c.alpha4 = *scaled(0, 1000, 1e-3);
  c.sigma_hit = *scaled(1, 500, 1e-3);
  c.obs_sigma_gain = *scaled(1, 100, 0.1);

  c.lm_lambda_min = *scaled(1, 1000, 1e-9);
  c.lm_lambda_init = c.lm_lambda_min * (1.0 + *scaled(0, 1000, 1.0));
  c.lm_lambda_max = c.lm_lambda_init * (1.0 + *scaled(0, 1000, 1.0));
  c.lm_lambda_up = 1.0 + *scaled(1, 100, 0.1);
  c.lm_lambda_down = *scaled(1, 99, 0.01);

  c.motion_threshold_trans = *scaled(0, 1000, 1e-3);
  c.motion_threshold_rot = *scaled(0, 1000, 1e-3);
  c.map_update_dist = *scaled(0, 1000, 1e-3);
  c.map_update_angle = *scaled(0, 1000, 1e-3);

  try {
    slam::validate(c);
  } catch (const std::exception & e) {
    RC_FAIL(std::string("valid config rejected: ") + e.what());
  }
}
