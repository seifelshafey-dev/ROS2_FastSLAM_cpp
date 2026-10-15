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

#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "slam_cpp/occupancy_grid.hpp"
#include "slam_cpp/particle_filter.hpp"
#include "slam_cpp/slam_config.hpp"

namespace
{

const double kNaN = std::numeric_limits<double>::quiet_NaN();
const double kInf = std::numeric_limits<double>::infinity();

// Every double field of SlamConfig, by name.
const std::vector<std::pair<const char *, double slam::SlamConfig::*>> kDoubleFields = {
  {"map_size", &slam::SlamConfig::map_size},
  {"map_resolution", &slam::SlamConfig::map_resolution},
  {"log_odds_scale", &slam::SlamConfig::log_odds_scale},
  {"lo_occupied", &slam::SlamConfig::lo_occupied},
  {"lo_free", &slam::SlamConfig::lo_free},
  {"lo_max", &slam::SlamConfig::lo_max},
  {"lo_min", &slam::SlamConfig::lo_min},
  {"alpha1", &slam::SlamConfig::alpha1},
  {"alpha2", &slam::SlamConfig::alpha2},
  {"alpha3", &slam::SlamConfig::alpha3},
  {"alpha4", &slam::SlamConfig::alpha4},
  {"sigma_hit", &slam::SlamConfig::sigma_hit},
  {"lm_lambda_init", &slam::SlamConfig::lm_lambda_init},
  {"lm_lambda_min", &slam::SlamConfig::lm_lambda_min},
  {"lm_lambda_max", &slam::SlamConfig::lm_lambda_max},
  {"lm_lambda_up", &slam::SlamConfig::lm_lambda_up},
  {"lm_lambda_down", &slam::SlamConfig::lm_lambda_down},
  {"motion_threshold_trans", &slam::SlamConfig::motion_threshold_trans},
  {"motion_threshold_rot", &slam::SlamConfig::motion_threshold_rot},
  {"map_update_dist", &slam::SlamConfig::map_update_dist},
  {"map_update_angle", &slam::SlamConfig::map_update_angle},
  {"obs_sigma_gain", &slam::SlamConfig::obs_sigma_gain},
};

// Starts from a valid default config, applies `breakit` (which must break
// exactly one rule) and expects validate to throw; `rule` names the case.
// Every string in `in_msg` must appear in the thrown message, which proves
// which rule fired.
void expect_rejected(
  const char * rule, const std::function<void(slam::SlamConfig &)> & breakit,
  const std::vector<std::string> & in_msg = {})
{
  slam::SlamConfig c;
  ASSERT_NO_THROW(slam::validate(c)) << "default config must be valid";
  breakit(c);
  try {
    slam::validate(c);
    ADD_FAILURE() << "rule not enforced: " << rule;
  } catch (const std::invalid_argument & e) {
    const std::string what = e.what();
    for (const auto & s : in_msg) {
      EXPECT_NE(what.find(s), std::string::npos) << rule << ": '" << s << "' not in: " << what;
    }
  }
}

}  // namespace

TEST(SlamConfig, DefaultConfigIsValid)
{
  EXPECT_NO_THROW(slam::validate(slam::SlamConfig{}));
}

TEST(SlamConfig, ShippedYamlValuesAreValid)
{
  // Hand-kept copy of the numeric values in config/slam_params.yaml
  // (slam_node.ros__parameters); it does not read the file, so update it with the YAML.
  slam::SlamConfig c;
  c.num_particles = 80;
  c.map_size = 50.0;
  c.map_resolution = 0.1;
  c.log_odds_scale = 20.0;
  c.lo_occupied = 0.85;
  c.lo_free = -0.4;
  c.lo_max = 5.0;
  c.lo_min = -5.0;
  c.alpha1 = 0.075;
  c.alpha2 = 0.03;
  c.alpha3 = 0.075;
  c.alpha4 = 0.03;
  c.sigma_hit = 0.05;
  c.score_stride = 2;
  c.lm_max_iters = 10;
  c.lm_max_sub_iters = 3;
  c.lm_lambda_init = 0.01;
  c.lm_lambda_min = 1.0e-7;
  c.lm_lambda_max = 1.0e4;
  c.lm_lambda_up = 10.0;
  c.lm_lambda_down = 0.1;
  c.motion_threshold_trans = 0.05;
  c.motion_threshold_rot = 0.05;
  c.map_update_dist = 0.20;
  c.map_update_angle = 0.25;
  c.obs_sigma_gain = 3.0;
  c.scan_stride = 1;
  c.quality_cache_interval = 5;
  EXPECT_NO_THROW(slam::validate(c));
}

TEST(SlamConfig, RejectsCountsBelowOne)
{
  expect_rejected("num_particles 0", [](auto & c) {c.num_particles = 0;});
  expect_rejected("scan_stride 0", [](auto & c) {c.scan_stride = 0;});
  expect_rejected("score_stride 0", [](auto & c) {c.score_stride = 0;});
  expect_rejected("lm_max_iters 0", [](auto & c) {c.lm_max_iters = 0;});
  expect_rejected("lm_max_sub_iters 0", [](auto & c) {c.lm_max_sub_iters = 0;});
  expect_rejected("quality_cache_interval 0", [](auto & c) {c.quality_cache_interval = 0;});
}

TEST(SlamConfig, RejectsBadMapGeometry)
{
  expect_rejected("map_resolution 0", [](auto & c) {c.map_resolution = 0.0;});
  expect_rejected("map_resolution negative", [](auto & c) {c.map_resolution = -0.1;});
  expect_rejected("map_size 0", [](auto & c) {c.map_size = 0.0;});
  expect_rejected("map_size negative", [](auto & c) {c.map_size = -50.0;}, {"map_size"});
  expect_rejected(
    "cell count not whole (10 / 0.3)", [](auto & c) {
      c.map_size = 10.0;
      c.map_resolution = 0.3;
    });
  // 100.4 cells: rounds to 100 (even, in range), so only the whole-number rule can fire.
  expect_rejected(
    "cell count not whole, even after rounding (10.04 / 0.1)",
    [](auto & c) {
      c.map_size = 10.04;
      c.map_resolution = 0.1;
    },
    {"a whole number of cells"});
  expect_rejected(
    "cell count odd (10.1 / 0.1)", [](auto & c) {
      c.map_size = 10.1;
      c.map_resolution = 0.1;
    });
  expect_rejected(
    "cell count below 2 (0.1 / 0.1)", [](auto & c) {
      c.map_size = 0.1;
      c.map_resolution = 0.1;
    });
  expect_rejected(
    "cell count above 20000 (2002 / 0.1)", [](auto & c) {
      c.map_size = 2002.0;
      c.map_resolution = 0.1;
    });
}

TEST(SlamConfig, RejectsBadLogOddsScale)
{
  expect_rejected("log_odds_scale 0", [](auto & c) {c.log_odds_scale = 0.0;});
  expect_rejected("log_odds_scale negative", [](auto & c) {c.log_odds_scale = -20.0;});
  expect_rejected("log_odds_scale 128", [](auto & c) {c.log_odds_scale = 128.0;});
  // Products 64, 64, 115.2, 115.2 all pass the 127.5 rule, so only the scale cap can fire.
  expect_rejected(
    "log_odds_scale 128 with small log-odds",
    [](auto & c) {
      c.log_odds_scale = 128.0;
      c.lo_occupied = 0.5;
      c.lo_free = -0.5;
      c.lo_max = 0.9;
      c.lo_min = -0.9;
    },
    {"log_odds_scale", "<= 127"});
}

TEST(SlamConfig, RejectsBadLogOddsSigns)
{
  expect_rejected("lo_occupied 0", [](auto & c) {c.lo_occupied = 0.0;});
  expect_rejected("lo_occupied negative", [](auto & c) {c.lo_occupied = -0.85;});
  expect_rejected("lo_free 0", [](auto & c) {c.lo_free = 0.0;});
  expect_rejected("lo_free positive", [](auto & c) {c.lo_free = 0.4;});
  expect_rejected("lo_max 0", [](auto & c) {c.lo_max = 0.0;});
  expect_rejected("lo_max negative", [](auto & c) {c.lo_max = -5.0;});
  expect_rejected("lo_min 0", [](auto & c) {c.lo_min = 0.0;});
  expect_rejected("lo_min positive", [](auto & c) {c.lo_min = 5.0;});
}

TEST(SlamConfig, RejectsLogOddsOutOfInt8RangeOrRoundingToZero)
{
  expect_rejected("lo_max * scale = 140", [](auto & c) {c.lo_max = 7.0; c.log_odds_scale = 20.0;});
  expect_rejected(
    "lo_min * scale = -140",
    [](auto & c) {c.lo_min = -7.0; c.log_odds_scale = 20.0;});
  expect_rejected("lo_occupied * scale = 140", [](auto & c) {c.lo_occupied = 7.0;});
  expect_rejected("lo_free * scale = -140", [](auto & c) {c.lo_free = -7.0;});
  expect_rejected("lo_free rounds to 0 (-0.01 * 20)", [](auto & c) {c.lo_free = -0.01;});
  expect_rejected("lo_occupied rounds to 0 (0.01 * 20)", [](auto & c) {c.lo_occupied = 0.01;});
}

TEST(SlamConfig, RejectsNegativeNoiseAndBadGains)
{
  expect_rejected("alpha1 negative", [](auto & c) {c.alpha1 = -0.01;});
  expect_rejected("alpha2 negative", [](auto & c) {c.alpha2 = -0.01;});
  expect_rejected("alpha3 negative", [](auto & c) {c.alpha3 = -0.01;});
  expect_rejected("alpha4 negative", [](auto & c) {c.alpha4 = -0.01;});
  expect_rejected("sigma_hit 0", [](auto & c) {c.sigma_hit = 0.0;});
  expect_rejected("sigma_hit negative", [](auto & c) {c.sigma_hit = -0.05;}, {"sigma_hit"});
  expect_rejected("obs_sigma_gain 0", [](auto & c) {c.obs_sigma_gain = 0.0;});
  expect_rejected(
    "obs_sigma_gain negative", [](auto & c) {c.obs_sigma_gain = -3.0;},
    {"obs_sigma_gain"});
}

TEST(SlamConfig, RejectsBrokenDampingChain)
{
  expect_rejected("lm_lambda_min 0", [](auto & c) {c.lm_lambda_min = 0.0;});
  expect_rejected(
    "lm_lambda_init < min", [](auto & c) {
      c.lm_lambda_min = 0.1;
      c.lm_lambda_init = 0.01;
    });
  expect_rejected(
    "lm_lambda_max < init", [](auto & c) {
      c.lm_lambda_init = 1.0;
      c.lm_lambda_max = 0.5;
    });
  expect_rejected("lm_lambda_up 1", [](auto & c) {c.lm_lambda_up = 1.0;});
  expect_rejected("lm_lambda_down 0", [](auto & c) {c.lm_lambda_down = 0.0;});
  expect_rejected("lm_lambda_down 1", [](auto & c) {c.lm_lambda_down = 1.0;});
}

TEST(SlamConfig, RejectsNegativeThresholds)
{
  expect_rejected(
    "motion_threshold_trans negative", [](auto & c) {
      c.motion_threshold_trans = -0.01;
    });
  expect_rejected("motion_threshold_rot negative", [](auto & c) {c.motion_threshold_rot = -0.01;});
  expect_rejected("map_update_dist negative", [](auto & c) {c.map_update_dist = -0.01;});
  expect_rejected("map_update_angle negative", [](auto & c) {c.map_update_angle = -0.01;});
}

// Each field is tested alone; "finite" in the message proves the finiteness rule fired
// (infinity passes every sign rule, so without that rule most of these would be accepted).
TEST(SlamConfig, RejectsNonFiniteInEveryDoubleField)
{
  ASSERT_EQ(kDoubleFields.size(), 22u);
  for (const auto & [name, member] : kDoubleFields) {
    for (const double bad : {kNaN, kInf, -kInf}) {
      expect_rejected(
        name, [&, member = member](auto & c) {c.*member = bad;},
        {name, "finite"});
    }
  }
}

TEST(SlamConfig, LogOddsExtremaMustRoundToAtLeastOneCell)
{
  // lo_max * 20 = 0.4 rounds to 0: no cell could ever rise above unknown.
  expect_rejected(
    "lo_max 0.02", [](auto & c) {c.lo_max = 0.02; c.log_odds_scale = 20.0;},
    {"lo_max * log_odds_scale", "at least 1"});
  expect_rejected(
    "lo_min -0.02", [](auto & c) {c.lo_min = -0.02; c.log_odds_scale = 20.0;},
    {"lo_min * log_odds_scale", "at most -1"});
  // 0.0249 * 20 = 0.498 rounds to 0 and is rejected.
  expect_rejected(
    "lo_max 0.0249", [](auto & c) {c.lo_max = 0.0249; c.log_odds_scale = 20.0;},
    {"lo_max * log_odds_scale", "at least 1"});
  expect_rejected(
    "lo_min -0.0249", [](auto & c) {c.lo_min = -0.0249; c.log_odds_scale = 20.0;},
    {"lo_min * log_odds_scale", "at most -1"});
  // 0.025 * 20 is exactly 0.5 in double; std::lround rounds half away from zero, so the
  // smallest accepted extremum has magnitude 0.025.
  slam::SlamConfig c;
  c.log_odds_scale = 20.0;
  c.lo_max = 0.025;
  EXPECT_NO_THROW(slam::validate(c));
  c.lo_max = 5.0;
  c.lo_min = -0.025;
  EXPECT_NO_THROW(slam::validate(c));
}

TEST(SlamConfig, LogOddsExtremaProductBoundIs127Point5)
{
  // 6.37 * 20 = 127.4 rounds to 127 (fits int8); 6.38 * 20 = 127.6 rounds to 128 (does not).
  slam::SlamConfig c;
  c.log_odds_scale = 20.0;
  c.lo_max = 6.37;
  EXPECT_NO_THROW(slam::validate(c));
  c.lo_min = -6.37;
  EXPECT_NO_THROW(slam::validate(c));
  expect_rejected(
    "lo_max 6.38", [](auto & x) {x.lo_max = 6.38; x.log_odds_scale = 20.0;},
    {"lo_max * log_odds_scale"});
  expect_rejected(
    "lo_min -6.38", [](auto & x) {x.lo_min = -6.38; x.log_odds_scale = 20.0;},
    {"lo_min * log_odds_scale"});
}

TEST(SlamConfig, MessageNamesFieldAndSourceLocation)
{
  slam::SlamConfig c;
  c.num_particles = 0;
  try {
    slam::validate(c);
    FAIL() << "expected std::invalid_argument";
  } catch (const std::invalid_argument & e) {
    const std::string what = e.what();
    EXPECT_NE(what.find("num_particles"), std::string::npos) << what;
    EXPECT_NE(what.find("slam_config.cpp:"), std::string::npos) << what;
  }
}

TEST(SlamConfig, BothParticleFilterConstructorsValidate)
{
  slam::SlamConfig c;
  c.num_particles = 0;
  EXPECT_THROW(slam::ParticleFilter filter(c), std::invalid_argument);
  EXPECT_THROW(slam::ParticleFilter filter(c, /*seed=*/ 1), std::invalid_argument);
}

TEST(SlamConfig, OccupancyGridRejectsBadGeometry)
{
  const slam::SlamConfig c;
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_THROW(slam::OccupancyGrid(10.0, 10.0, 0.0, c), std::invalid_argument) << "resolution 0";
  EXPECT_THROW(slam::OccupancyGrid(-10.0, 10.0, 0.1, c), std::invalid_argument) << "negative width";
  EXPECT_THROW(
    slam::OccupancyGrid(10.0, -10.0, 0.1, c),
    std::invalid_argument) << "negative height";
  EXPECT_THROW(slam::OccupancyGrid(0.1, 0.1, 0.1, c), std::invalid_argument) << "ratio 1, below 2";
  EXPECT_THROW(
    slam::OccupancyGrid(2002.0, 2002.0, 0.1, c),
    std::invalid_argument) << "ratio above 20000";
  EXPECT_THROW(slam::OccupancyGrid(inf, 10.0, 0.1, c), std::invalid_argument) << "infinite width";
}

TEST(SlamConfig, OccupancyGridValidatesConfig)
{
  slam::SlamConfig c;
  EXPECT_NO_THROW(slam::OccupancyGrid(10.0, 10.0, 0.1, c));
  c.log_odds_scale = 200.0;
  EXPECT_THROW(slam::OccupancyGrid(10.0, 10.0, 0.1, c), std::invalid_argument);
}

TEST(SlamConfig, OccupancyGridRoundsCellCountInsteadOfTruncating)
{
  // 0.6 / 0.1 = 5.999999999999999 in floating point; truncation would give 5.
  const double width = 0.6, res = 0.1;
  ASSERT_LT(width / res, 6.0);
  const slam::OccupancyGrid g(width, width, res, slam::SlamConfig{});
  EXPECT_EQ(g.cols(), 6);
  EXPECT_EQ(g.rows(), 6);
}

TEST(SlamConfig, ScaledAccessorsRound)
{
  // Each product is x.6, so rounding gives x+1 in magnitude and truncation x.
  slam::SlamConfig c;
  c.log_odds_scale = 20.0;
  c.lo_occupied = 0.83;  // 16.6 -> 17
  c.lo_free = -0.43;     // -8.6 -> -9
  c.lo_max = 4.93;       // 98.6 -> 99
  c.lo_min = -4.93;      // -98.6 -> -99
  EXPECT_EQ(c.lo_occupied_scaled(), 17);
  EXPECT_EQ(c.lo_free_scaled(), -9);
  EXPECT_EQ(c.lo_max_scaled(), 99);
  EXPECT_EQ(c.lo_min_scaled(), -99);
}
