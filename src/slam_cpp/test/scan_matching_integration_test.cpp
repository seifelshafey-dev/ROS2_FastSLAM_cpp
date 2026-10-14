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
#include <iostream>
#include <random>

#include "slam_cpp/slam_config.hpp"
#include "slam_cpp/occupancy_grid.hpp"
#include "slam_cpp/scan_matching.hpp"
#include "slam_cpp/sigmoid_lut.hpp"
#include "support/room.hpp"

// Map built from true poses along a circle; each later scan is matched from a
// perturbed start. The matcher must not carry a systematic offset.
TEST(ScanMatchingIntegration, ScanMatchOnKnownPoseMapHasNoBias)
{
  const double pi = std::acos(-1.0);
  slam::SlamConfig cfg;
  cfg.map_size = 20.0;
  slam::SigmoidLUT lut(cfg.log_odds_scale);
  slam::OccupancyGrid grid(cfg.map_size, cfg.map_size, cfg.map_resolution, cfg);
  std::mt19937 g(7);
  std::normal_distribution<double> nd(0.0, 1.0);

  int n = 0, fallback = 0;
  Eigen::Vector3d sum = Eigen::Vector3d::Zero(), sq = Eigen::Vector3d::Zero();
  for (int k = 0; k < 400; ++k) {
    const double a = 0.04 * k;
    const Eigen::Vector3d truth(1.5 * std::cos(a), 1.5 * std::sin(a), a + pi / 2);
    const auto z = slam_test::scan_from(truth, 4.03, 3.02, 0.01, g);
    if (k >= 5) {
      const Eigen::Vector3d start = truth +
        Eigen::Vector3d(0.02 * nd(g), 0.02 * nd(g), 0.01 * nd(g));
      const auto sm = slam::scan_match_single(
        grid, z, start, Eigen::Matrix3d::Identity() * 1e-5, grid.map_quality(), cfg, lut);
      ++n;
      if (sm.pose == start) {
        ++fallback;
      } else {
        Eigen::Vector3d e = sm.pose - truth;
        e(2) = std::remainder(e(2), 2.0 * pi);
        sum += e;
        sq += e.cwiseProduct(e);
      }
    }
    grid.update(truth, z);
  }
  const int m = n - fallback;
  ASSERT_GT(m, 0);
  const Eigen::Vector3d mean = sum / m;
  const Eigen::Vector3d rms = (sq / m).cwiseSqrt();
  std::cout << "matches=" << n << " fallback=" << fallback
            << " mean_err=(" << mean(0) << ", " << mean(1) << ", " << mean(2) << ")"
            << " rms=(" << rms(0) << ", " << rms(1) << ", " << rms(2) << ")\n";

  EXPECT_EQ(fallback, 0);
  for (int d = 0; d < 2; ++d) {
    EXPECT_LT(std::abs(mean(d)), 0.01) << "axis " << d;
    EXPECT_LT(rms(d), 0.015) << "axis " << d;
  }
}
