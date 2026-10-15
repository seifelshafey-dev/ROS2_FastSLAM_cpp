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
#include <random>

#include "room.hpp"

namespace
{
const double kPi = std::acos(-1.0);
const double kHx = 4.0, kHy = 3.0;
}  // namespace

TEST(SupportRoom, BeamAtIndex180HasRangeHxOverCos) {
  std::mt19937 rng(1);
  const auto pts = slam_test::scan_from({0, 0, 0}, kHx, kHy, 0.0, rng);
  // Beam i = 180 is the 91st kept beam (kept indices 0, 2, ...), row 90, assuming
  // all beams earlier in the list are in range (true here: nearest wall 3 m, farthest 5 m).
  const double angle = -kPi + 180 * (2.0 * kPi / 359.0);  // pi/359 ~ 0.008751 rad
  EXPECT_NEAR(angle, kPi / 359.0, 1e-12);
  ASSERT_EQ(pts.rows(), 180);
  const double range = kHx / std::cos(angle);
  EXPECT_NEAR(pts(90, 0), range * std::cos(angle), 1e-9);
  EXPECT_NEAR(pts(90, 1), range * std::sin(angle), 1e-9);
  EXPECT_NEAR(pts.row(90).norm(), range, 1e-9);
}

TEST(SupportRoom, EveryPointLiesOnAWall) {
  std::mt19937 rng(1);
  const auto pts = slam_test::scan_from({0, 0, 0}, kHx, kHy, 0.0, rng);
  ASSERT_GT(pts.rows(), 0);
  for (int k = 0; k < pts.rows(); ++k) {
    const double x = pts(k, 0), y = pts(k, 1);
    const double d = std::min(std::abs(std::abs(x) - kHx), std::abs(std::abs(y) - kHy));
    EXPECT_LT(d, 1e-9) << "row " << k;
  }
}

TEST(SupportRoom, CountEqualsKeptBeamsInRange) {
  std::mt19937 rng(1);
  const auto pts = slam_test::scan_from({0, 0, 0}, kHx, kHy, 0.0, rng);
  // Independent count: nearest-wall distance per beam, from the room geometry.
  int expected = 0;
  for (int i = 0; i < 360; i += 2) {
    const double a = -kPi + i * (2.0 * kPi / 359.0);
    const double t = std::min(std::abs(kHx / std::cos(a)), std::abs(kHy / std::sin(a)));
    if (t > 0.15 && t < 10.0) {++expected;}
  }
  EXPECT_EQ(pts.rows(), expected);
  EXPECT_EQ(expected, 180);
}

TEST(SupportRoom, BeamStepOneGivesTwiceTheDefaultPoints) {
  std::mt19937 rng(1);
  const auto dense = slam_test::scan_from({0, 0, 0}, kHx, kHy, 0.0, rng, 1);
  const auto sparse = slam_test::scan_from({0, 0, 0}, kHx, kHy, 0.0, rng, 2);
  EXPECT_EQ(sparse.rows(), 180);
  EXPECT_EQ(dense.rows(), 2 * sparse.rows());
}
