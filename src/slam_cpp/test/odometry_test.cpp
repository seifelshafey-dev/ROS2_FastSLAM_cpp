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

#include "slam_cpp/odometry.hpp"

namespace
{

const double kPi = std::acos(-1.0);

}  // namespace

TEST(Odometry, PureRotation)
{
  const auto d = slam::decompose_motion(0.0, 0.0, 0.3, 0.7);
  EXPECT_EQ(d.trans, 0.0);
  EXPECT_EQ(d.rot1, 0.0);
  EXPECT_NEAR(d.rot2, 0.3, 1e-12);
  EXPECT_EQ(d.direction, 1.0);
}

TEST(Odometry, StraightForwardAlongStartHeading)
{
  const double th = 0.5;
  const auto d = slam::decompose_motion(1.2 * std::cos(th), 1.2 * std::sin(th), 0.0, th);
  EXPECT_NEAR(d.trans, 1.2, 1e-12);
  EXPECT_NEAR(d.rot1, 0.0, 1e-12);
  EXPECT_NEAR(d.rot2, 0.0, 1e-12);
  EXPECT_EQ(d.direction, 1.0);
}

TEST(Odometry, StraightReverse)
{
  const double th = 0.5, dth = 0.2;
  const auto d = slam::decompose_motion(-1.2 * std::cos(th), -1.2 * std::sin(th), dth, th);
  EXPECT_NEAR(d.trans, 1.2, 1e-12);
  EXPECT_NEAR(d.rot1, 0.0, 1e-12);
  EXPECT_NEAR(d.rot2, dth, 1e-12);
  EXPECT_EQ(d.direction, -1.0);
}

TEST(Odometry, SidewaysBelowThresholdIgnoresBearing)
{
  // hypot(0.003, 0.0085) = 0.00901 < kMinTranslationForHeading.
  ASSERT_LT(std::hypot(0.003, 0.0085), slam::kMinTranslationForHeading);
  const auto d = slam::decompose_motion(0.003, 0.0085, 0.4, 0.0);
  EXPECT_NEAR(d.trans, std::hypot(0.003, 0.0085), 1e-15);
  EXPECT_EQ(d.rot1, 0.0);
  EXPECT_NEAR(d.rot2, 0.4, 1e-12);
  EXPECT_EQ(d.direction, 1.0);
}

TEST(Odometry, SidewaysAboveThresholdUsesBearing)
{
  // hypot(0.003, 0.0105) = 0.01092 > kMinTranslationForHeading.
  ASSERT_GT(std::hypot(0.003, 0.0105), slam::kMinTranslationForHeading);
  const auto d = slam::decompose_motion(0.003, 0.0105, 0.4, 0.0);
  const double bearing = std::atan2(0.0105, 0.003);  // ~1.2925 rad, within +-pi/2
  EXPECT_NEAR(d.rot1, bearing, 1e-12);
  EXPECT_NEAR(d.rot2, 0.4 - bearing, 1e-12);
  EXPECT_EQ(d.direction, 1.0);
}

TEST(Odometry, HeadingChangeNearPiWraps)
{
  // 3.5 rad is 3.5 - 2*pi = -2.78319 rad; -3.5 rad is +2.78319 rad.
  const auto a = slam::decompose_motion(0.0, 0.0, 3.5, 0.0);
  EXPECT_NEAR(a.rot2, 3.5 - 2.0 * kPi, 1e-12);
  const auto b = slam::decompose_motion(0.0, 0.0, -3.5, 0.0);
  EXPECT_NEAR(b.rot2, -3.5 + 2.0 * kPi, 1e-12);
}

TEST(Odometry, MapToOdomOfEqualPosesIsIdentity)
{
  const Eigen::Vector3d p(1.3, -0.7, 2.1);
  const Eigen::Vector3d t = slam::map_to_odom(p, p);
  EXPECT_NEAR(t(0), 0.0, 1e-12);
  EXPECT_NEAR(t(1), 0.0, 1e-12);
  EXPECT_NEAR(t(2), 0.0, 1e-12);
}

TEST(Odometry, MapToOdomHandComputedWith90DegreeHeadingDifference)
{
  // S = (2, 1, pi), O = (1, 1, pi/2).
  // T.theta = wrap(pi - pi/2) = pi/2.
  // R(pi/2) = [[0, -1], [1, 0]], so R * O.xy = R * (1, 1) = (-1, 1).
  // T.xy = S.xy - R * O.xy = (2, 1) - (-1, 1) = (3, 0).
  // Check: compose(T, O) = (3 + 0*1 - 1*1, 0 + 1*1 + 0*1) = (2, 1), theta = pi/2 + pi/2 = pi.
  const Eigen::Vector3d s(2.0, 1.0, kPi), o(1.0, 1.0, kPi / 2);
  const Eigen::Vector3d t = slam::map_to_odom(s, o);
  EXPECT_NEAR(t(0), 3.0, 1e-12);
  EXPECT_NEAR(t(1), 0.0, 1e-12);
  EXPECT_NEAR(t(2), kPi / 2, 1e-12);
}
