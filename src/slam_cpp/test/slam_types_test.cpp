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
#include <limits>

#include "slam_cpp/slam_types.hpp"

TEST(SlamTypes, NormalizeAngleJustBelowMinusPiStaysInRange)
{
  const double in = std::nextafter(-slam::kPi, -10.0);
  const double r = slam::normalize_angle(in);
  EXPECT_GE(r, -slam::kPi);
  EXPECT_LT(r, slam::kPi);
  EXPECT_DOUBLE_EQ(r, -slam::kPi);
}

TEST(SlamTypes, NormalizeAngleJustBelowPiStaysInRange)
{
  const double in = std::nextafter(slam::kPi, 0.0);
  const double r = slam::normalize_angle(in);
  // The sum in + pi rounds to 2*pi, so the result is -pi.
  EXPECT_GE(r, -slam::kPi);
  EXPECT_LT(r, slam::kPi);
  EXPECT_LT(std::abs(std::remainder(r - in, slam::kTwoPi)), 1e-15);
}

TEST(SlamTypes, NormalizeAnglePiReturnsMinusPi)
{
  EXPECT_EQ(slam::normalize_angle(slam::kPi), -slam::kPi);
  EXPECT_EQ(slam::normalize_angle(-slam::kPi), -slam::kPi);
  EXPECT_EQ(slam::normalize_angle(0.0), 0.0);
}

TEST(SlamTypes, NormalizeAngleRangeHoldsNearOddMultiplesOfPi)
{
  const double inf = std::numeric_limits<double>::infinity();
  for (int k = -50; k <= 50; ++k) {
    const double base = (2 * k + 1) * slam::kPi;
    for (int n = -4; n <= 4; ++n) {
      double in = base;
      for (int i = 0; i < std::abs(n); ++i) {
        in = std::nextafter(in, n < 0 ? -inf : inf);
      }
      const double r = slam::normalize_angle(in);
      EXPECT_TRUE(r >= -slam::kPi && r < slam::kPi) << "input " << std::hexfloat << in
                                                    << " result " << r << " (k=" << k << ", n=" <<
        n << ")";
    }
  }
}
