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

#include <algorithm>
#include <cmath>

#include "slam_cpp/slam_types.hpp"

// For any finite angle in +-1e6: the result is in [-pi, pi), a second
// normalisation changes nothing, and the result differs from the input by a
// whole number of turns.
RC_GTEST_PROP(SlamTypesProperty, NormalizeAngleRangeIdempotenceAndWholeTurns, ())
{
  const double a = *rc::gen::map(
    rc::gen::inRange(-1000000000, 1000000001), [](int v) {return v * 1e-3;});
  const double r = slam::normalize_angle(a);
  RC_ASSERT(r >= -slam::kPi);
  RC_ASSERT(r < slam::kPi);
  RC_ASSERT(slam::normalize_angle(r) == r);
  const double turns = (a - r) / slam::kTwoPi;
  RC_ASSERT(std::abs(turns - std::round(turns)) < 1e-9 * std::max(1.0, std::abs(a)));
}
