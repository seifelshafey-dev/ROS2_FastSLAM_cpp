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

#include <cmath>
#include <random>

#include "room.hpp"

RC_GTEST_PROP(SupportRoomProperty, PointsLieOnWallsInWorldFrame, ()) {
  const double hx = 4.0, hy = 3.0;
  const double x = *rc::gen::map(
    rc::gen::inRange(-1000, 1001), [&](int v) {
      return (hx - 0.5) * 0.999 * v / 1000.0;
    });
  const double y = *rc::gen::map(
    rc::gen::inRange(-1000, 1001), [&](int v) {
      return (hy - 0.5) * 0.999 * v / 1000.0;
    });
  const double th = *rc::gen::map(rc::gen::inRange(-3141, 3142), [](int v) {return v / 1000.0;});
  std::mt19937 rng(1);
  const auto pts = slam_test::scan_from({x, y, th}, hx, hy, 0.0, rng);
  RC_ASSERT(pts.rows() > 0);
  const double c = std::cos(th), s = std::sin(th);
  for (int k = 0; k < pts.rows(); ++k) {
    const double wx = x + c * pts(k, 0) - s * pts(k, 1);
    const double wy = y + s * pts(k, 0) + c * pts(k, 1);
    const double d = std::min(std::abs(std::abs(wx) - hx), std::abs(std::abs(wy) - hy));
    RC_ASSERT(d < 1e-9);
  }
}
