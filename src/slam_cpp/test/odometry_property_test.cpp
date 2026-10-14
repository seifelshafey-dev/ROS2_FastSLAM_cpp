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

#include "slam_cpp/odometry.hpp"

namespace
{

const double kPi = std::acos(-1.0);

// Value v * unit for an integer v in [lo, hi].
rc::Gen<double> scaled(int lo, int hi, double unit)
{
  return rc::gen::map(rc::gen::inRange(lo, hi + 1), [unit](int v) {return v * unit;});
}

// Difference of two angles, wrapped to (-pi, pi].
double angle_diff(double a, double b) {return std::remainder(a - b, 2.0 * kPi);}

double wrap(double a)
{
  const double w = std::fmod(a + kPi, 2.0 * kPi);
  return (w < 0 ? w + 2.0 * kPi : w) - kPi;
}

}  // namespace

RC_GTEST_PROP(OdometryProperty, ComposeMapToOdomWithOdomGivesSlamPose, ())
{
  const double sx = *scaled(-10000, 10000, 1e-3);
  const double sy = *scaled(-10000, 10000, 1e-3);
  const double sth = *scaled(-3141, 3141, 1e-3);
  const double ox = *scaled(-10000, 10000, 1e-3);
  const double oy = *scaled(-10000, 10000, 1e-3);
  const double oth = *scaled(-3141, 3141, 1e-3);
  const Eigen::Vector3d s(sx, sy, sth), o(ox, oy, oth);

  const Eigen::Vector3d r = slam::compose(slam::map_to_odom(s, o), o);
  RC_ASSERT(std::abs(r(0) - s(0)) < 1e-9);
  RC_ASSERT(std::abs(r(1) - s(1)) < 1e-9);
  RC_ASSERT(std::abs(angle_diff(r(2), s(2))) < 1e-9);
}

RC_GTEST_PROP(OdometryProperty, ComposeWithInverseIsIdentity, ())
{
  const double x = *scaled(-10000, 10000, 1e-3);
  const double y = *scaled(-10000, 10000, 1e-3);
  const double th = *scaled(-3141, 3141, 1e-3);
  const Eigen::Vector3d a(x, y, th);

  const Eigen::Vector3d r = slam::compose(a, slam::inverse(a));
  RC_ASSERT(std::abs(r(0)) < 1e-9);
  RC_ASSERT(std::abs(r(1)) < 1e-9);
  RC_ASSERT(std::abs(angle_diff(r(2), 0.0)) < 1e-9);
}

namespace
{

// Builds the end pose of a noise-free motion, feeds the displacement to
// decompose_motion and checks the three values come back. sign = +1 forward,
// -1 reverse.
//
// rot1 stays 0.01 rad away from +-pi/2: there the bearing of the displacement
// is at +-pi/2 from the heading and the forward/reverse decision flips, so the
// motion no longer has a unique decomposition.
void check_round_trip(double sign)
{
  const double x = *scaled(-5000, 5000, 1e-3);
  const double y = *scaled(-5000, 5000, 1e-3);
  const double th = *scaled(-3141, 3141, 1e-3);
  const double trans = *scaled(20, 2000, 1e-3);
  const double rot1 = *scaled(-1560, 1560, 1e-3);  // |rot1| <= pi/2 - 0.01
  const double rot2 = *scaled(-1570, 1570, 1e-3);

  const double th1 = th + rot1;
  const double x_end = x + sign * trans * std::cos(th1);
  const double y_end = y + sign * trans * std::sin(th1);
  const double th_end = wrap(th1 + rot2);

  const slam::MotionDelta d =
    slam::decompose_motion(x_end - x, y_end - y, wrap(th_end - th), th);
  RC_ASSERT(d.direction == sign);
  RC_ASSERT(std::abs(d.trans - trans) < 1e-9);
  RC_ASSERT(std::abs(angle_diff(d.rot1, rot1)) < 1e-9);
  RC_ASSERT(std::abs(angle_diff(d.rot2, rot2)) < 1e-9);
}

}  // namespace

RC_GTEST_PROP(OdometryProperty, ForwardMotionRoundTrips, ())
{
  check_round_trip(1.0);
}

RC_GTEST_PROP(OdometryProperty, ReverseMotionRoundTrips, ())
{
  check_round_trip(-1.0);
}

RC_GTEST_PROP(OdometryProperty, DecomposeOutputsAreInRange, ())
{
  const double dx = *scaled(-5000, 5000, 1e-3);
  const double dy = *scaled(-5000, 5000, 1e-3);
  const double dth = *scaled(-10000, 10000, 1e-3);
  const double th = *scaled(-10000, 10000, 1e-3);

  const slam::MotionDelta d = slam::decompose_motion(dx, dy, dth, th);
  RC_ASSERT(d.trans >= 0.0);
  RC_ASSERT(d.rot1 >= -kPi && d.rot1 < kPi);
  RC_ASSERT(d.rot2 >= -kPi && d.rot2 < kPi);
  RC_ASSERT(d.direction == 1.0 || d.direction == -1.0);
}
