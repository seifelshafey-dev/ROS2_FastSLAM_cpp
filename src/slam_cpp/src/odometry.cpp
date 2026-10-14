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

#include "slam_cpp/odometry.hpp"

#include <cmath>

#include "slam_cpp/slam_types.hpp"

namespace slam
{

// ---------------------------------------------------------------------------
// decompose_motion: odometry segment to rotation-translation-rotation.
// ---------------------------------------------------------------------------
MotionDelta decompose_motion(
  double dx, double dy, double dth,
  double start_theta)
{
  double trans = std::hypot(dx, dy);

  if (trans < kMinTranslationForHeading) {
    return {trans, 0.0, normalize_angle(dth), 1.0};
  }

  double rot1 = normalize_angle(std::atan2(dy, dx) - start_theta);
  double direction = 1.0;

  // Reverse detection: if rot1 > 90 deg, robot is likely driving backward.
  if (std::abs(rot1) > (kPi / 2.0)) {
    direction = -1.0;
    rot1 = normalize_angle(std::atan2(-dy, -dx) - start_theta);
  }

  return {trans, rot1, normalize_angle(dth - rot1), direction};
}

// ---------------------------------------------------------------------------
// compose: a (+) b: b, given in the frame of a, in the frame of a's parent.
// ---------------------------------------------------------------------------
Eigen::Vector3d compose(const Eigen::Vector3d & a, const Eigen::Vector3d & b)
{
  double c = std::cos(a(2));
  double s = std::sin(a(2));
  return Eigen::Vector3d(
    a(0) + c * b(0) - s * b(1),
    a(1) + s * b(0) + c * b(1),
    normalize_angle(a(2) + b(2)));
}

// ---------------------------------------------------------------------------
// inverse: pose whose composition with a is the identity.
// ---------------------------------------------------------------------------
Eigen::Vector3d inverse(const Eigen::Vector3d & a)
{
  double c = std::cos(a(2));
  double s = std::sin(a(2));
  return Eigen::Vector3d(
    -c * a(0) - s * a(1),
    s * a(0) - c * a(1),
    normalize_angle(-a(2)));
}

// ---------------------------------------------------------------------------
// map_to_odom: pose of the odom frame in the map frame.
// ---------------------------------------------------------------------------
Eigen::Vector3d map_to_odom(
  const Eigen::Vector3d & slam_pose,
  const Eigen::Vector3d & odom_pose)
{
  return compose(slam_pose, inverse(odom_pose));
}

}  // namespace slam
