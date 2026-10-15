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

#ifndef SLAM_CPP__ODOMETRY_HPP_
#define SLAM_CPP__ODOMETRY_HPP_

/**
 * @file odometry.hpp
 * @brief Odometry helpers: rotation-translation-rotation motion decomposition and planar
 *        pose algebra. No ROS dependency.
 */

#include <Eigen/Dense>

namespace slam
{

/// Net translation, in metres, below which the direction of travel is not used: the
/// bearing of so short a displacement is mostly noise.
inline constexpr double kMinTranslationForHeading = 0.01;  // m

/// Rotation-translation-rotation motion between two odometry poses.
struct MotionDelta
{
  /// Distance travelled in metres; >= 0.
  double trans;
  /// Turn before the translation in radians, in [-pi, pi).
  double rot1;
  /// Turn after the translation in radians, in [-pi, pi).
  double rot2;
  /// +1.0 for forward motion, -1.0 for reverse.
  double direction;
};

/**
 * @brief Decompose an odometry segment into rotation, translation, rotation.
 *
 * `trans` is hypot(dx, dy). If `trans` is below `kMinTranslationForHeading`, `rot1` is 0,
 * `rot2` is the wrapped `dth` and `direction` is +1. Otherwise `rot1` is the bearing of
 * (dx, dy) relative to `start_theta`; if |rot1| > pi/2 the motion is taken as reverse:
 * `direction` is -1 and `rot1` is the bearing of (-dx, -dy) relative to `start_theta`.
 * `rot2` is the wrapped (`dth` - `rot1`).
 *
 * Applying the result as theta' = theta + rot1, x += direction * trans * cos(theta'),
 * y += direction * trans * sin(theta'), theta = wrap(theta' + rot2) reproduces the segment
 * when `trans` is not below the threshold.
 *
 * @param dx          Displacement in x over the segment, odom frame (m).
 * @param dy          Displacement in y over the segment, odom frame (m).
 * @param dth         Heading change over the segment (rad); any value, it is wrapped.
 * @param start_theta Heading at the start of the segment, odom frame (rad).
 * @return The decomposed motion; angles in [-pi, pi).
 */
MotionDelta decompose_motion(
  double dx, double dy, double dth,
  double start_theta);

/**
 * @brief Compose two planar poses, a (+) b.
 *
 * `b` is expressed in the frame of `a`; the result is `b` in the frame `a` is expressed in:
 * xy = a.xy + R(a.theta) * b.xy, theta = wrap(a.theta + b.theta).
 *
 * @param a Pose (x, y, theta); metres and radians.
 * @param b Pose (x, y, theta) in the frame of `a`; metres and radians.
 * @return Composed pose (x, y, theta); theta in [-pi, pi).
 */
Eigen::Vector3d compose(const Eigen::Vector3d & a, const Eigen::Vector3d & b);

/**
 * @brief Inverse of a planar pose.
 *
 * compose(a, inverse(a)) and compose(inverse(a), a) are both (0, 0, 0):
 * xy = -R(a.theta)^T * a.xy, theta = wrap(-a.theta).
 *
 * @param a Pose (x, y, theta); metres and radians.
 * @return Inverse pose (x, y, theta); theta in [-pi, pi).
 */
Eigen::Vector3d inverse(const Eigen::Vector3d & a);

/**
 * @brief Transform from the odom frame to the map frame.
 *
 * Returns T = compose(slam_pose, inverse(odom_pose)), the pose of the odom frame in the
 * map frame, so that compose(T, odom_pose) = slam_pose:
 * T.theta = wrap(slam_pose.theta - odom_pose.theta),
 * T.xy = slam_pose.xy - R(T.theta) * odom_pose.xy.
 *
 * @param slam_pose Pose of the base in the map frame (x, y, theta); metres and radians.
 * @param odom_pose Pose of the base in the odom frame at the same moment as `slam_pose`
 *                  (x, y, theta); metres and radians.
 * @return Pose of the odom frame in the map frame (x, y, theta); theta in [-pi, pi).
 */
Eigen::Vector3d map_to_odom(
  const Eigen::Vector3d & slam_pose,
  const Eigen::Vector3d & odom_pose);

}  // namespace slam

#endif  // SLAM_CPP__ODOMETRY_HPP_
