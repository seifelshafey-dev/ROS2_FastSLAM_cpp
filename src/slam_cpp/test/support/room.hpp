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

// Test helper: analytic ray-caster for a rectangular room. Header-only.
// Units: metres, radians. World frame: walls at x = +-hx, y = +-hy.
// Base frame: x forward, y left, origin at the robot.
#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

namespace slam_test
{

// Hit points (base frame, m, one per row) of 360 beams at angles
// -pi + i*(2*pi/359) in the base frame, every beam_step-th beam (i = 0, beam_step, ...)
// kept; the default 2 keeps every second beam. With beam_step 1 the first and last beams
// (i = 0 and i = 359) both point along +-pi, so the 360 points contain one repeated
// direction and the angle between neighbouring beams is 2*pi/359.
// range = distance to nearest wall along the beam from pose (x, y, yaw in world)
// + noise_std * N(0,1); a beam is kept only if 0.15 < range < 10.0.
inline Eigen::Matrix<double, Eigen::Dynamic, 2> scan_from(
  const Eigen::Vector3d & pose, double hx, double hy, double noise_std, std::mt19937 & rng,
  int beam_step = 2)
{
  const double pi = std::acos(-1.0);
  std::normal_distribution<double> nd(0.0, 1.0);
  std::vector<Eigen::Vector2d> pts;
  for (int i = 0; i < 360; i += beam_step) {
    const double a = -pi + i * (2.0 * pi / 359.0);
    const double w = pose(2) + a, c = std::cos(w), s = std::sin(w);
    double t = 1e9;
    if (c > 1e-12) {t = std::min(t, (hx - pose(0)) / c);}
    if (c < -1e-12) {t = std::min(t, (-hx - pose(0)) / c);}
    if (s > 1e-12) {t = std::min(t, (hy - pose(1)) / s);}
    if (s < -1e-12) {t = std::min(t, (-hy - pose(1)) / s);}
    t += noise_std * nd(rng);
    if (t > 0.15 && t < 10.0) {pts.emplace_back(t * std::cos(a), t * std::sin(a));}
  }
  Eigen::Matrix<double, Eigen::Dynamic, 2> m(pts.size(), 2);
  for (size_t k = 0; k < pts.size(); ++k) {
    m.row(k) = pts[k].transpose();
  }
  return m;
}

// Pose a^-1 * b: b expressed in the frame of a. Angle wrapped to [-pi, pi].
inline Eigen::Vector3d rel(const Eigen::Vector3d & a, const Eigen::Vector3d & b)
{
  const double c = std::cos(a(2)), s = std::sin(a(2));
  const double dx = b(0) - a(0), dy = b(1) - a(1);
  return {c * dx + s * dy, -s * dx + c * dy, std::remainder(b(2) - a(2), 2.0 * std::acos(-1.0))};
}

}  // namespace slam_test
