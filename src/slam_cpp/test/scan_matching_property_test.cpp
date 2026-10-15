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
#include <limits>
#include <vector>
#include <random>

#include "slam_cpp/slam_config.hpp"
#include "slam_cpp/occupancy_grid.hpp"
#include "slam_cpp/scan_matching.hpp"
#include "slam_cpp/sigmoid_lut.hpp"
#include "support/room.hpp"
#include "support/room_match.hpp"

namespace
{

slam::SlamConfig make_config()
{
  slam::SlamConfig config;
  config.map_size = 10.0;
  config.map_resolution = 0.1;
  return config;
}

// Fraction in [0.01, 0.99], generated as an integer count of hundredths.
rc::Gen<double> fraction()
{
  return rc::gen::map(rc::gen::inRange(1, 100), [](int v) {return v / 100.0;});
}

}  // namespace

// A hit written at a world point is read back, at the centre of the cell
// world_to_grid names, as sigmoid(lo_occupied_scaled).
RC_GTEST_PROP(ScanMatchingProperty, WrittenHitIsReadAtCellCentre, ())
{
  const slam::SlamConfig config = make_config();
  const slam::SigmoidLUT sigmoid(config.log_odds_scale);
  slam::OccupancyGrid grid(config.map_size, config.map_size, config.map_resolution, config);
  const int cols = grid.cols(), rows = grid.rows();
  const double res = config.map_resolution;

  const int col = *rc::gen::inRange(2, cols - 2);
  const int row = *rc::gen::inRange(2, rows - 2);
  const double fx = *fraction();
  const double fy = *fraction();
  RC_PRE(!(col == cols / 2 && row == rows / 2));  // hit cell is not the robot's cell

  const double wx = (col - cols / 2 + fx) * res;
  const double wy = (row - rows / 2 + fy) * res;
  Eigen::Matrix<double, Eigen::Dynamic, 2> hit(1, 2);
  hit << wx, wy;
  grid.update(Eigen::Vector3d(0.0, 0.0, 0.0), hit);

  const auto cell = grid.world_to_grid(wx, wy);
  RC_ASSERT(cell.has_value());
  RC_ASSERT(cell->x == col);
  RC_ASSERT(cell->y == row);

  const slam::GridView gv = slam::make_grid_view(grid, config, sigmoid);
  Eigen::Matrix<double, Eigen::Dynamic, 2> centre(1, 2);
  centre << (col - cols / 2 + 0.5) * res, (row - rows / 2 + 0.5) * res;
  const auto pix = slam::to_grid_coords(centre, gv.inv_res, gv.sample_origin);
  const auto r = slam::bilinear_interpolation(gv, pix);
  RC_ASSERT(r.mask(0) == 1);
  RC_ASSERT(std::abs(r.values(0) - sigmoid(config.lo_occupied_scaled())) < 1e-12);
}

// An interpolated value lies within the min and max of its four cells (mask 1);
// when any of the four is outside the grid, mask 0 and value 0.
RC_GTEST_PROP(ScanMatchingProperty, InterpolationIsBoundedByItsFourCells, ())
{
  static const slam::SlamConfig config = make_config();
  static const slam::SigmoidLUT sigmoid(config.log_odds_scale);
  static const slam::OccupancyGrid grid = [] {
      slam::OccupancyGrid g(config.map_size, config.map_size, config.map_resolution, config);
      std::mt19937 rng(11);
      for (const Eigen::Vector3d & p :
        {Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(1.0, 0.5, 0.3),
          Eigen::Vector3d(-1.0, -0.5, -0.4)})
      {
        g.update(p, slam_test::scan_from(p, 4.0, 3.0, 0.01, rng));
      }
      return g;
    }();
  const slam::GridView gv = slam::make_grid_view(grid, config, sigmoid);
  const int n = grid.cols();

  // Sample coordinates from 2 cells outside the grid on each side, in 1/1000 cell steps.
  auto coord =
    rc::gen::map(rc::gen::inRange(-2000, (n + 2) * 1000), [](int v) {return v / 1000.0;});
  const double sx = *coord;
  const double sy = *coord;

  Eigen::Matrix<double, Eigen::Dynamic, 2> pix(1, 2);
  pix << sx, sy;
  const auto r = slam::bilinear_interpolation(gv, pix);

  const int x0 = static_cast<int>(std::floor(sx)), y0 = static_cast<int>(std::floor(sy));
  const bool inside = x0 >= 0 && x0 + 1 <= n - 1 && y0 >= 0 && y0 + 1 <= n - 1;
  if (inside) {
    RC_ASSERT(r.mask(0) == 1);
    double lo = 1e9, hi = -1e9;
    for (int dy = 0; dy <= 1; ++dy) {
      for (int dx = 0; dx <= 1; ++dx) {
        const double v = sigmoid(grid.at(y0 + dy, x0 + dx));
        lo = std::min(lo, v);
        hi = std::max(hi, v);
      }
    }
    RC_ASSERT(r.values(0) >= lo - 1e-12);
    RC_ASSERT(r.values(0) <= hi + 1e-12);
  } else {
    RC_ASSERT(r.mask(0) == 0);
    RC_ASSERT(r.values(0) == 0.0);
  }
}

// The result of scan_match_single never costs more than its odometry pose.
// C is compute_joint_cost with the odom_info that scan_match_single builds
// internally from Identity * 1e-5: diagonal floored at 0.01, plus 1e-9 * I,
// inverted (reproduced in support/room_match.hpp).
// Precondition, by construction: every scan point is inside the grid at both
// poses (room <= 4.1 m, poses within 1.5 m, 20 m map), so no point is charged
// as off the grid in either cost.
RC_GTEST_PROP(ScanMatchingProperty, MatchIsNeverWorseThanOdometry, ())
{
  auto scaled = [](int lim, double unit) {
      return rc::gen::map(rc::gen::inRange(-lim, lim + 1), [unit](int v) {return v * unit;});
    };
  // Drawn into locals one by one so a failure prints the values in this order.
  const double x = *scaled(1500, 1e-3);
  const double y = *scaled(1000, 1e-3);
  const double th = *scaled(3141, 1e-3);
  const double dx = *scaled(50, 1e-3);
  const double dy = *scaled(50, 1e-3);
  const double dth = *scaled(30, 1e-3);
  const Eigen::Vector3d truth(x, y, th);
  const Eigen::Vector3d start = truth + Eigen::Vector3d(dx, dy, dth);

  const auto out = slam_test::match_from(truth, start);
  RC_ASSERT(out.cost_result <= out.cost_start + 1e-9);
}

// Rows equal the used beams with finite range r > 0 in [range_min, range_max); every
// row's norm lies in the same window (to float rounding).
RC_GTEST_PROP(ScanMatchingProperty, ExtractLocalScanKeepsExactlyInWindowBeams, ())
{
  const float inf = std::numeric_limits<float>::infinity();
  const auto range_gen = rc::gen::oneOf(
    rc::gen::map(rc::gen::inRange(-100, 2001), [](int v) {return v / 100.0f;}),
    rc::gen::just(inf), rc::gen::just(-inf), rc::gen::just(std::nanf("")));
  const auto ranges = *rc::gen::container<std::vector<float>>(range_gen);
  const float range_min = *rc::gen::map(rc::gen::inRange(0, 101), [](int v) {return v / 100.0f;});
  const float range_max =
    *rc::gen::map(rc::gen::inRange(101, 1501), [](int v) {return v / 100.0f;});
  const int stride = *rc::gen::inRange(1, 5);
  const float inc = *rc::gen::element(0.01f, 0.1f, -0.1f, 0.5f);

  const int n = static_cast<int>(ranges.size());
  const auto out = slam::extract_local_scan(
    ranges.data(), n, 0.0f, inc, range_min, range_max, stride);

  int expected = 0;
  for (int i = 0; i < n; i += stride) {
    const float r = ranges[i];
    if (std::isfinite(r) && r > 0.0f && r >= range_min && r < range_max) {++expected;}
  }
  RC_ASSERT(out.rows() == expected);
  for (int k = 0; k < out.rows(); ++k) {
    const double norm = std::hypot(out(k, 0), out(k, 1));
    RC_ASSERT(norm >= range_min - 1e-5);
    RC_ASSERT(norm < range_max + 1e-5);
  }
}

// On an empty grid every point reads 0.5 whether it is inside the grid or not,
// so the scan term is N * 0.5 * (0.5 / sigma_hit)^2 for any points and pose.
RC_GTEST_PROP(ScanMatchingProperty, EmptyGridCostIsConstantPerPoint, ())
{
  slam::SlamConfig config = make_config();
  config.map_size = 20.0;
  const slam::SigmoidLUT sigmoid(config.log_odds_scale);
  const slam::OccupancyGrid grid(config.map_size, config.map_size, config.map_resolution, config);
  const slam::GridView gv = slam::make_grid_view(grid, config, sigmoid);

  auto scaled = [](int lim, double unit) {
      return rc::gen::map(rc::gen::inRange(-lim, lim + 1), [unit](int v) {return v * unit;});
    };
  const int n = *rc::gen::inRange(0, 51);
  Eigen::Matrix<double, Eigen::Dynamic, 2> pts(n, 2);
  for (int k = 0; k < n; ++k) {
    pts(k, 0) = *scaled(200000, 1e-3);
    pts(k, 1) = *scaled(200000, 1e-3);
  }
  const Eigen::Vector3d pose(*scaled(50000, 1e-3), *scaled(50000, 1e-3), *scaled(3000, 1e-3));
  const Eigen::Vector3d odom_pose = Eigen::Vector3d::Zero();
  const Eigen::Matrix3d odom_info = Eigen::Matrix3d::Identity();

  const double cost =
    slam::compute_joint_cost(gv, pts, pose, odom_pose, odom_info, config.sigma_hit);
  const double odom_term = 0.5 * (pose - odom_pose).squaredNorm();  // |theta| < pi: no wrap
  const double per_point = 0.5 * std::pow(0.5 / config.sigma_hit, 2);
  RC_ASSERT(std::abs(cost - odom_term - n * per_point) < 1e-6);
}

// With every point at least one cell inside the grid, the cost is the sum of
// 0.5 * ((1 - m) / sigma_hit)^2 over the bilinear values m, plus the odometry
// term; nothing is charged for being off the grid.
RC_GTEST_PROP(ScanMatchingProperty, InGridCostIsSumOfInterpolatedResiduals, ())
{
  static const slam::SlamConfig config = make_config();
  static const slam::SigmoidLUT sigmoid(config.log_odds_scale);
  static const slam::OccupancyGrid grid = [] {
      slam::OccupancyGrid g(config.map_size, config.map_size, config.map_resolution, config);
      std::mt19937 rng(11);
      for (const Eigen::Vector3d & p :
        {Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(1.0, 0.5, 0.3),
          Eigen::Vector3d(-1.0, -0.5, -0.4)})
      {
        g.update(p, slam_test::scan_from(p, 4.0, 3.0, 0.01, rng));
      }
      return g;
    }();
  const slam::GridView gv = slam::make_grid_view(grid, config, sigmoid);

  auto scaled = [](int lim, double unit) {
      return rc::gen::map(rc::gen::inRange(-lim, lim + 1), [unit](int v) {return v * unit;});
    };
  // Map half-width 5 m. |local| <= 2.8 * sqrt(2) = 3.96 m and |translation| <=
  // 0.5 * sqrt(2) = 0.71 m put every world point within 4.67 m of the origin,
  // more than one cell (0.1 m) inside the edge.
  const int n = *rc::gen::inRange(1, 51);
  Eigen::Matrix<double, Eigen::Dynamic, 2> pts(n, 2);
  for (int k = 0; k < n; ++k) {
    pts(k, 0) = *scaled(2800, 1e-3);
    pts(k, 1) = *scaled(2800, 1e-3);
  }
  const Eigen::Vector3d pose(*scaled(500, 1e-3), *scaled(500, 1e-3), *scaled(3000, 1e-3));
  const Eigen::Vector3d odom_pose = Eigen::Vector3d::Zero();
  const Eigen::Matrix3d odom_info = Eigen::Matrix3d::Identity();

  const auto pix = slam::to_grid_coords(
    slam::transform_scan_to_world(pts, pose), gv.inv_res, gv.sample_origin);
  const auto interp = slam::bilinear_interpolation(gv, pix);
  double expected = 0.0;
  for (int k = 0; k < n; ++k) {
    RC_ASSERT(interp.mask(k) == 1);
    const double r = (1.0 - interp.values(k)) / config.sigma_hit;
    expected += 0.5 * r * r;
  }
  expected += 0.5 * (pose - odom_pose).squaredNorm();

  const double cost =
    slam::compute_joint_cost(gv, pts, pose, odom_pose, odom_info, config.sigma_hit);
  RC_ASSERT(std::abs(cost - expected) < 1e-9 * std::max(1.0, expected));
}

// The median range lies between the nearest and farthest point, and the
// spacing is linear in the angle between points.
RC_GTEST_PROP(ScanMatchingProperty, MedianPointSpacingBoundedAndLinear, ())
{
  auto scaled = [](int lim, double unit) {
      return rc::gen::map(rc::gen::inRange(-lim, lim + 1), [unit](int v) {return v * unit;});
    };
  const int n = *rc::gen::inRange(1, 51);
  Eigen::Matrix<double, Eigen::Dynamic, 2> pts(n, 2);
  for (int k = 0; k < n; ++k) {
    pts(k, 0) = *scaled(10000, 1e-3);
    pts(k, 1) = *scaled(10000, 1e-3);
  }
  const double angle = *scaled(500, 1e-3);

  const Eigen::VectorXd norms = pts.rowwise().norm();
  const double lo = norms.minCoeff() * std::abs(angle);
  const double hi = norms.maxCoeff() * std::abs(angle);
  const double s = slam::median_point_spacing(pts, angle);
  RC_ASSERT(s >= lo - 1e-12);
  RC_ASSERT(s <= hi + 1e-12);

  const double s2 = slam::median_point_spacing(pts, 2.0 * angle);
  RC_ASSERT(std::abs(s2 - 2.0 * s) <= 1e-12 * std::max(1.0, 2.0 * s));
}
