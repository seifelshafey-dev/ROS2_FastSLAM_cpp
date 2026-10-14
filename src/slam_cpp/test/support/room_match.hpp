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

// Test helper: scan_match_single on a map built from a room scan at the
// origin, with the joint cost used to compare the result with the start.
#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <random>

#include "slam_cpp/slam_config.hpp"
#include "slam_cpp/occupancy_grid.hpp"
#include "slam_cpp/scan_matching.hpp"
#include "slam_cpp/sigmoid_lut.hpp"
#include "room.hpp"

namespace slam_test
{

struct RoomMap
{
  slam::SlamConfig config;
  slam::SigmoidLUT lut;
  slam::OccupancyGrid grid;
};

// Map size 20 m, 20 updates from (0,0,0), room half sizes 4.03 / 3.02, noise
// 0.01 m, seed 7. Built once.
inline const RoomMap & room_map()
{
  static const RoomMap map = [] {
      slam::SlamConfig c;
      c.map_size = 20.0;
      slam::SigmoidLUT lut(c.log_odds_scale);
      slam::OccupancyGrid g(c.map_size, c.map_size, c.map_resolution, c);
      std::mt19937 rng(7);
      const Eigen::Vector3d origin(0.0, 0.0, 0.0);
      const auto scan = scan_from(origin, 4.03, 3.02, 0.01, rng);
      for (int i = 0; i < 20; ++i) {
        g.update(origin, scan);
      }
      return RoomMap{c, lut, g};
    }();
  return map;
}

struct MatchOutcome
{
  Eigen::Vector3d pose;  // scan_match_single result (map frame; m, rad)
  double cost_result;    // compute_joint_cost(result, odometry = start)
  double cost_start;     // compute_joint_cost(start, odometry = start)
};

// Zero-noise scan from `truth` (seed 3), matched from `start` with prior
// covariance Identity * 1e-5. Costs use the odom_info that scan_match_single
// builds internally: diagonal floored at 0.01, plus 1e-9 * I, inverted
// (reproduced here). The room (<= 4.1 m) in a 20 m map keeps every point
// inside the grid for poses within 1.5 m of the origin, so no point is charged
// as off the grid.
inline MatchOutcome match_from(const Eigen::Vector3d & truth, const Eigen::Vector3d & start)
{
  const RoomMap & m = room_map();
  std::mt19937 rng(3);
  const auto scan = scan_from(truth, 4.03, 3.02, 0.0, rng);

  const double cov_in = 1e-5;
  const double floored = std::max(cov_in, 0.01);
  const Eigen::Matrix3d odom_info = (Eigen::Matrix3d::Identity() * (floored + 1e-9)).inverse();

  const auto res = slam::scan_match_single(
    m.grid, scan, start, Eigen::Matrix3d::Identity() * cov_in, m.grid.map_quality(), m.config,
    m.lut);
  const slam::GridView gv = slam::make_grid_view(m.grid, m.config, m.lut);
  return {res.pose,
    slam::compute_joint_cost(gv, scan, res.pose, start, odom_info, m.config.sigma_hit),
    slam::compute_joint_cost(gv, scan, start, start, odom_info, m.config.sigma_hit)};
}

}  // namespace slam_test
