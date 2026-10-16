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
#include <cstdlib>
#include <map>
#include <utility>

#include "slam_cpp/occupancy_grid.hpp"
#include "slam_cpp/slam_config.hpp"

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

// A point inside cell k + u (u in [0.01, 0.99]) maps to cell k + cols/2; the lower edge
// -map_size/2 is in cell 0, the upper edge +map_size/2 and anything beyond either edge is outside.
RC_GTEST_PROP(OccupancyGridProperty, WorldToGridMapsCellInteriorsAndRejectsOutside, ())
{
  const slam::SlamConfig config = make_config();
  slam::OccupancyGrid grid(config.map_size, config.map_size, config.map_resolution, config);
  const int cols = grid.cols(), rows = grid.rows();
  const double res = config.map_resolution;

  const int kx = *rc::gen::inRange(-cols / 2, cols / 2);
  const int ky = *rc::gen::inRange(-rows / 2, rows / 2);
  const double x = (kx + *fraction()) * res;
  const double y = (ky + *fraction()) * res;
  const auto cell = grid.world_to_grid(x, y);
  RC_ASSERT(cell.has_value());
  RC_ASSERT(cell->x == kx + cols / 2);
  RC_ASSERT(cell->y == ky + rows / 2);

  // The cell range is [-half, half): the lower edge is inside, the upper edge is not.
  const double half = config.map_size / 2.0;
  const auto lower = grid.world_to_grid(-half, -half);
  RC_ASSERT(lower.has_value());
  RC_ASSERT(lower->x == 0 && lower->y == 0);
  const double over = *rc::gen::inRange(0, 1000) * 1e-3;
  RC_ASSERT(!grid.world_to_grid(half + over, 0.0).has_value());
  RC_ASSERT(!grid.world_to_grid(0.0, half + over).has_value());
  RC_ASSERT(!grid.world_to_grid(-half - over - 1e-3, 0.0).has_value());
  RC_ASSERT(!grid.world_to_grid(0.0, -half - over - 1e-3).has_value());
}

// One beam on a fresh grid changes exactly D + 1 cells (D = larger cell distance
// between robot and hit): each index along the major axis once, the minor index
// within 0.5 of the straight line, the hit cell occupied and the rest free.
// Robot and hit are drawn anywhere in the grid interior, so all octants occur.
RC_GTEST_PROP(OccupancyGridProperty, RayTraceChangesDPlusOneCellsAlongTheLine, ())
{
  const slam::SlamConfig config = make_config();
  slam::OccupancyGrid grid(config.map_size, config.map_size, config.map_resolution, config);
  const int cols = grid.cols(), rows = grid.rows();
  const double res = config.map_resolution;

  const int c0 = *rc::gen::inRange(5, cols - 5), r0 = *rc::gen::inRange(5, rows - 5);
  const int c1 = *rc::gen::inRange(5, cols - 5), r1 = *rc::gen::inRange(5, rows - 5);
  const double x0 = (c0 - cols / 2 + *fraction()) * res;
  const double y0 = (r0 - rows / 2 + *fraction()) * res;
  const double x1 = (c1 - cols / 2 + *fraction()) * res;
  const double y1 = (r1 - rows / 2 + *fraction()) * res;

  Eigen::Matrix<double, Eigen::Dynamic, 2> hit(1, 2);
  hit << x1 - x0, y1 - y0;
  grid.update(Eigen::Vector3d(x0, y0, 0.0), hit);

  const int dc = c1 - c0, dr = r1 - r0;
  const int D = std::max(std::abs(dc), std::abs(dr));
  const bool major_is_col = std::abs(dc) >= std::abs(dr);

  std::map<int, int> per_major;  // major index -> number of changed cells
  int changed = 0;
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      const int v = grid.at(r, c);
      if (v == 0) {continue;}
      ++changed;
      const bool is_hit = (c == c1 && r == r1);
      RC_ASSERT(v == (is_hit ? config.lo_occupied_scaled() : config.lo_free_scaled()));
      const int major = major_is_col ? c : r;
      const int minor = major_is_col ? r : c;
      ++per_major[major];
      const int m0 = major_is_col ? c0 : r0, m1 = major_is_col ? c1 : r1;
      const int n0 = major_is_col ? r0 : c0, n1 = major_is_col ? r1 : c1;
      RC_ASSERT(major >= std::min(m0, m1) && major <= std::max(m0, m1));
      const double line = D ==
        0 ? n0 : n0 + (n1 - n0) * static_cast<double>(major - m0) / (m1 - m0);
      RC_ASSERT(std::abs(minor - line) <= 0.5 + 1e-9);
    }
  }
  RC_ASSERT(changed == D + 1);
  RC_ASSERT(static_cast<int>(per_major.size()) == D + 1);
  for (const auto & kv : per_major) {
    RC_ASSERT(kv.second == 1);
  }
}
