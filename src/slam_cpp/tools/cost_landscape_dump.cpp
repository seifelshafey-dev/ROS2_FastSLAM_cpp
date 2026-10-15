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

/// @file cost_landscape_dump.cpp
/// @brief Writes the joint scan-matching cost on a grid of offsets around the true pose, for a
/// map built from one scan and for a map built along a straight line.
///
/// Usage: `cost_landscape_dump <out.csv> [--res metres]` (default 0.1).
/// Output columns: `map` (`one` or `full`), `dx`, `dy` (offset from the true pose, metres),
/// `cost` (unitless).

#include <Eigen/Dense>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>

#include "slam_cpp/occupancy_grid.hpp"
#include "slam_cpp/scan_matching.hpp"
#include "slam_cpp/sigmoid_lut.hpp"
#include "slam_cpp/slam_config.hpp"
#include "slam_cpp/slam_types.hpp"
#include "support/room.hpp"

namespace
{
const char kUsage[] = "usage: cost_landscape_dump <out.csv> [--res metres]";

bool parse_double(const char * s, double & v)
{
  char * end = nullptr;
  v = std::strtod(s, &end);
  return end != s && *end == '\0' && std::isfinite(v);
}
}  // namespace

int main(int argc, char ** argv)
{
  std::string out;
  double res = 0.1;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--res" && i + 1 < argc && parse_double(argv[++i], res) && res > 0.0) {
      continue;
    }
    if (a.rfind("--", 0) != 0 && out.empty()) {
      out = a;
      continue;
    }
    std::cerr << kUsage << "\n";
    return 2;
  }
  if (out.empty()) {
    std::cerr << kUsage << "\n";
    return 2;
  }

  slam::SlamConfig cfg;
  cfg.map_size = 20.0;
  cfg.map_resolution = res;
  const double hx = 4.925, hy = 3.925;
  std::mt19937 rng(7);
  slam::SigmoidLUT lut(cfg.log_odds_scale);
  const Eigen::Matrix3d info = 100.0 * Eigen::Matrix3d::Identity();

  slam::OccupancyGrid one(cfg.map_size, cfg.map_size, cfg.map_resolution, cfg);
  one.update(Eigen::Vector3d(0, 0, 0), slam_test::scan_from({0, 0, 0}, hx, hy, 0.01, rng, 1));

  slam::OccupancyGrid full(cfg.map_size, cfg.map_size, cfg.map_resolution, cfg);
  for (int i = 0; i < 100; ++i) {
    const Eigen::Vector3d p(0.015 * i, 0.0, 0.0);
    full.update(p, slam_test::scan_from(p, hx, hy, 0.01, rng, 1));
  }

  std::ofstream f(out);
  if (!f) {
    std::cerr << "cannot open " << out << "\n";
    return 1;
  }
  f << std::setprecision(10) << "map,dx,dy,cost\n";
  struct Case
  {
    const char * name;
    const slam::OccupancyGrid * grid;
    Eigen::Vector3d truth;
  };
  const Case cases[] = {{"one", &one, {0.06, 0.0, 0.0}}, {"full", &full, {0.75, 0.0, 0.0}}};
  for (const Case & c : cases) {
    const slam::GridView gv{c.grid->data(), c.grid->rows(), c.grid->cols(),
      1.0 / cfg.map_resolution, c.grid->cols() / 2 - 0.5, &lut};
    const auto scan = slam_test::scan_from(c.truth, hx, hy, 0.01, rng, 1);
    for (int iy = -30; iy <= 30; ++iy) {
      for (int ix = -30; ix <= 30; ++ix) {
        const double dx = ix * 0.01, dy = iy * 0.01;
        const double cost = slam::compute_joint_cost(
          gv, scan, c.truth + Eigen::Vector3d(dx, dy, 0.0), c.truth, info, cfg.sigma_hit);
        f << c.name << "," << dx << "," << dy << "," << cost << "\n";
      }
    }
  }
  return 0;
}
