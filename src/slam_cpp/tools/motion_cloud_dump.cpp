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

/// @file motion_cloud_dump.cpp
/// @brief Writes the particle poses after a fixed sequence of motions, with no scan update.
///
/// Usage: `motion_cloud_dump <out.csv>`.
/// Output columns: `step` (0 is the start; 1 to 8 each move 0.25 m with a 0.1 rad turn before and
/// after, forward, so the path is an arc), `particle` (-1 is the noise-free pose), `x`, `y`
/// (metres), `theta` (radians).

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#include "slam_cpp/particle_filter.hpp"
#include "slam_cpp/slam_config.hpp"
#include "slam_cpp/slam_types.hpp"

int main(int argc, char ** argv)
{
  if (argc != 2 || std::string(argv[1]).rfind("--", 0) == 0) {
    std::cerr << "usage: motion_cloud_dump <out.csv>\n";
    return 2;
  }
  std::ofstream f(argv[1]);
  if (!f) {
    std::cerr << "cannot open " << argv[1] << "\n";
    return 1;
  }
  slam::ParticleFilter pf{slam::SlamConfig{}, 42};
  f << std::setprecision(10) << "step,particle,x,y,theta\n";
  const double trans = 0.25, rot1 = 0.1, rot2 = 0.1;
  double nx = 0.0, ny = 0.0, nth = 0.0;
  for (int step = 0; step <= 8; ++step) {
    if (step >= 1) {
      pf.predict(trans, rot1, rot2, 1.0);
      nth += rot1;
      nx += trans * std::cos(nth);
      ny += trans * std::sin(nth);
      nth = slam::normalize_angle(nth + rot2);
    }
    f << step << ",-1," << nx << "," << ny << "," << nth << "\n";
    for (Eigen::Index p = 0; p < pf.poses().cols(); ++p) {
      f << step << "," << p << "," << pf.poses()(0, p) << "," << pf.poses()(1, p) << "," <<
        pf.poses()(2, p) << "\n";
    }
  }
  return 0;
}
