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

#include <chrono>
#include <cmath>
#include <iostream>
#include <random>

#include "slam_cpp/slam_config.hpp"
#include "slam_cpp/particle_filter.hpp"
#include "support/room.hpp"

namespace
{

struct CircleRun
{
  double rms = 0, max = 0, min_neff = 1e9, mean_ms = 0;
  Eigen::Vector3d final_err = Eigen::Vector3d::Zero();
  int resamples = 0, best_changes = 0;
};

// Full filter on a circle of radius 1.5 m, 400 steps of 0.04 rad, exact
// odometry, room 4.03 x 3.02 (half sizes), range noise 0.01 m. The scan
// generator seed is 7. The best particle is compared with the true pose
// relative to the first true pose (the filter's frame starts there).
CircleRun run_circle(uint64_t filter_seed)
{
  const double pi = std::acos(-1.0);
  slam::SlamConfig cfg;
  cfg.map_size = 20.0;
  slam::ParticleFilter pf(cfg, filter_seed);
  std::mt19937 rng(7);
  const double r = 1.5, dth = 0.04, chord = 2 * r * std::sin(dth / 2);
  const int steps = 400;
  const Eigen::Vector3d t0(r, 0.0, pi / 2);

  CircleRun out;
  double se = 0, ms = 0;
  size_t prev_best = 0;
  for (int k = 0; k <= steps; ++k) {
    const double a = dth * k;
    const Eigen::Vector3d truth(r * std::cos(a), r * std::sin(a),
      std::remainder(a + pi / 2, 2 * pi));
    const auto z = slam_test::scan_from(truth, 4.03, 3.02, 0.01, rng);
    const auto c0 = std::chrono::steady_clock::now();
    if (k > 0) {pf.predict(chord, dth / 2, dth / 2, 1.0);}
    pf.update_and_resample(z);
    ms += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - c0).count();

    const double neff = 1.0 / pf.weights().squaredNorm();
    out.min_neff = std::min(out.min_neff, neff);
    if (k > 0 && (pf.weights().array() - 1.0 / cfg.num_particles).abs().maxCoeff() < 1e-15) {
      ++out.resamples;
    }
    const size_t b = pf.best_particle_index();
    if (k > 0 && b != prev_best) {++out.best_changes;}
    prev_best = b;

    const Eigen::Vector3d est = pf.poses().col(static_cast<Eigen::Index>(b));
    const Eigen::Vector3d tr = slam_test::rel(t0, truth);
    Eigen::Vector3d e = est - tr;
    e(2) = std::remainder(e(2), 2 * pi);
    se += e.head<2>().squaredNorm();
    out.max = std::max(out.max, e.head<2>().norm());
    out.final_err = e;
  }
  out.rms = std::sqrt(se / (steps + 1));
  out.mean_ms = ms / (steps + 1);
  return out;
}

void print(uint64_t seed, const CircleRun & c)
{
  std::cout << "seed=" << seed << " rms=" << c.rms << " max=" << c.max
            << " final_err=(" << c.final_err(0) << ", " << c.final_err(1) << ", " <<
    c.final_err(2) << ")"
            << " Neff_min=" << c.min_neff << " resamples=" << c.resamples
            << " best_changes=" << c.best_changes << " ms/update=" << c.mean_ms << "\n";
}

}  // namespace

TEST(ParticleFilterIntegration, CircleWithExactOdometryTracksTruth)
{
  const CircleRun c = run_circle(42);
  print(42, c);
  // Bounds are 2x the worst of filter seeds 1..5 and 42, rounded up to one
  // significant figure. Measured worst case: rms 0.0370 m, final position
  // error 0.0408 m (both seed 42).
  EXPECT_LT(c.rms, 0.08);
  EXPECT_LT(c.final_err.head<2>().norm(), 0.09);
}
