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

/// @file filter_run_dump.cpp
/// @brief Runs the particle filter on an analytic rectangular room along the simulator drive
/// script with exact odometry, and writes one row per filter update. `--seed` seeds only the
/// particle filter; the scan noise uses a fixed generator (seed 7), so the scans are identical
/// for every `--seed`.
///
/// Usage: `filter_run_dump <out.csv> [--res m] [--beam-step n] [--seed n] [--noise m] [--hx m]
/// [--hy m] [--map-update-dist m]`. Defaults: 0.1, 1, 42, 0.01, 4.925, 3.925, and the
/// `SlamConfig` default. Prints one summary line to stdout.
/// Output columns: `k` (tick), `t` (s), `phase` (`S` straight, `T` turning), `gt_*` and `est_*`
/// (pose x, y in metres, theta in radians; est is the best particle), `neff` (effective sample
/// size after the update), `resampled` (1 if the update resampled, else 0).

#include <Eigen/Dense>

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>

#include "slam_cpp/particle_filter.hpp"
#include "slam_cpp/slam_config.hpp"
#include "support/room.hpp"

namespace
{
const char kUsage[] =
  "usage: filter_run_dump <out.csv> [--res m] [--beam-step n] [--seed n] [--noise m] "
  "[--hx m] [--hy m] [--map-update-dist m]\n"
  "  --seed seeds the particle filter only; the scan noise is fixed (seed 7)";

bool parse_double(const char * s, double & v)
{
  char * end = nullptr;
  v = std::strtod(s, &end);
  return end != s && *end == '\0' && std::isfinite(v);
}

bool parse_int(const char * s, int64_t & v)
{
  char * end = nullptr;
  v = std::strtoll(s, &end, 10);
  return end != s && *end == '\0';
}
}  // namespace

int main(int argc, char ** argv)
{
  const double pi = std::acos(-1.0);
  slam::SlamConfig cfg;
  cfg.map_size = 20.0;
  std::string out;
  int64_t beam_step = 1, seed = 42;
  double noise = 0.01, hx = 4.925, hy = 3.925;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    const bool has = i + 1 < argc;
    bool ok = false;
    if (a.rfind("--", 0) != 0 && out.empty()) {
      out = a;
      continue;
    }
    if (!has) {
      std::cerr << kUsage << "\n";
      return 2;
    }
    const char * v = argv[++i];
    if (a == "--res") {
      ok = parse_double(v, cfg.map_resolution) && cfg.map_resolution > 0.0;
    } else if (a == "--beam-step") {
      ok = parse_int(v, beam_step) && beam_step >= 1;
    } else if (a == "--seed") {
      ok = parse_int(v, seed) && seed >= 0;
    } else if (a == "--noise") {
      ok = parse_double(v, noise) && noise >= 0.0;
    } else if (a == "--hx") {
      ok = parse_double(v, hx) && hx > 0.0;
    } else if (a == "--hy") {
      ok = parse_double(v, hy) && hy > 0.0;
    } else if (a == "--map-update-dist") {
      ok = parse_double(v, cfg.map_update_dist) && cfg.map_update_dist >= 0.0;
    }
    if (!ok) {
      std::cerr << kUsage << "\n";
      return 2;
    }
  }
  if (out.empty()) {
    std::cerr << kUsage << "\n";
    return 2;
  }
  std::ofstream f(out);
  if (!f) {
    std::cerr << "cannot open " << out << "\n";
    return 1;
  }
  f << std::setprecision(10) << "k,t,phase,gt_x,gt_y,gt_th,est_x,est_y,est_th,neff,resampled\n";

  const int bs = static_cast<int>(beam_step);
  slam::ParticleFilter pf(cfg, static_cast<uint64_t>(seed));
  std::mt19937 rng(7);
  const double dt = 0.1;
  Eigen::Vector3d truth(0, 0, 0), last = truth;
  double se = 0, sx = 0, sy = 0, rx = 0, ry = 0;
  int n = 0, nturn = 0;

  // Writes one row from the filter state after an update.
  auto row = [&](int k, bool straight) {
      const Eigen::Vector3d est =
        pf.poses().col(static_cast<Eigen::Index>(pf.best_particle_index()));
      const Eigen::VectorXd & w = pf.weights();
      const double neff = 1.0 / w.squaredNorm();
      const bool resampled = k > 0 &&
        (w.array() - 1.0 / static_cast<double>(w.size())).abs().maxCoeff() < 1e-9;
      f << k << "," << k * dt << "," << (straight ? 'S' : 'T') << "," << truth(0) << "," <<
        truth(1) << "," << truth(2) << "," << est(0) << "," << est(1) << "," << est(2) << "," <<
        neff << "," << (resampled ? 1 : 0) << "\n";
      return est;
    };

  pf.update_and_resample(slam_test::scan_from(truth, hx, hy, noise, rng, bs));
  row(0, true);
  for (int k = 1; k <= 1440; ++k) {
    const double ph = std::fmod(k * dt - 1e-9, 8.93);
    const bool straight = ph < 5.0;
    if (straight) {
      truth(0) += 0.3 * dt * std::cos(truth(2));
      truth(1) += 0.3 * dt * std::sin(truth(2));
    } else {
      truth(2) = std::remainder(truth(2) - 0.4 * dt, 2 * pi);
    }
    const Eigen::Vector3d d = slam_test::rel(last, truth);
    const double tr = d.head<2>().norm();
    if (tr < cfg.motion_threshold_trans && std::abs(d(2)) < cfg.motion_threshold_rot) {
      continue;
    }
    const double r1 = tr > 1e-6 ? std::atan2(d(1), d(0)) : d(2) / 2;
    pf.predict(tr, r1, std::remainder(d(2) - r1, 2 * pi), 1.0);
    pf.update_and_resample(slam_test::scan_from(truth, hx, hy, noise, rng, bs));
    last = truth;
    const Eigen::Vector3d est = row(k, straight);
    const double ex = est(0) - truth(0), ey = est(1) - truth(1);
    sx += ex;
    sy += ey;
    se += ex * ex + ey * ey;
    ++n;
    if (!straight) {
      rx += ex;
      ry += ey;
      ++nturn;
    }
  }
  if (n == 0) {
    std::cerr << "no update after the first scan\n";
    return 1;
  }
  std::cout << "updates=" << n << " rms=" << std::sqrt(se / n) << " mean=(" << sx / n << "," <<
    sy / n << ") mean_turning=(" << (nturn ? rx / nturn : 0.0) << "," <<
    (nturn ? ry / nturn : 0.0) << ")\n";
  return 0;
}
