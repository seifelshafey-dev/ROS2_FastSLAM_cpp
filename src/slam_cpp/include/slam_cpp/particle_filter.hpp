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

#ifndef SLAM_CPP__PARTICLE_FILTER_HPP_
#define SLAM_CPP__PARTICLE_FILTER_HPP_

/// @file
/// @brief Rao-Blackwellized particle filter that estimates the robot pose and builds one
///        occupancy grid per particle.

// SOA layout: poses in a 3xN matrix (vectorized trig), grids stored separately (250KB each).
// Per-particle scan matching is parallelized with OpenMP; RNG is sequential for determinism.

#include <Eigen/Dense>

#include <random>
#include <vector>

#include "slam_cpp/slam_config.hpp"
#include "slam_cpp/occupancy_grid.hpp"
#include "slam_cpp/sigmoid_lut.hpp"

namespace slam
{

/// A set of `config.num_particles` particles, each with a pose (x, y, theta), a
/// weight and its own occupancy grid.
///
/// Poses are in the map frame, in metres and radians. The map frame is the
/// robot's pose at the first update: all particles start at (0, 0, 0).
/// Call predict() with each motion step, then update_and_resample() with
/// the scan taken after it.
///
/// Results are reproducible for a given seed and scan sequence, whatever
/// the number of threads.
class ParticleFilter
{
public:
  /// Creates a filter whose random numbers are seeded from hardware entropy.
  ///
  /// @param config  Filter settings; checked on construction.
  /// @throws std::invalid_argument if the configuration is invalid.
  explicit ParticleFilter(const SlamConfig & config);

  /// Creates a filter with a fixed seed, so a run can be repeated exactly.
  ///
  /// @param config  Filter settings; checked on construction.
  /// @param seed    Seed of the random number generator. Only the low 32 bits are used:
  ///                seeds that differ only in the high 32 bits give the same run.
  /// @throws std::invalid_argument if the configuration is invalid.
  ParticleFilter(const SlamConfig & config, uint64_t seed);

  /// Moves every particle by a noisy rotation, translation, rotation step.
  ///
  /// Each particle turns by `rot1`, moves `trans` along that heading (times
  /// `direction`), then turns by `rot2`, with noise drawn from the motion
  /// noise settings in the configuration. Headings are wrapped to [-pi, pi).
  ///
  /// @param trans      Distance moved, in metres; finite and >= 0. The sign of the motion
  ///                   comes from `direction`.
  /// @param rot1       Rotation before the move, in radians; finite.
  /// @param rot2       Rotation after the move, in radians; finite.
  /// @param direction  +1.0 for forward motion, -1.0 for reverse; other finite values scale the
  ///                   translation.
  /// @throws std::invalid_argument if `trans` is negative or any argument is not finite.
  void predict(double trans, double rot1, double rot2, double direction);

  /// Measurement update.
  ///
  /// The first call registers the scan into every grid at the particle's
  /// current pose, without matching; poses and weights are unchanged. Later
  /// calls scan match, sample from the proposal, update weights, resample
  /// when N_eff < N/2, and update the occupancy grids.
  ///
  /// The grids are updated only when the best particle has moved or turned
  /// more than the map update thresholds in the configuration since the last
  /// grid update.
  ///
  /// @param local_scans  Hit points in the base frame, one row (x, y), in metres.
  /// @throws std::invalid_argument if `local_scans` has no rows.
  void update_and_resample(
    const Eigen::Matrix<double, Eigen::Dynamic, 2> & local_scans);

  /// Index of the particle with the largest cumulative log weight along its
  /// ancestry. Index 0 on a fresh filter; among copies of the same ancestor
  /// after resampling, the lowest index.
  ///
  /// @return Particle index in `[0, num_particles)`.
  size_t best_particle_index() const;

  /// Particle poses, one column (x, y, theta) per particle, in the map frame: metres, metres,
  /// radians.
  /// @return 3 x N matrix of poses, in the map frame.
  const Eigen::Matrix<double, 3, Eigen::Dynamic> & poses() const {return poses_;}
  /// Normalised particle weights, one per particle, summing to 1.
  /// @return Vector of N weights, dimensionless.
  const Eigen::VectorXd & weights() const {return weights_;}
  /// The occupancy grid of each particle, in particle order.
  /// @return N grids, each in the map frame, cell values as in OccupancyGrid::at().
  const std::vector<OccupancyGrid> & grids() const {return grids_;}
  /// The configuration the filter was built with.
  /// @return Reference to the stored configuration.
  const SlamConfig & config() const {return config_;}
  /// Number of calls to update_and_resample() so far, including the first.
  /// @return Call count; 0 before the first call.
  int global_counter() const {return global_counter_;}

private:
  SlamConfig config_;
  int N_;
  SigmoidLUT sigmoid_;

  // SOA particle state
  Eigen::Matrix<double, 3, Eigen::Dynamic> poses_;  // 3 x N
  Eigen::VectorXd weights_;                          // N x 1
  std::vector<Eigen::Matrix3d> covs_;                // N covariance matrices
  std::vector<OccupancyGrid> grids_;                 // N grids (COW internally)

  // Running sum of each particle's log weights along its ancestry, shifted
  // after each matched update so that its maximum is zero at that moment.
  // Used only to pick the best particle.
  Eigen::VectorXd cum_log_weights_;                  // N x 1

  // Map quality cache
  Eigen::VectorXd cached_map_quality_;
  int last_quality_update_ = -5;

  // Sparse map update tracking
  Eigen::Vector3d last_map_update_pose_;
  bool has_last_map_update_pose_ = false;

  // RNG: every random number is drawn from this one generator.
  std::mt19937 rng_;
  int global_counter_ = 0;

  void update_map_quality_cache();
  std::vector<size_t> systematic_resample();
  void init_particles();
};

}  // namespace slam

#endif  // SLAM_CPP__PARTICLE_FILTER_HPP_
