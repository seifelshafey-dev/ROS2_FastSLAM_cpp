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

#ifndef SLAM_CPP__SLAM_TYPES_HPP_
#define SLAM_CPP__SLAM_TYPES_HPP_

/**
 * @file slam_types.hpp
 * @brief Angle constants, angle wrapping, and the result and view structs shared by the
 *        scan matcher and the particle filter.
 */

#include <Eigen/Dense>

#include <cmath>
#include <cstdint>
#include <vector>

namespace slam
{

class SigmoidLUT;  // forward declaration (sigmoid_lut.hpp)

/// The constant pi (radians).
inline constexpr double kPi = 3.14159265358979323846;
/// Two pi (radians).
inline constexpr double kTwoPi = 2.0 * kPi;

/**
 * @brief Wrap an angle to the half-open interval [-pi, pi).
 *
 * @param angle Angle in radians, any finite value.
 * @return The equivalent angle in [-pi, pi) (radians); an input of exactly pi returns -pi.
 *         A non-finite input gives NaN.
 */
inline double normalize_angle(double angle)
{
  // fmod keeps the sign of the dividend, so a negative result is shifted up by 2*pi.
  angle = std::fmod(angle + kPi, kTwoPi);
  if (angle < 0.0) {
    angle += kTwoPi;
  }
  angle -= kPi;
  if (angle >= kPi) {
    angle -= kTwoPi;  // rounding can land exactly on pi
  }
  return angle;
}

class OccupancyGrid;  // forward declaration (occupancy_grid.hpp)

/// Bilinear interpolation of the grid at a set of points, optionally with spatial gradients.
struct BilinearResult
{
  /// Occupancy probability per query point, in [0, 1]; 0 where `mask` is 0.
  Eigen::VectorXd values;
  /// Gradient of `values` per query point, two columns (x, y), per cell in sample
  /// coordinates; empty unless requested.
  Eigen::Matrix<double, Eigen::Dynamic, 2> gradients;
  /// Per query point: 1 if all four surrounding cells are on the grid, else 0.
  Eigen::VectorXi mask;
};

/// Output of the Levenberg-Marquardt pose refinement.
struct LMResult
{
  /// Refined pose (x, y, theta) in the map frame; metres and radians.
  Eigen::Vector3d pose;
  /// Information matrix of the pose: J^T J plus the odometry information at `pose`.
  /// Only the odometry information if fewer than 10 scan points fall on the grid.
  Eigen::Matrix3d hessian;
  /// False only if fewer than 10 scan points fall on the grid at the pose that was linearised.
  bool success;
};

/// Scan matching output for one particle.
struct ScanMatchResult
{
  /// Optimized or fallback pose (x, y, theta) in the map frame; metres and radians.
  Eigen::Vector3d pose;
  /// Covariance of the proposal distribution around `pose`.
  Eigen::Matrix3d proposal_cov;
  /// Natural-log importance weight, scaled by 1 / (obs_sigma_gain * num_particles), <= 0.
  double log_weight;
};

/// Residuals and Jacobian of the scan cost at one pose.
struct JacobianResult
{
  /// Residual per valid scan point; size n_valid.
  Eigen::VectorXd residuals;
  /// Derivative of each residual with respect to (x, y, theta); n_valid rows, 3 columns.
  Eigen::Matrix<double, Eigen::Dynamic, 3> jacobian;
  /// Row indices of the scan points that produced a residual.
  std::vector<int> valid_indices;
  /// False if fewer than 10 scan points are valid.
  bool success = false;
};

/// Read-only view of an OccupancyGrid for scan matching. It does not own the grid or the
/// lookup table; both must outlive the view.
struct GridView
{
  /// Cell values, row-major (row is y, column is x): int8 log-odds times the log-odds scale.
  const int8_t * data;
  /// Number of rows (cells along y).
  int rows;
  /// Number of columns (cells along x).
  int cols;
  /// 1 / map resolution, in cells per metre.
  double inv_res;
  /// Offset in cells such that x * inv_res + sample_origin is the sample coordinate of
  /// world x (m). Cell i's value sits at sample coordinate i, the centre of the cell, so
  /// sample_origin = cols/2 - 0.5. One offset serves both axes because the grid is square.
  double sample_origin;
  /// Lookup table converting a stored cell value to an occupancy probability. Not owned.
  const SigmoidLUT * sigmoid;
};

}  // namespace slam

#endif  // SLAM_CPP__SLAM_TYPES_HPP_
