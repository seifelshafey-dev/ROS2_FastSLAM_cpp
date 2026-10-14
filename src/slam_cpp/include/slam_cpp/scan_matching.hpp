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

#ifndef SLAM_CPP__SCAN_MATCHING_HPP_
#define SLAM_CPP__SCAN_MATCHING_HPP_

/// @file
/// @brief Scan matching: coarse search, Levenberg-Marquardt refinement and scan likelihood
///        weighting against an occupancy grid.
///
/// Poses are (x, y, theta) in the map frame, in metres and radians. Scan
/// points are in the base frame, in metres. "Sample coordinates" are positions
/// in cells in which the value of cell i sits at coordinate i, at the cell centre.

#include <Eigen/Dense>

#include <vector>

#include "slam_cpp/slam_config.hpp"
#include "slam_cpp/slam_types.hpp"
#include "slam_cpp/occupancy_grid.hpp"
#include "slam_cpp/sigmoid_lut.hpp"

namespace slam
{

/// Builds a read-only view of a grid for the matching functions.
///
/// The view holds a pointer to the grid's cells and to `sigmoid`; both must
/// outlive the view, and the grid must not be written while the view is in use.
/// The grid must be square and `config.map_resolution` must equal the grid's
/// resolution; neither is checked.
///
/// @param grid     Grid to read.
/// @param config   Supplies `map_resolution`, in metres per cell.
/// @param sigmoid  Table that converts a stored cell value to an occupancy probability.
/// @return A view whose `sample_origin` is `cols/2 - 0.5`, so cell values are read at cell centres.
GridView make_grid_view(
  const OccupancyGrid & grid, const SlamConfig & config,
  const SigmoidLUT & sigmoid);

/// Converts a planar laser scan to hit points in the base frame.
///
/// Beam i has bearing angle_min + i * angle_increment. Beams are taken every
/// `stride` from index 0. A beam gives one row (x, y) in metres when its range
/// is finite, greater than zero and range_min <= range < range_max; any other
/// beam gives no row.
///
/// @param ranges           ranges in metres, `num_ranges` values
/// @param num_ranges       number of beams, >= 0
/// @param angle_min        bearing of beam 0 in radians, base frame
/// @param angle_increment  bearing step between beams in radians, may be negative
/// @param range_min        smallest valid range in metres
/// @param range_max        largest valid range in metres, exclusive
/// @param stride           use every stride-th beam, >= 1
/// @return one row per kept beam, in beam order; no rows if nothing is kept
///         or if angle_min or angle_increment is not finite
Eigen::Matrix<double, Eigen::Dynamic, 2> extract_local_scan(
  const float * ranges, int num_ranges, float angle_min, float angle_increment,
  float range_min, float range_max, int stride);

/// Arc length between neighbouring scan points at the median range.
///
/// @param local_scans           Scan points in the base frame, metres, one per row (as returned
///                              by extract_local_scan()).
/// @param angle_between_points  Angle between consecutive rows, radians: the lidar's angle
///                              increment times the scan stride. The sign is ignored.
/// @return Median distance of the points from the origin times |angle_between_points|, in
///         metres. For an even number of points the upper of the two middle distances is used.
///         0 for an empty scan.
double median_point_spacing(
  const Eigen::Matrix<double, Eigen::Dynamic, 2> & local_scans,
  double angle_between_points);

/// Keeps every stride-th row of a scan, starting with row 0.
///
/// @param scans   Scan points, one row (x, y) per point.
/// @param stride  Step between kept rows; must be >= 1 (not checked).
/// @return Rows 0, stride, 2 * stride, ... of `scans`, in order.
Eigen::Matrix<double, Eigen::Dynamic, 2> subsample_scan(
  const Eigen::Matrix<double, Eigen::Dynamic, 2> & scans, int stride);

/// Converts metric points to sample coordinates, in cells: `s = x * inv_res + sample_origin`.
///
/// The same offset is applied to both columns. Cell i's value sits at s = i.
///
/// @param points_meters  Points in the map frame, one row (x, y), in metres.
/// @param inv_res        Cells per metre.
/// @param sample_origin  Offset in cells, as in `GridView::sample_origin`.
/// @return One row (sx, sy) per input row, in cells.
Eigen::Matrix<double, Eigen::Dynamic, 2> to_grid_coords(
  const Eigen::Matrix<double, Eigen::Dynamic, 2> & points_meters,
  double inv_res, double sample_origin);

/// Rotates and translates scan points from the base frame into the map frame.
///
/// @param local_scans  Points in the base frame, one row (x, y), in metres.
/// @param pose         Robot pose (x, y, theta) in the map frame: metres, metres, radians.
/// @return One row (x, y) per input row, in the map frame, in metres.
Eigen::Matrix<double, Eigen::Dynamic, 2> transform_scan_to_world(
  const Eigen::Matrix<double, Eigen::Dynamic, 2> & local_scans,
  const Eigen::Vector3d & pose);

/// Bilinear interpolation of occupancy probability over the grid.
///
/// For each point, blends the four cells whose centres surround it. Each cell
/// value is converted to an occupancy probability in [0, 1] through the
/// view's sigmoid table.
///
/// @param gv                   Grid view.
/// @param grid_points_pixels   Points in sample coordinates (cells), one row (sx, sy), as from
///                             to_grid_coords().
/// @param compute_gradients    Also fill `gradients`.
/// @return Per point: `values` (probability), `mask` (1 when all four surrounding
///         cells are inside the grid, otherwise 0 and value 0), and, only when
///         `compute_gradients` is true, `gradients` (d value / d sx, d value / d sy,
///         per cell). `gradients` is empty when `compute_gradients` is false, and
///         its row is 0 for a point with mask 0.
BilinearResult bilinear_interpolation(
  const GridView & gv,
  const Eigen::Matrix<double, Eigen::Dynamic, 2> & grid_points_pixels,
  bool compute_gradients = false);

/// Joint cost of a pose: scan mismatch against the map plus distance from the odometry pose.
///
/// The scan term is `0.5 * sum(r_k^2)` with `r_k = (1 - p_k) / sigma_hit`, where
/// `p_k` is the interpolated occupancy probability at scan point k. The
/// odometry term is `0.5 * d^T * odom_info * d`, with `d = pose - odom_pose`
/// and the angle difference wrapped to [-pi, pi). Lower is better.
///
/// A point whose four surrounding cells are not all inside the grid is charged
/// as an unknown cell, residual 0.5 / sigma_hit.
///
/// @param gv            Grid view.
/// @param local_scans   Hit points in the base frame, one row (x, y), in metres.
/// @param pose          Candidate pose (x, y, theta) in the map frame.
/// @param odom_pose     Odometry pose (x, y, theta) in the map frame.
/// @param odom_info     Information matrix (inverse covariance) of the odometry pose, 3x3 over
///                      (x, y, theta).
/// @param sigma_hit     Scan noise scale; > 0 (not checked).
/// @param scan_indices  Rows of `local_scans` to use; all rows when empty. Each index must be a
///                      valid row.
/// @return The cost, unitless and >= 0 when `odom_info` is positive semi-definite.
double compute_joint_cost(
  const GridView & gv,
  const Eigen::Matrix<double, Eigen::Dynamic, 2> & local_scans,
  const Eigen::Vector3d & pose,
  const Eigen::Vector3d & odom_pose,
  const Eigen::Matrix3d & odom_info,
  double sigma_hit,
  const std::vector<int> & scan_indices = {});

/// Builds the residuals and Jacobian of the scan term at a pose.
///
/// Only points whose four surrounding cells are inside the grid are used. For
/// each, the residual is `(1 - p) / sigma_hit` and the Jacobian row holds the
/// derivative of the residual with respect to pose x (per metre), y (per
/// metre) and theta (per radian).
///
/// @param gv           Grid view.
/// @param local_scans  Hit points in the base frame, one row (x, y), in metres.
/// @param pose         Pose (x, y, theta) in the map frame.
/// @param sigma_hit    Scan noise scale; > 0 (not checked).
/// @return `success` is false when fewer than 10 points are inside the grid; then
///         `residuals` and `jacobian` are empty. `valid_indices` lists the rows of
///         `local_scans` inside the grid; on success `residuals` and `jacobian`
///         have one row per entry, in that order.
JacobianResult build_scan_jacobian(
  const GridView & gv,
  const Eigen::Matrix<double, Eigen::Dynamic, 2> & local_scans,
  const Eigen::Vector3d & pose,
  double sigma_hit);

/// Sums the interpolated occupancy probability of a scan placed at a candidate position.
///
/// @param gv           Grid view.
/// @param rotated_base Scan points already rotated into the map frame's orientation, one row (x,
///                     y), in metres.
/// @param cand_x       Candidate x translation in the map frame, in metres.
/// @param cand_y       Candidate y translation in the map frame, in metres.
/// @param[out] valid_count  Number of points whose four surrounding cells are inside the grid.
/// @return Sum of the interpolated probabilities over those points; higher is a better fit.
///         Points off the grid add nothing.
double coarse_score(
  const GridView & gv,
  const Eigen::Matrix<double, Eigen::Dynamic, 2> & rotated_base,
  double cand_x, double cand_y,
  int & valid_count);

/// Searches a small window of poses around the odometry pose for the best fit to the map.
///
/// Tries translations of a few cells (multiples of `config.map_resolution`) in
/// x and y and small rotations about the odometry heading, scored with
/// coarse_score() on a subsampled scan. The odometry prior is ignored. The window
/// is wider when `map_quality` is below 0.2. When `map_quality` > 0.01,
/// candidates with fewer than 20% of the subsampled points inside the grid are skipped.
///
/// @param gv            Grid view.
/// @param map_quality   Fraction of occupied confident cells, in [0, 1], from
///                      OccupancyGrid::map_quality().
/// @param local_scans   Hit points in the base frame, one row (x, y), in metres.
/// @param odom_pose     Odometry pose (x, y, theta) in the map frame; the centre of the search.
/// @param config        Supplies `map_resolution`.
/// @return The best-scoring pose; its theta is the odometry theta plus the tried
///         rotation, not wrapped. `odom_pose` when every candidate was skipped.
Eigen::Vector3d coarse_search(
  const GridView & gv,
  double map_quality,
  const Eigen::Matrix<double, Eigen::Dynamic, 2> & local_scans,
  const Eigen::Vector3d & odom_pose,
  const SlamConfig & config);

/// Levenberg-Marquardt refinement of scan-to-map alignment, with the odometry pose as a prior.
///
/// Minimises the joint cost of compute_joint_cost(). The returned Hessian is
/// J^T J + odom_info evaluated at the returned pose; it is odom_info alone
/// when fewer than 10 scan points are valid there.
/// success is false only when fewer than 10 scan points fall inside the grid
/// at the pose being linearised. The returned pose is the last accepted pose;
/// each accepted step lowers the joint cost over the scan points valid at that
/// iteration.
///
/// @param gv            Grid view.
/// @param local_scans   Hit points in the base frame, one row (x, y), in metres.
/// @param curr_pose     Starting pose (x, y, theta) in the map frame.
/// @param odom_pose     Odometry pose (x, y, theta) in the map frame, the prior's mean.
/// @param odom_info     Information matrix (inverse covariance) of the odometry pose, 3x3 over
///                      (x, y, theta).
/// @param config        Supplies `sigma_hit` and the `lm_*` settings.
/// @return The refined pose, the Hessian and `success`.
LMResult lm_optimize(
  const GridView & gv,
  const Eigen::Matrix<double, Eigen::Dynamic, 2> & local_scans,
  Eigen::Vector3d curr_pose,
  const Eigen::Vector3d & odom_pose,
  const Eigen::Matrix3d & odom_info,
  const SlamConfig & config);

/// Log importance weight of a pose from how well the scan fits the map.
///
/// Sums the logarithm of the interpolated occupancy probability over every
/// `config.score_stride`-th scan point (each probability limited to [1e-9, 1];
/// a point with a surrounding cell off the grid counts as 1e-9), then scales
/// the sum by `1 / (obs_sigma_gain * num_particles)` so that the weights of
/// different particles stay close together.
///
/// @param gv          Grid view.
/// @param final_pose  Pose (x, y, theta) in the map frame.
/// @param local_scans Hit points in the base frame, one row (x, y), in metres.
/// @param config      Supplies `score_stride`, `obs_sigma_gain` and `num_particles`.
/// @return Natural-log importance weight, scaled by 1 / (obs_sigma_gain * num_particles),
///         <= 0; larger is a better fit.
double calculate_log_weight(
  const GridView & gv,
  const Eigen::Vector3d & final_pose,
  const Eigen::Matrix<double, Eigen::Dynamic, 2> & local_scans,
  const SlamConfig & config);

/// Matches one scan against one particle's map: coarse search, refinement and weight.
///
/// The optimiser starts from the coarse-search pose or the odometry pose,
/// whichever has the lower joint cost; the fallback (odometry pose) is used
/// only when too few scan points fall inside the grid.
///
/// @param grid         The particle's map.
/// @param local_scans  Hit points in the base frame, one row (x, y), in metres.
/// @param odom_pose    Predicted pose (x, y, theta) in the map frame.
/// @param odom_cov     Covariance of the predicted pose, 3x3 over (x, y, theta); its
///                     diagonal entries are raised to at least 0.01 before use.
/// @param map_quality  Fraction of occupied confident cells of `grid`, in [0, 1]; selects the
///                     coarse search window.
/// @param config       Matching and weighting settings.
/// @param sigmoid      Table that converts stored cell values to occupancy probabilities.
/// @return `pose`: the matched pose (the odometry pose when too few scan points
///         fall inside the grid); `proposal_cov`: covariance for sampling a
///         pose around it, 3x3 over (x, y, theta); `log_weight`: as from
///         calculate_log_weight().
ScanMatchResult scan_match_single(
  const OccupancyGrid & grid,
  const Eigen::Matrix<double, Eigen::Dynamic, 2> & local_scans,
  const Eigen::Vector3d & odom_pose,
  const Eigen::Matrix3d & odom_cov,
  double map_quality,
  const SlamConfig & config,
  const SigmoidLUT & sigmoid);

}  // namespace slam

#endif  // SLAM_CPP__SCAN_MATCHING_HPP_
