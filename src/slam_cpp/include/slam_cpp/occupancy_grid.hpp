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

#ifndef SLAM_CPP__OCCUPANCY_GRID_HPP_
#define SLAM_CPP__OCCUPANCY_GRID_HPP_

/// @file
/// @brief Occupancy grid that stores log-odds per cell as int8 and integrates laser scans by ray
///        casting.

// Copy-on-write cost: each particle owns a grid (250 KB for 500x500 cells), so
// copying every grid at each resampling step would cost about 80 * 250 KB.
// A copy shares the storage through a std::shared_ptr; the real copy happens in
// ensure_unique() when one of the copies is written.

#include <Eigen/Dense>

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "slam_cpp/slam_config.hpp"

namespace slam
{

/// A cell position in the grid, in whole cells.
struct GridCoord
{
  int x;  ///< Column index; increases with world x.
  int y;  ///< Row index; increases with world y.
};

/// A 2D occupancy grid centred on the world origin.
///
/// Each cell stores log-odds quantised to `int8`: the value is
/// `log-odds * log_odds_scale`, clamped to `[lo_min, lo_max] * log_odds_scale`
/// (from the `SlamConfig` given at construction). Zero means unknown. World
/// coordinate (0, 0) lies at the cell `(cols()/2, rows()/2)`. Storage is
/// row-major; row is the y axis and column is the x axis.
///
/// Copying a grid is cheap: copies share their cells until one of them is
/// written by update().
class OccupancyGrid
{
public:
  /// Creates a grid with every cell unknown (0).
  ///
  /// The grid has `round(height / resolution)` rows and `round(width / resolution)`
  /// columns, for example `OccupancyGrid(50.0, 50.0, 0.1, config)` gives 500 x 500
  /// cells covering 50 m x 50 m.
  ///
  /// @param width       Extent along world x, in metres; > 0.
  /// @param height      Extent along world y, in metres; > 0.
  /// @param resolution  Cell side in metres; > 0.
  /// @param config      Supplies the log-odds scale, increments and limits; checked with
  ///                    validate().
  /// @throws std::invalid_argument if `config` is invalid, if width, height or resolution
  ///         is not positive, or if a side would have fewer than 2 or more than 20000 cells.
  OccupancyGrid(
    double width, double height, double resolution,
    const SlamConfig & config);

  /// Copy shares the cells with `other` until either grid is written.
  ///
  /// @param other  Grid to copy.
  OccupancyGrid(const OccupancyGrid & other) = default;
  /// Copy assignment shares the cells with `other` until either grid is written.
  ///
  /// @param other  Grid to copy.
  /// @return This grid.
  OccupancyGrid & operator=(const OccupancyGrid & other) = default;
  /// Move takes the cells from `other`. Afterwards `other` may only be assigned to or destroyed.
  ///
  /// @param other  Grid to move from.
  OccupancyGrid(OccupancyGrid && other) = default;
  /// Move assignment takes the cells from `other`. Afterwards `other` may only be assigned to or
  /// destroyed.
  ///
  /// @param other  Grid to move from.
  /// @return This grid.
  OccupancyGrid & operator=(OccupancyGrid && other) = default;

  /// Converts a point in the map frame to the cell that contains it.
  ///
  /// A cell covers `[i, i+1)` in cell units, so conversion rounds toward
  /// negative infinity: with 0.1 m cells, x = -0.05 m lies in the cell left of
  /// the origin cell.
  ///
  /// @param x  Map-frame x in metres.
  /// @param y  Map-frame y in metres.
  /// @return The cell (column x, row y); `std::nullopt` if the point lies outside the grid.
  std::optional<GridCoord> world_to_grid(double x, double y) const;

  /// Integrates one laser scan into the grid.
  ///
  /// Every cell on the ray from the robot cell to a beam's hit cell has its
  /// value lowered by the free-space increment; the hit cell has its value
  /// raised by the occupied increment. Values are clamped to the configured
  /// limits. A beam whose hit point lies outside the grid is skipped. If the
  /// robot itself is outside the grid, the grid is left unchanged.
  ///
  /// If this grid shares its cells with another, it takes its own copy first,
  /// so the other grid is not changed.
  ///
  /// @param pose        Robot pose (x, y, theta) in the map frame: metres, metres, radians.
  /// @param local_scans Hit points in the base frame, one row (x, y) per beam, in metres.
  void update(
    const Eigen::Vector3d & pose,
    const Eigen::Matrix<double, Eigen::Dynamic, 2> & local_scans);

  // Call before a parallel section to do all copies sequentially and avoid
  // concurrent shared_ptr operations.
  /// Gives this grid its own cells if it shares them with another grid.
  ///
  /// Does nothing when the cells are not shared. update() calls this itself.
  void ensure_unique();

  /// True if at least one other grid shares this grid's cells.
  ///
  /// @return `true` if the cells are shared, `false` if this grid owns them alone.
  bool is_shared() const;

  /// Returns the stored value of one cell: log-odds times the log-odds scale; 0 is unknown.
  ///
  /// @param row  Row index (y axis), in `[0, rows())`; not checked.
  /// @param col  Column index (x axis), in `[0, cols())`; not checked.
  /// @return Stored value: log-odds times the log-odds scale, in `[lo_min, lo_max] *
  ///         log_odds_scale`.
  int8_t at(int row, int col) const;

  /// Returns a pointer to the cell values, row-major: `data()[row * cols() + col]`.
  ///
  /// The pointer is valid until this grid is written (update() or ensure_unique())
  /// or destroyed.
  ///
  /// @return Pointer to `rows() * cols()` stored cell values.
  const int8_t * data() const;

  /// Number of rows (cells along y).
  /// @return Rows, in cells.
  int rows() const {return rows_;}
  /// Number of columns (cells along x).
  /// @return Columns, in cells.
  int cols() const {return cols_;}
  /// Column of the cell that contains world x = 0.
  /// @return Column index, in cells.
  int origin_x() const {return origin_x_;}
  /// Row of the cell that contains world y = 0.
  /// @return Row index, in cells.
  int origin_y() const {return origin_y_;}
  /// Cell side in metres.
  /// @return Cell side, in metres.
  double resolution() const {return resolution_;}

  /// Fraction of confident cells that are occupied.
  ///
  /// A cell is confident when its log-odds are beyond +1 (occupied) or -1
  /// (free), that is, when its stored value is strictly beyond `+-round(log_odds_scale)`.
  ///
  /// @return Occupied confident cells divided by all confident cells, in
  ///         `[0, 1]`; 0 when no cell is confident yet.
  double map_quality() const;

private:
  // Grid storage, shared between copies. Written only after ensure_unique().
  struct GridData
  {
    std::vector<int8_t> cells;  // row-major, size = rows * cols (250,000 for 500x500)
    int n_occupied = 0;         // incremental: cells above +quality_threshold
    int n_free = 0;             // incremental: cells below -quality_threshold
  };

  std::shared_ptr<GridData> data_;

  // Grid geometry (set once in constructor, never changes)
  int rows_, cols_;
  double resolution_;
  double width_, height_;
  int origin_x_, origin_y_;  // pixel coordinates of world origin (0,0)

  // Cached log-odds parameters (derived from SlamConfig at construction time).
  // Cached here to avoid re-computing round(float * scale) on every cell update.
  int8_t lo_occupied_;        // e.g., 17 = round(0.85 * 20)
  int8_t lo_free_;            // e.g., -8 = round(-0.4 * 20)
  int8_t lo_max_;             // e.g., 100 = round(5.0 * 20)
  int8_t lo_min_;             // e.g., -100 = round(-5.0 * 20)
  int8_t quality_threshold_;  // e.g., 20 = round(1.0 * 20) for map_quality counters

  // Traces one ray from (x0,y0) to (x1,y1) in cell coordinates with Bresenham's
  // line algorithm in integer arithmetic. Cells on the ray get lo_free_, the
  // end cell gets lo_occupied_. Every cell is bounds-checked, including the end.
  void bresenham_raycast(int x0, int y0, int x1, int y1);

  // The only function that writes to data_->cells. Adds delta, clamps to
  // [lo_min_, lo_max_], and keeps n_occupied / n_free current when the cell
  // crosses +-quality_threshold_ (e.g. 19 -> 25 increments n_occupied).
  void update_cell(int row, int col, int8_t delta);
};

}  // namespace slam

#endif  // SLAM_CPP__OCCUPANCY_GRID_HPP_
