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
#include <cmath>
#include <limits>
#include <vector>
#include <random>
#include <stdexcept>
#include "slam_cpp/slam_config.hpp"
#include "slam_cpp/scan_matching.hpp"
#include "slam_cpp/occupancy_grid.hpp"
#include "slam_cpp/sigmoid_lut.hpp"
#include "support/room.hpp"
#include "support/room_match.hpp"

// ============================================================================
// Test fixture: provides a SlamConfig, SigmoidLUT, and helper to build
// a pre-populated grid for scan matching tests.
// ============================================================================
class ScanMatchingTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    config.map_size = 10.0;       // 10m x 10m (smaller for faster tests)
    config.map_resolution = 0.1;  // 100 x 100 grid
    sigmoid = std::make_unique<slam::SigmoidLUT>(config.log_odds_scale);
  }

  // Helper: create a grid with a wall of occupied cells.
  // Places a horizontal wall at world y=2.0 (grid row 70), from x=-3 to x=3.
  slam::OccupancyGrid make_grid_with_wall()
  {
    slam::OccupancyGrid grid(config.map_size, config.map_size,
      config.map_resolution, config);
    // Manually update cells along the wall to be highly occupied.
    // We use a scan from a pose directly below the wall.
    Eigen::Vector3d pose(0.0, 0.0, 0.0);
    Eigen::Matrix<double, Eigen::Dynamic, 2> scans(7, 2);
    for (int i = 0; i < 7; ++i) {
      scans(i, 0) = -3.0 + i * 1.0;  // x from -3 to 3
      scans(i, 1) = 2.0;              // all at y=2
    }
    // Hit the wall multiple times to build strong occupancy.
    for (int rep = 0; rep < 10; ++rep) {
      grid.update(pose, scans);
    }
    return grid;
  }

  slam::SlamConfig config;
  std::unique_ptr<slam::SigmoidLUT> sigmoid;
};

// --------------------------------------------------------------------------
// Test 1: extract_local_scan converts polar to Cartesian correctly and drops
// NaN and below-range_min beams.
// --------------------------------------------------------------------------
TEST_F(ScanMatchingTest, ExtractLocalScanPolarToCartesian)
{
  // 5 beams from -pi/2 to +pi/2 (front semicircle), stride=1.
  int n = 5;
  float angle_min = -static_cast<float>(M_PI) / 2.0f;
  float angle_increment = static_cast<float>(M_PI) / 4.0f;
  float range_min = 0.15f;
  float range_max = 10.0f;

  // Ranges: [1.0, NaN, 2.0, 0.0, 5.0]
  // Expected valid: beam 0 (r=1.0), beam 2 (r=2.0), beam 4 (r=5.0)
  // beam 1 (NaN) and beam 3 (r=0.0 < range_min) should be filtered.
  std::vector<float> ranges = {1.0f, std::nanf(""), 2.0f, 0.0f, 5.0f};

  auto result = slam::extract_local_scan(
    ranges.data(), n, angle_min, angle_increment, range_min, range_max, /*stride=*/ 1);

  EXPECT_EQ(result.rows(), 3);  // only 3 valid beams

  // Beam 0: angle = -pi/2, range = 1.0 -> (0, -1)
  EXPECT_NEAR(result(0, 0), 0.0, 1e-6);   // x ~ 0
  EXPECT_NEAR(result(0, 1), -1.0, 1e-6);  // y ~ -1
}

// --------------------------------------------------------------------------
// Test 2: extract_local_scan respects stride parameter.
// --------------------------------------------------------------------------
TEST_F(ScanMatchingTest, ExtractLocalScanStride)
{
  int n = 9;
  float angle_min = 0.0f;
  float angle_increment = static_cast<float>(M_PI) / 8.0f;
  float range_min = 0.15f;
  float range_max = 10.0f;
  std::vector<float> ranges(9, 1.0f);  // all valid, all 1.0m

  auto result = slam::extract_local_scan(
    ranges.data(), n, angle_min, angle_increment, range_min, range_max, /*stride=*/ 3);

  // Beams at indices 0, 3, 6 should be kept (3 beams).
  EXPECT_EQ(result.rows(), 3);
}

TEST_F(ScanMatchingTest, StrideBelowOneIsRejected)
{
  const std::vector<float> ranges(4, 1.0f);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(
    slam::extract_local_scan(ranges.data(), 4, 0.0f, 0.1f, 0.15f, 10.0f, 1).rows(), 4);
  for (const int bad : {0, -1}) {
    EXPECT_THROW(
      slam::extract_local_scan(ranges.data(), 4, 0.0f, 0.1f, 0.15f, 10.0f, bad),
      std::invalid_argument);
  }
  // The stride check comes before the non-finite-angle early return.
  EXPECT_THROW(
    slam::extract_local_scan(ranges.data(), 4, nan, 0.1f, 0.15f, 10.0f, 0),
    std::invalid_argument);

  Eigen::Matrix<double, Eigen::Dynamic, 2> scans = Eigen::Matrix<double, 4, 2>::Ones();
  EXPECT_EQ(slam::subsample_scan(scans, 1).rows(), 4);
  EXPECT_EQ(slam::subsample_scan(scans, 3).rows(), 2);
  EXPECT_THROW(slam::subsample_scan(scans, 0), std::invalid_argument);
  EXPECT_THROW(slam::subsample_scan(scans, -1), std::invalid_argument);
}

TEST_F(ScanMatchingTest, MakeGridViewChecksSquareAndResolution)
{
  const slam::OccupancyGrid grid(config.map_size, config.map_size, config.map_resolution, config);
  EXPECT_NO_THROW(slam::make_grid_view(grid, config, *sigmoid));

  // 100 columns by 50 rows.
  const slam::OccupancyGrid wide(10.0, 5.0, 0.1, config);
  EXPECT_THROW(slam::make_grid_view(wide, config, *sigmoid), std::invalid_argument);

  // Tolerance is a relative 1e-9: 1e-10 off is accepted, 1e-8 off is rejected.
  slam::SlamConfig near = config;
  near.map_resolution = 0.1 * (1.0 + 1e-10);
  EXPECT_NO_THROW(slam::make_grid_view(grid, near, *sigmoid));
  slam::SlamConfig far = config;
  far.map_resolution = 0.1 * (1.0 + 1e-8);
  EXPECT_THROW(slam::make_grid_view(grid, far, *sigmoid), std::invalid_argument);
}

// Beam classes: inf, NaN, -inf, below range_min and the range_max beam are
// dropped; range_min is kept; just below range_max is kept.
TEST_F(ScanMatchingTest, ExtractLocalScanBeamClasses)
{
  const float inf = std::numeric_limits<float>::infinity();
  std::vector<float> ranges = {inf, 1.0f, 10.0f, std::nanf(""), 0.05f, -inf, 0.15f, 9.99f};
  const float inc = static_cast<float>(M_PI) / 2.0f;

  auto r = slam::extract_local_scan(
    ranges.data(), static_cast<int>(ranges.size()), 0.0f, inc, 0.15f, 10.0f, 1);

  ASSERT_EQ(r.rows(), 3);
  EXPECT_NEAR(r(0, 0), 0.0, 1e-5);    EXPECT_NEAR(r(0, 1), 1.0, 1e-5);
  EXPECT_NEAR(r(1, 0), -0.15, 1e-5);  EXPECT_NEAR(r(1, 1), 0.0, 1e-5);
  EXPECT_NEAR(r(2, 0), 0.0, 1e-5);    EXPECT_NEAR(r(2, 1), -9.99, 1e-5);
}

// A zero range is dropped even when range_min is 0; so is negative zero.
TEST_F(ScanMatchingTest, ExtractLocalScanZeroRangeDropped)
{
  std::vector<float> ranges = {0.0f, 1.0f, -0.0f};
  auto r = slam::extract_local_scan(ranges.data(), 3, 0.0f, 0.1f, 0.0f, 10.0f, 1);

  ASSERT_EQ(r.rows(), 1);
  EXPECT_NEAR(r(0, 0), 0.995004, 1e-5);
  EXPECT_NEAR(r(0, 1), 0.099833, 1e-5);
}

// The bearing step is angle_increment, not derived from the end angles.
TEST_F(ScanMatchingTest, ExtractLocalScanAngleFromIncrement)
{
  std::vector<float> ranges(4, 1.0f);
  auto r = slam::extract_local_scan(ranges.data(), 4, 0.0f, 0.1f, 0.15f, 10.0f, 1);

  ASSERT_EQ(r.rows(), 4);
  EXPECT_NEAR(r(3, 0), 0.955336, 1e-5);
  EXPECT_NEAR(r(3, 1), 0.295520, 1e-5);
}

// Stride 2 over 5 beams uses beams 0, 2, 4.
TEST_F(ScanMatchingTest, ExtractLocalScanStrideSelectsBeams)
{
  std::vector<float> ranges(5, 1.0f);
  const float inc = static_cast<float>(M_PI) / 2.0f;
  auto r = slam::extract_local_scan(ranges.data(), 5, 0.0f, inc, 0.15f, 10.0f, 2);

  ASSERT_EQ(r.rows(), 3);
  EXPECT_NEAR(r(0, 0), 1.0, 1e-5);   EXPECT_NEAR(r(0, 1), 0.0, 1e-5);   // beam 0
  EXPECT_NEAR(r(1, 0), -1.0, 1e-5);  EXPECT_NEAR(r(1, 1), 0.0, 1e-5);   // beam 2
  EXPECT_NEAR(r(2, 0), 1.0, 1e-5);   EXPECT_NEAR(r(2, 1), 0.0, 1e-5);   // beam 4
}

// A negative increment makes the bearing decrease with the index.
TEST_F(ScanMatchingTest, ExtractLocalScanNegativeIncrement)
{
  std::vector<float> ranges = {1.0f, 1.0f};
  auto r = slam::extract_local_scan(
    ranges.data(), 2, 0.0f, -static_cast<float>(M_PI) / 2.0f, 0.15f, 10.0f, 1);

  ASSERT_EQ(r.rows(), 2);
  EXPECT_NEAR(r(0, 0), 1.0, 1e-5);  EXPECT_NEAR(r(0, 1), 0.0, 1e-5);
  EXPECT_NEAR(r(1, 0), 0.0, 1e-5);  EXPECT_NEAR(r(1, 1), -1.0, 1e-5);
}

// A non-finite angle_min gives no rows.
TEST_F(ScanMatchingTest, ExtractLocalScanNonFiniteAngleGivesNoRows)
{
  std::vector<float> ranges(3, 1.0f);
  auto r = slam::extract_local_scan(
    ranges.data(), 3, std::nanf(""), 0.1f, 0.15f, 10.0f, 1);
  EXPECT_EQ(r.rows(), 0);
}

// With range_max <= range_min no range qualifies.
TEST_F(ScanMatchingTest, ExtractLocalScanEmptyRangeWindowGivesNoRows)
{
  std::vector<float> ranges = {5.0f};
  auto r = slam::extract_local_scan(ranges.data(), 1, 0.0f, 0.1f, 5.0f, 5.0f, 1);
  EXPECT_EQ(r.rows(), 0);
}

// --------------------------------------------------------------------------
// Test 3: to_grid_coords scales and offsets correctly.
// --------------------------------------------------------------------------
TEST_F(ScanMatchingTest, ToGridCoordsScaling)
{
  Eigen::Matrix<double, Eigen::Dynamic, 2> pts(2, 2);
  pts << 1.0, 2.0,
    -0.5, 0.0;

  double inv_res = 10.0;  // 1/0.1
  double origin = 50.0;   // explicit offset in cells (arbitrary here)

  auto grid_pts = slam::to_grid_coords(pts, inv_res, origin);

  // (1.0, 2.0) → (1.0*10 + 50, 2.0*10 + 50) = (60, 70)
  EXPECT_NEAR(grid_pts(0, 0), 60.0, 1e-10);
  EXPECT_NEAR(grid_pts(0, 1), 70.0, 1e-10);

  // (-0.5, 0.0) → (-0.5*10 + 50, 0.0*10 + 50) = (45, 50)
  EXPECT_NEAR(grid_pts(1, 0), 45.0, 1e-10);
  EXPECT_NEAR(grid_pts(1, 1), 50.0, 1e-10);
}

// --------------------------------------------------------------------------
// Test 4: transform_scan_to_world applies rotation and translation.
// --------------------------------------------------------------------------
TEST_F(ScanMatchingTest, TransformScanToWorld)
{
  Eigen::Matrix<double, Eigen::Dynamic, 2> local(1, 2);
  local << 1.0, 0.0;  // 1m in front of robot

  // Pose at origin, facing +x (theta=0).
  Eigen::Vector3d pose_fwd(0.0, 0.0, 0.0);
  auto world_fwd = slam::transform_scan_to_world(local, pose_fwd);
  EXPECT_NEAR(world_fwd(0, 0), 1.0, 1e-10);
  EXPECT_NEAR(world_fwd(0, 1), 0.0, 1e-10);

  // Pose at origin, facing +y (theta=pi/2).
  Eigen::Vector3d pose_left(0.0, 0.0, M_PI / 2.0);
  auto world_left = slam::transform_scan_to_world(local, pose_left);
  EXPECT_NEAR(world_left(0, 0), 0.0, 1e-10);
  EXPECT_NEAR(world_left(0, 1), 1.0, 1e-10);

  // Pose at (5, 3), facing +x.
  Eigen::Vector3d pose_offset(5.0, 3.0, 0.0);
  auto world_offset = slam::transform_scan_to_world(local, pose_offset);
  EXPECT_NEAR(world_offset(0, 0), 6.0, 1e-10);
  EXPECT_NEAR(world_offset(0, 1), 3.0, 1e-10);
}

// --------------------------------------------------------------------------
// Test 5: bilinear_interpolation returns known values at grid cell centers.
// --------------------------------------------------------------------------
TEST_F(ScanMatchingTest, BilinearInterpolationKnownValues)
{
  // Create a small grid and populate one cell with a known value.
  slam::OccupancyGrid grid(config.map_size, config.map_size,
    config.map_resolution, config);

  // Populate a cell by raycasting. After one hit, the hit cell has lo_occupied.
  Eigen::Vector3d pose(0.0, 0.0, 0.0);
  Eigen::Matrix<double, Eigen::Dynamic, 2> scans(1, 2);
  scans << 1.0, 0.0;
  grid.update(pose, scans);

  // Build GridView for querying.
  slam::GridView gv = slam::make_grid_view(grid, config, *sigmoid);

  // Query at the hit cell center: world (1.0, 0) → pixel (60, 50).
  // The cell at (row=50, col=60) should have lo_occupied value.
  Eigen::Matrix<double, Eigen::Dynamic, 2> query_pts(1, 2);
  query_pts << 60.0, 50.0;  // pixel coordinates

  auto result = slam::bilinear_interpolation(gv, query_pts, false);

  // The interpolated value should be the sigmoid of the cell's log-odds.
  // After one hit, the cell has lo_occupied_scaled = 17.
  // sigmoid(17) = 1/(1+exp(-17/20)) ≈ 0.701
  double expected = (*sigmoid)(config.lo_occupied_scaled());
  EXPECT_NEAR(result.values(0), expected, 0.05);
  EXPECT_EQ(result.mask(0), 1);  // valid (in-bounds)
}

// --------------------------------------------------------------------------
// Test 6: bilinear gradients match finite-difference approximation.
// --------------------------------------------------------------------------
TEST_F(ScanMatchingTest, BilinearGradientsMatchFiniteDifferences)
{
  // Build a grid with a wall so there are spatial gradients to measure.
  auto grid = make_grid_with_wall();
  slam::GridView gv = slam::make_grid_view(grid, config, *sigmoid);

  // Query a point near the wall edge where gradients should be non-zero.
  // Wall is at y=2.0 → pixel row = 70. Query just below: pixel (50, 69.5).
  Eigen::Matrix<double, Eigen::Dynamic, 2> pt(1, 2);
  pt << 50.0, 69.5;

  auto result = slam::bilinear_interpolation(gv, pt, /*compute_gradients=*/ true);

  // Skip if point is out of bounds.
  if (result.mask(0) == 0) {
    GTEST_SKIP() << "Query point out of bounds";
  }

  // Finite difference approximation: f(x+h) - f(x-h)) / (2h)
  double h = 0.01;
  Eigen::Matrix<double, Eigen::Dynamic, 2> pt_xp(1, 2), pt_xm(1, 2);
  Eigen::Matrix<double, Eigen::Dynamic, 2> pt_yp(1, 2), pt_ym(1, 2);
  pt_xp << 50.0 + h, 69.5;
  pt_xm << 50.0 - h, 69.5;
  pt_yp << 50.0, 69.5 + h;
  pt_ym << 50.0, 69.5 - h;

  auto r_xp = slam::bilinear_interpolation(gv, pt_xp, false);
  auto r_xm = slam::bilinear_interpolation(gv, pt_xm, false);
  auto r_yp = slam::bilinear_interpolation(gv, pt_yp, false);
  auto r_ym = slam::bilinear_interpolation(gv, pt_ym, false);

  if (r_xp.mask(0) && r_xm.mask(0) && r_yp.mask(0) && r_ym.mask(0)) {
    double fd_gx = (r_xp.values(0) - r_xm.values(0)) / (2.0 * h);
    double fd_gy = (r_yp.values(0) - r_ym.values(0)) / (2.0 * h);

    EXPECT_NEAR(result.gradients(0, 0), fd_gx, 0.1);
    EXPECT_NEAR(result.gradients(0, 1), fd_gy, 0.1);
  }
}

// --------------------------------------------------------------------------
// Test 7: coarse_search finds a pose near the true pose on a known map.
// --------------------------------------------------------------------------
TEST_F(ScanMatchingTest, CoarseSearchFindsCorrectPose)
{
  // Build a grid with a known wall.
  auto grid = make_grid_with_wall();
  slam::GridView gv = slam::make_grid_view(grid, config, *sigmoid);

  // The scan that matches the wall from pose (0, 0, 0).
  Eigen::Matrix<double, Eigen::Dynamic, 2> scans(7, 2);
  for (int i = 0; i < 7; ++i) {
    scans(i, 0) = -3.0 + i * 1.0;
    scans(i, 1) = 2.0;
  }

  // Start from a slightly offset odometry pose.
  Eigen::Vector3d odom_pose(0.2, 0.1, 0.05);

  Eigen::Vector3d result = slam::coarse_search(
    gv, grid.map_quality(), scans, odom_pose, config);

  // Coarse search should find a pose reasonably close to origin.
  EXPECT_NEAR(result(0), 0.0, 0.5);  // x within 0.5m
  EXPECT_NEAR(result(1), 0.0, 0.5);  // y within 0.5m
}

// --------------------------------------------------------------------------
// Test 8: compute_joint_cost is lower at the true pose than at an offset.
// --------------------------------------------------------------------------
TEST_F(ScanMatchingTest, ComputeJointCostLowerAtTruePose)
{
  auto grid = make_grid_with_wall();
  slam::GridView gv = slam::make_grid_view(grid, config, *sigmoid);

  Eigen::Matrix<double, Eigen::Dynamic, 2> scans(7, 2);
  for (int i = 0; i < 7; ++i) {
    scans(i, 0) = -3.0 + i * 1.0;
    scans(i, 1) = 2.0;
  }

  Eigen::Vector3d true_pose(0.0, 0.0, 0.0);
  Eigen::Vector3d offset_pose(1.0, 0.5, 0.3);
  Eigen::Vector3d odom_pose(0.0, 0.0, 0.0);
  Eigen::Matrix3d odom_info = Eigen::Matrix3d::Identity();

  double cost_true = slam::compute_joint_cost(
    gv, scans, true_pose, odom_pose, odom_info, config.sigma_hit);
  double cost_offset = slam::compute_joint_cost(
    gv, scans, offset_pose, odom_pose, odom_info, config.sigma_hit);

  EXPECT_LT(cost_true, cost_offset);
}

// --------------------------------------------------------------------------
// Test 9: from world metres, a point at the centre of a hit cell reads that
// cell's value (sample coordinates put cell i's value at the cell centre).
// --------------------------------------------------------------------------
TEST_F(ScanMatchingTest, InterpolationAtCellCentreFromWorldReturnsCellValue)
{
  slam::OccupancyGrid grid(config.map_size, config.map_size,
    config.map_resolution, config);
  Eigen::Matrix<double, Eigen::Dynamic, 2> hit(1, 2);
  hit << 1.0, 0.0;  // lands in cell (row 50, col 60)
  grid.update(Eigen::Vector3d(0.0, 0.0, 0.0), hit);
  slam::GridView gv = slam::make_grid_view(grid, config, *sigmoid);

  // Centre of cell (50, 60) is world (1.05, 0.05).
  Eigen::Matrix<double, Eigen::Dynamic, 2> world(1, 2);
  world << 1.05, 0.05;
  auto pix = slam::to_grid_coords(world, gv.inv_res, gv.sample_origin);
  auto result = slam::bilinear_interpolation(gv, pix);

  EXPECT_EQ(result.mask(0), 1);
  EXPECT_NEAR(result.values(0), (*sigmoid)(grid.at(50, 60)), 1e-12);
  EXPECT_NEAR(result.values(0), (*sigmoid)(config.lo_occupied_scaled()), 1e-12);
}

// --------------------------------------------------------------------------
// Test 10: on the boundary between two cells in x, centred in y, the value
// is the mean of the two cells.
// --------------------------------------------------------------------------
TEST_F(ScanMatchingTest, InterpolationOnCellBoundaryFromWorldIsMeanOfCells)
{
  slam::OccupancyGrid grid(config.map_size, config.map_size,
    config.map_resolution, config);
  Eigen::Matrix<double, Eigen::Dynamic, 2> hit(1, 2);
  hit << 1.0, 0.0;  // cell (row 50, col 60)
  grid.update(Eigen::Vector3d(0.0, 0.0, 0.0), hit);
  slam::GridView gv = slam::make_grid_view(grid, config, *sigmoid);

  // Boundary between cols 60 and 61 is world x = 1.1; row 50 centre is y = 0.05.
  Eigen::Matrix<double, Eigen::Dynamic, 2> world(1, 2);
  world << 1.10, 0.05;
  auto pix = slam::to_grid_coords(world, gv.inv_res, gv.sample_origin);
  auto result = slam::bilinear_interpolation(gv, pix);

  double expected = 0.5 * ((*sigmoid)(grid.at(50, 60)) + (*sigmoid)(grid.at(50, 61)));
  EXPECT_EQ(result.mask(0), 1);
  EXPECT_NEAR(result.values(0), expected, 1e-12);
  // Guard against a vacuous test: the two cells must differ.
  EXPECT_NE(grid.at(50, 60), grid.at(50, 61));
}

// --------------------------------------------------------------------------
// Test 11: lm_optimize returns H = J^T J + odom_info evaluated at the pose it
// returns, not odom_info and not the Hessian of an earlier iterate.
// --------------------------------------------------------------------------
TEST_F(ScanMatchingTest, LmHessianIsEvaluatedAtReturnedPose)
{
  config.map_size = 20.0;
  slam::OccupancyGrid grid(config.map_size, config.map_size,
    config.map_resolution, config);
  std::mt19937 rng(7);
  Eigen::Vector3d truth(0.0, 0.0, 0.0);
  auto scan = slam_test::scan_from(truth, 4.05, 3.05, 0.0, rng);
  for (int rep = 0; rep < 20; ++rep) {
    grid.update(truth, scan);
  }
  slam::GridView gv = slam::make_grid_view(grid, config, *sigmoid);

  Eigen::Matrix3d odom_info = Eigen::Matrix3d::Identity() * 100.0;
  Eigen::Vector3d start(0.03, -0.02, 0.01);
  auto lm = slam::lm_optimize(gv, scan, start, start, odom_info, config);

  auto jac = slam::build_scan_jacobian(gv, scan, lm.pose, config.sigma_hit);
  ASSERT_TRUE(jac.success);
  Eigen::Matrix3d H = jac.jacobian.transpose() * jac.jacobian + odom_info;

  EXPECT_LE((lm.hessian - H).norm(), 1e-9 * H.norm());
  // Scan information must be present: far from odom_info alone.
  EXPECT_GT((lm.hessian - odom_info).norm(), 0.1 * odom_info.norm());
}

// --------------------------------------------------------------------------
// Test 12: started at the true pose on a map built from that pose, the
// optimiser succeeds and stays at the pose, for several wall positions.
// --------------------------------------------------------------------------
TEST_F(ScanMatchingTest, LmFromTruePoseOnOwnMapSucceeds)
{
  config.map_size = 20.0;
  for (double d : {0.0, 0.025, 0.05, 0.075}) {
    SCOPED_TRACE(d);
    slam::OccupancyGrid grid(config.map_size, config.map_size,
      config.map_resolution, config);
    std::mt19937 rng(7);
    Eigen::Vector3d truth(0.0, 0.0, 0.0);
    auto scan = slam_test::scan_from(truth, 4.0 + d, 3.0 + d, 0.0, rng);
    for (int rep = 0; rep < 20; ++rep) {
      grid.update(truth, scan);
    }
    slam::GridView gv = slam::make_grid_view(grid, config, *sigmoid);

    Eigen::Matrix3d odom_info = Eigen::Matrix3d::Identity() * 100.0;
    auto lm = slam::lm_optimize(gv, scan, truth, truth, odom_info, config);

    EXPECT_TRUE(lm.success);
    EXPECT_LE(std::abs(lm.pose(0)), 0.055);
    EXPECT_LE(std::abs(lm.pose(1)), 0.055);
    EXPECT_LE(std::abs(lm.pose(2)), 0.002);
  }
}

// --------------------------------------------------------------------------
// Test 13: the coarse search ignores the odometry prior and can jump to a
// pose that costs more than the odometry pose (0.15 m and 0.30 m from truth in
// these two cases). The result must still cost no more than the start and
// stay near the truth.
// --------------------------------------------------------------------------
TEST_F(ScanMatchingTest, MatchNotWorseThanOdometryWhenCoarseSearchJumps)
{
  struct Case { Eigen::Vector3d truth, start; };
  const Case cases[] = {
    {{-0.49, -0.213, -3.041}, {-0.54, -0.206, -3.041}},
    {{-0.016, -0.747, -0.458}, {-0.014, -0.792, -0.488}},
  };
  for (const Case & c : cases) {
    SCOPED_TRACE(::testing::Message() << "truth (" << c.truth.transpose() << ")");
    const auto out = slam_test::match_from(c.truth, c.start);
    EXPECT_LE(out.cost_result, out.cost_start + 1e-9);
    EXPECT_LT((out.pose.head<2>() - c.truth.head<2>()).norm(), 0.03);
  }
}

// --------------------------------------------------------------------------
// Test 14: compute_joint_cost charges a point whose four surrounding cells are
// not all inside the grid as an unknown cell (residual 0.5 / sigma_hit). Empty
// 20 m grid: every point, inside or not, costs 0.5 * (0.5 / 0.05)^2 = 50.
// --------------------------------------------------------------------------
TEST_F(ScanMatchingTest, JointCostChargesOffGridPointAsUnknownCell)
{
  config.map_size = 20.0;
  slam::OccupancyGrid grid(config.map_size, config.map_size,
    config.map_resolution, config);
  slam::GridView gv = slam::make_grid_view(grid, config, *sigmoid);

  // A stored 0 reads exactly 0.5.
  ASSERT_EQ((*sigmoid)(0), 0.5);

  Eigen::Matrix<double, Eigen::Dynamic, 2> pts(4, 2);
  pts << 1.0, 0.0,
    0.0, 1.0,
    -1.0, 0.0,
    100.0, 0.0;     // far outside the grid
  const Eigen::Matrix3d odom_info = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d zero = Eigen::Vector3d::Zero();

  // All four points.
  EXPECT_NEAR(
    slam::compute_joint_cost(gv, pts, zero, zero, odom_info, config.sigma_hit),
    200.0, 1e-9);
  // Subset: one inside, one outside.
  EXPECT_NEAR(
    slam::compute_joint_cost(gv, pts, zero, zero, odom_info, config.sigma_hit, {0, 3}),
    100.0, 1e-9);
  // Odometry term 0.5 * 0.1^2 adds to the scan term.
  EXPECT_NEAR(
    slam::compute_joint_cost(
      gv, pts, Eigen::Vector3d(0.1, 0.0, 0.0), zero, odom_info,
      config.sigma_hit),
    200.005, 1e-9);
}

// median_point_spacing: median range times |angle_between_points|; the upper
// middle for an even count; 0 for an empty scan.
TEST_F(ScanMatchingTest, MedianPointSpacingOddCount)
{
  Eigen::Matrix<double, Eigen::Dynamic, 2> pts(3, 2);
  pts << 1.0, 0.0,
    0.0, 2.0,
    -3.0, 0.0;
  EXPECT_DOUBLE_EQ(slam::median_point_spacing(pts, 0.1), 0.2);
}

TEST_F(ScanMatchingTest, MedianPointSpacingEvenCountUsesUpperMiddle)
{
  Eigen::Matrix<double, Eigen::Dynamic, 2> pts(4, 2);
  pts << 1.0, 0.0,
    0.0, 2.0,
    -3.0, 0.0,
    0.0, -4.0;
  EXPECT_NEAR(slam::median_point_spacing(pts, -0.1), 0.3, 1e-12);
}

TEST_F(ScanMatchingTest, MedianPointSpacingEmptyScanIsZero)
{
  const Eigen::Matrix<double, Eigen::Dynamic, 2> pts(0, 2);
  EXPECT_EQ(slam::median_point_spacing(pts, 0.1), 0.0);
}

TEST_F(ScanMatchingTest, MedianPointSpacingOrderIndependent)
{
  Eigen::Matrix<double, Eigen::Dynamic, 2> a(4, 2), b(4, 2);
  a << 1.0, 0.0,
    0.0, 2.0,
    -3.0, 0.0,
    0.0, -4.0;
  b << 0.0, -4.0,
    1.0, 0.0,
    -3.0, 0.0,
    0.0, 2.0;
  EXPECT_DOUBLE_EQ(slam::median_point_spacing(a, 0.1), slam::median_point_spacing(b, 0.1));
}
