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
#ifdef SLAM_HAS_OPENMP
#include <omp.h>
#endif
#include <limits>
#include <random>
#include <stdexcept>
#include <set>
#include <tuple>
#include "slam_cpp/slam_config.hpp"
#include "slam_cpp/particle_filter.hpp"
#include "support/room.hpp"

// ============================================================================
// Test fixture: creates a small particle filter (10 particles, fixed seed)
// for fast, deterministic tests.
// ============================================================================
class ParticleFilterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    config.num_particles = 10;
    config.map_size = 10.0;       // small grid for speed
    config.map_resolution = 0.1;
    pf = std::make_unique<slam::ParticleFilter>(config, /*seed=*/ 42);
  }

  slam::SlamConfig config;
  std::unique_ptr<slam::ParticleFilter> pf;
};

// --------------------------------------------------------------------------
// Test 1: Construction initializes uniform weights, zero poses, correct count.
// --------------------------------------------------------------------------
TEST_F(ParticleFilterTest, ConstructionInitializesCorrectly)
{
  EXPECT_EQ(pf->weights().size(), 10);
  EXPECT_NEAR(pf->weights().sum(), 1.0, 1e-12);

  // Each weight should be 1/N = 0.1.
  for (int i = 0; i < 10; ++i) {
    EXPECT_NEAR(pf->weights()(i), 0.1, 1e-12);
  }

  // All poses should be zero.
  const auto & poses = pf->poses();
  EXPECT_EQ(poses.cols(), 10);
  for (int i = 0; i < 10; ++i) {
    EXPECT_DOUBLE_EQ(poses(0, i), 0.0);
    EXPECT_DOUBLE_EQ(poses(1, i), 0.0);
    EXPECT_DOUBLE_EQ(poses(2, i), 0.0);
  }

  // Correct number of grids.
  EXPECT_EQ(pf->grids().size(), 10u);
}

// predict rejects a negative or non-finite argument and accepts the valid edge.
TEST_F(ParticleFilterTest, PredictChecksArguments)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_NO_THROW(pf->predict(0.0, 0.0, 0.0, 1.0));
  EXPECT_THROW(pf->predict(-0.1, 0.0, 0.0, 1.0), std::invalid_argument);
  EXPECT_THROW(pf->predict(nan, 0.0, 0.0, 1.0), std::invalid_argument);
  EXPECT_THROW(pf->predict(0.0, nan, 0.0, 1.0), std::invalid_argument);
  EXPECT_THROW(pf->predict(0.0, 0.0, nan, 1.0), std::invalid_argument);
  EXPECT_THROW(pf->predict(0.0, 0.0, 0.0, nan), std::invalid_argument);
  EXPECT_THROW(pf->predict(0.0, inf, 0.0, 1.0), std::invalid_argument);
  EXPECT_THROW(pf->predict(inf, 0.0, 0.0, 1.0), std::invalid_argument);
  EXPECT_THROW(pf->predict(0.0, 0.0, inf, 1.0), std::invalid_argument);
  EXPECT_THROW(pf->predict(0.0, 0.0, 0.0, inf), std::invalid_argument);
}

// An empty scan is rejected without consuming the "first call": a later one-row scan is
// still registered (counter 1, map written, poses and weights untouched), not matched.
TEST_F(ParticleFilterTest, EmptyScanThrowsAndFirstCallIsStillPending)
{
  EXPECT_THROW(
    pf->update_and_resample(Eigen::Matrix<double, Eigen::Dynamic, 2>::Zero(0, 2)),
    std::invalid_argument);
  EXPECT_EQ(pf->global_counter(), 0);
  for (const auto & grid : pf->grids()) {
    EXPECT_EQ(grid.map_quality(), 0.0);
  }

  Eigen::Matrix<double, Eigen::Dynamic, 2> one(1, 2);
  one << 2.0, 0.0;
  ASSERT_NO_THROW(pf->update_and_resample(one));
  EXPECT_EQ(pf->global_counter(), 1);
  for (int i = 0; i < 10; ++i) {
    EXPECT_EQ(pf->poses()(0, i), 0.0);
    EXPECT_EQ(pf->weights()(i), 1.0 / 10);
  }
  for (const auto & grid : pf->grids()) {
    const int8_t * d = grid.data();
    const size_t n = static_cast<size_t>(grid.rows()) * static_cast<size_t>(grid.cols());
    bool written = false;
    for (size_t k = 0; k < n; ++k) {
      written = written || d[k] != 0;
    }
    EXPECT_TRUE(written);
  }
}

// --------------------------------------------------------------------------
// Test 2: predict with near-zero noise produces near-exact forward motion.
// --------------------------------------------------------------------------
TEST_F(ParticleFilterTest, PredictZeroNoiseExactMotion)
{
  // Set alphas to zero (only the 1e-9 floor remains).
  config.alpha1 = 0.0;
  config.alpha2 = 0.0;
  config.alpha3 = 0.0;
  config.alpha4 = 0.0;
  auto pf_quiet = std::make_unique<slam::ParticleFilter>(config, /*seed=*/ 42);

  // Move 1m forward, no rotation.
  pf_quiet->predict(1.0, 0.0, 0.0, 1.0);

  const auto & poses = pf_quiet->poses();
  for (int i = 0; i < 10; ++i) {
    EXPECT_NEAR(poses(0, i), 1.0, 1e-4);   // x ≈ 1.0
    EXPECT_NEAR(poses(1, i), 0.0, 1e-4);   // y ≈ 0.0
    EXPECT_NEAR(poses(2, i), 0.0, 1e-4);   // theta ≈ 0.0
  }
}

// --------------------------------------------------------------------------
// Test 3: predict with default noise spreads particles apart.
// --------------------------------------------------------------------------
TEST_F(ParticleFilterTest, PredictWithNoiseSpreadsPoses)
{
  pf->predict(1.0, 0.0, 0.0, 1.0);

  const auto & poses = pf->poses();

  // Compute standard deviation across particles for x, y, theta.
  Eigen::RowVectorXd x_row = poses.row(0);
  Eigen::RowVectorXd y_row = poses.row(1);
  Eigen::RowVectorXd th_row = poses.row(2);

  double mean_x = x_row.mean();
  double std_x = std::sqrt((x_row.array() - mean_x).square().mean());
  double std_y = std::sqrt((y_row.array() - y_row.mean()).square().mean());
  double std_th = std::sqrt((th_row.array() - th_row.mean()).square().mean());

  // Particles should have spread (non-zero std) due to noise.
  EXPECT_GT(std_x, 1e-6);
  EXPECT_GT(std_y, 1e-6);
  EXPECT_GT(std_th, 1e-6);

  // Mean should still be near 1.0 for x (forward motion).
  EXPECT_NEAR(mean_x, 1.0, 0.2);
}

// --------------------------------------------------------------------------
// Test 4: predict grows covariance from zero to positive-definite.
// --------------------------------------------------------------------------
TEST_F(ParticleFilterTest, PredictCovarianceGrows)
{
  pf->predict(1.0, 0.3, 0.1, 1.0);

  // Check covariance of the first particle.
  // Access via the poses/weights API: we don't have direct cov access,
  // so we test indirectly: if covs are PSD, Cholesky sampling in
  // update_and_resample won't fail. Here we verify that the particle
  // spread (which depends on covs feeding into the noise model) is nonzero.
  const auto & poses = pf->poses();
  double range_x = poses.row(0).maxCoeff() - poses.row(0).minCoeff();
  double range_th = poses.row(2).maxCoeff() - poses.row(2).minCoeff();

  // With rot1=0.3 and trans=1.0, there should be noticeable spread.
  EXPECT_GT(range_x, 1e-4);
  EXPECT_GT(range_th, 1e-4);
}

// --------------------------------------------------------------------------
// Test 7: predict with reverse direction moves particles backward.
// --------------------------------------------------------------------------
TEST_F(ParticleFilterTest, PredictReverseDirection)
{
  config.alpha1 = 0.0;
  config.alpha2 = 0.0;
  config.alpha3 = 0.0;
  config.alpha4 = 0.0;
  auto pf_quiet = std::make_unique<slam::ParticleFilter>(config, /*seed=*/ 42);

  // Move 1m with direction=-1 (reverse).
  pf_quiet->predict(1.0, 0.0, 0.0, -1.0);

  const auto & poses = pf_quiet->poses();
  for (int i = 0; i < 10; ++i) {
    EXPECT_NEAR(poses(0, i), -1.0, 1e-4);  // x ≈ -1.0 (backward)
    EXPECT_NEAR(poses(1, i), 0.0, 1e-4);
    EXPECT_NEAR(poses(2, i), 0.0, 1e-4);
  }
}

// --------------------------------------------------------------------------
// First update on a fresh filter only registers the scan in every grid.
// Room walls (hx=4.03, hy=3.02) lie inside the fixture's +-5 m map.
// --------------------------------------------------------------------------
TEST_F(ParticleFilterTest, FirstUpdateRegistersScanWithoutMovingParticles)
{
  std::mt19937 rng(7);
  auto scan = slam_test::scan_from({0.0, 0.0, 0.0}, 4.03, 3.02, 0.01, rng);
  pf->update_and_resample(scan);

  for (int i = 0; i < 10; ++i) {
    EXPECT_EQ(pf->poses()(0, i), 0.0);
    EXPECT_EQ(pf->poses()(1, i), 0.0);
    EXPECT_EQ(pf->poses()(2, i), 0.0);
    EXPECT_EQ(pf->weights()(i), 1.0 / 10);
  }
  EXPECT_EQ(pf->global_counter(), 1);

  for (const auto & grid : pf->grids()) {
    const int8_t * d = grid.data();
    const size_t n = static_cast<size_t>(grid.rows()) * static_cast<size_t>(grid.cols());
    bool has_pos = false, has_neg = false;
    for (size_t k = 0; k < n; ++k) {
      has_pos = has_pos || d[k] > 0;
      has_neg = has_neg || d[k] < 0;
    }
    EXPECT_TRUE(has_pos);
    EXPECT_TRUE(has_neg);
    double mq = grid.map_quality();
    EXPECT_GE(mq, 0.0);
    EXPECT_LE(mq, 1.0);
  }
}

// --------------------------------------------------------------------------
// Second update, same pose, no predict: matches against the registered map.
// --------------------------------------------------------------------------
TEST_F(ParticleFilterTest, SecondUpdateMatchesAgainstTheMap)
{
  std::mt19937 rng(7);
  pf->update_and_resample(slam_test::scan_from({0.0, 0.0, 0.0}, 4.03, 3.02, 0.01, rng));
  pf->update_and_resample(slam_test::scan_from({0.0, 0.0, 0.0}, 4.03, 3.02, 0.01, rng));

  Eigen::Vector3d best = pf->poses().col(static_cast<Eigen::Index>(pf->best_particle_index()));
  EXPECT_LT(std::hypot(best(0), best(1)), 0.02);
  EXPECT_LT(std::abs(best(2)), 0.01);
  EXPECT_NEAR(pf->weights().sum(), 1.0, 1e-12);
  EXPECT_EQ(pf->global_counter(), 2);
}

// --------------------------------------------------------------------------
// Fresh filter: all weights equal, the best particle is the first one.
// --------------------------------------------------------------------------
TEST_F(ParticleFilterTest, BestParticleIsIndexZeroOnFreshFilter)
{
  EXPECT_EQ(pf->best_particle_index(), 0u);
}

// --------------------------------------------------------------------------
// The best particle is chosen by cumulative log weight along its ancestry,
// which survives resampling (the weights themselves are reset to 1/N).
// obs_sigma_gain = 0.03 (default 3.0): with N = 10 the default gain flattens
// the weights so much that N_eff stays above N/2 and nothing resamples; at
// 0.03 N_eff drops below 5 on some steps, and a resample is visible as weights
// exactly 1/N (without one they are products of unequal factors).
// Steps where it resampled are found from that same observable, so the
// assertion on them is only "the reset is exact"; what the test pins is the
// pose of the best particle at every step and that, right after a resample,
// it is not always index 0 (seeded, so the run is fixed).
// The old behaviour (first maximum of the weights, i.e. index 0 after every
// resample) would fail that check.
// --------------------------------------------------------------------------
TEST_F(ParticleFilterTest, BestParticleSurvivesResampling)
{
  config.obs_sigma_gain = 0.03;
  slam::ParticleFilter filter(config, /*seed=*/ 42);
  std::mt19937 rng(7);
  const Eigen::Vector3d t0(0.0, 0.0, 0.0);
  filter.update_and_resample(slam_test::scan_from(t0, 4.03, 3.02, 0.01, rng));

  int resamples = 0;
  bool best_off_zero = false;
  for (int k = 1; k <= 8; ++k) {
    filter.predict(0.1, 0.0, 0.0, 1.0);
    const Eigen::Vector3d truth(0.1 * k, 0.0, 0.0);
    filter.update_and_resample(slam_test::scan_from(truth, 4.03, 3.02, 0.01, rng));

    const size_t best = filter.best_particle_index();
    ASSERT_LT(best, 10u);

    const Eigen::Vector3d expected = slam_test::rel(t0, truth);
    const Eigen::Vector3d est = filter.poses().col(static_cast<Eigen::Index>(best));
    EXPECT_LT((est.head<2>() - expected.head<2>()).norm(), 0.05) << "step " << k;

    if ((filter.weights().array() - 0.1).abs().maxCoeff() < 1e-15) {
      ++resamples;
      best_off_zero = best_off_zero || best != 0;
      for (int i = 0; i < 10; ++i) {
        EXPECT_EQ(filter.weights()(i), 1.0 / 10) << "step " << k;
      }
    }
  }
  EXPECT_GE(resamples, 2);  // otherwise the test did not exercise resampling
  EXPECT_TRUE(best_off_zero);
}

// --------------------------------------------------------------------------
// The cumulative log weight follows the particles through resampling: right
// after a resample the best particle is a copy of the ancestor with the
// largest cumulative weight, so its slot is the first of that ancestor's
// copies. The cumulative vector is private, so this is checked indirectly:
// obs_sigma_gain = 0.003 makes the weights decisive (one ancestor receives
// most of the copies), and on this seeded run the best particle's pose is
// then shared by the largest group of identical poses. If the cumulative
// vector were not permuted with the ancestors, the index it returns would
// point at an unrelated slot (a smaller group, and index 1 for most steps).
// Limits: it is an empirical property of this fixed run, not a theorem (with
// gain 0.03 the best ancestor is sometimes a group of one), and it does not
// check the cumulative values themselves, only which slot they select.
// --------------------------------------------------------------------------
TEST_F(ParticleFilterTest, BestParticleIsACopyOfTheHeaviestAncestorAfterResampling)
{
  config.obs_sigma_gain = 0.003;
  slam::ParticleFilter filter(config, /*seed=*/ 42);
  std::mt19937 rng(7);
  filter.update_and_resample(slam_test::scan_from({0.0, 0.0, 0.0}, 4.03, 3.02, 0.01, rng));

  int resamples = 0;
  for (int k = 1; k <= 10; ++k) {
    filter.predict(0.1, 0.0, 0.0, 1.0);
    filter.update_and_resample(
      slam_test::scan_from({0.1 * k, 0.0, 0.0}, 4.03, 3.02, 0.01, rng));
    if ((filter.weights().array() - 0.1).abs().maxCoeff() >= 1e-15) {
      continue;                                                                 // no resample
    }
    ++resamples;

    const auto & poses = filter.poses();
    auto group_size = [&](Eigen::Index i) {
        int n = 0;
        for (Eigen::Index j = 0; j < poses.cols(); ++j) {
          if ((poses.col(j) - poses.col(i)).norm() == 0.0) {
            ++n;                                                 // copies are bit-identical
          }
        }
        return n;
      };
    int largest = 0;
    for (Eigen::Index i = 0; i < poses.cols(); ++i) {
      largest = std::max(largest, group_size(i));
    }
    const Eigen::Index best = static_cast<Eigen::Index>(filter.best_particle_index());
    EXPECT_EQ(group_size(best), largest) << "step " << k;
    // First of its copies: no identical pose at a lower index.
    for (Eigen::Index j = 0; j < best; ++j) {
      EXPECT_GT((poses.col(j) - poses.col(best)).norm(), 0.0) << "step " << k;
    }
  }
  EXPECT_GE(resamples, 5);
}

#ifdef SLAM_HAS_OPENMP
// --------------------------------------------------------------------------
// Same seed, same result for 1 and 4 OpenMP threads (scan matching is
// per-particle and the RNG is used serially).
// --------------------------------------------------------------------------
TEST_F(ParticleFilterTest, SameSeedSameResultAcrossThreadCounts)
{
  config.obs_sigma_gain = 0.03;  // exercise resampling too
  const double pi = std::acos(-1.0);
  const double r = 1.5, dth = 0.04, chord = 2 * r * std::sin(dth / 2);

  auto run = [&](int threads) {
      omp_set_num_threads(threads);
      slam::ParticleFilter filter(config, /*seed=*/ 42);
      std::mt19937 rng(7);
      for (int k = 0; k <= 50; ++k) {
        const double a = dth * k;
        const Eigen::Vector3d truth(r * std::cos(a), r * std::sin(a), a + pi / 2);
        if (k > 0) {filter.predict(chord, dth / 2, dth / 2, 1.0);}
        filter.update_and_resample(slam_test::scan_from(truth, 4.03, 3.02, 0.01, rng));
      }
      return std::make_tuple(
        Eigen::MatrixXd(filter.poses()), Eigen::VectorXd(filter.weights()),
        filter.best_particle_index());
    };

  const int saved = omp_get_max_threads();
  const auto one = run(1);
  const auto four = run(4);
  omp_set_num_threads(saved);

  EXPECT_EQ(std::get<0>(one), std::get<0>(four));
  EXPECT_EQ(std::get<1>(one), std::get<1>(four));
  EXPECT_EQ(std::get<2>(one), std::get<2>(four));
}
#endif
