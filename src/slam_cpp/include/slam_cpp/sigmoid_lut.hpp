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

#ifndef SLAM_CPP__SIGMOID_LUT_HPP_
#define SLAM_CPP__SIGMOID_LUT_HPP_

/**
 * @file sigmoid_lut.hpp
 * @brief Lookup table from a stored int8 grid cell value to an occupancy probability.
 */

#include <array>
#include <cmath>
#include <cstdint>

namespace slam
{

/// Maps a stored cell value (int8 log-odds times a scale) to an occupancy probability.
/// Holds one entry for every int8 value.
class SigmoidLUT
{
public:
  /**
   * @brief Build the table for a given log-odds scale.
   *
   * @param scale Factor by which the grid multiplies log-odds before storing them as int8;
   *              must be > 0 (not checked here). Use the grid's `log_odds_scale`.
   */
  explicit SigmoidLUT(double scale = 20.0)
  {
    for (int i = 0; i < 256; ++i) {
      // Cast index to int8_t via uint8_t reinterpretation (maps 0..255 to -128..127).
      auto raw = static_cast<int8_t>(static_cast<uint8_t>(i));
      double log_odds = static_cast<double>(raw) / scale;
      table_[i] = 1.0 / (1.0 + std::exp(-log_odds));
    }
  }

  /**
   * @brief Occupancy probability of a stored cell value.
   *
   * @param log_odds_scaled Stored cell value, any int8; 0 (unknown) gives 0.5.
   * @return sigmoid(log_odds_scaled / scale), in [0, 1].
   */
  double operator()(int8_t log_odds_scaled) const
  {
    // The uint8_t cast maps [-128, 127] to the index range [0, 255].
    return table_[static_cast<uint8_t>(log_odds_scaled)];
  }

private:
  // 256 doubles, 2 KB.
  std::array<double, 256> table_;
};

}  // namespace slam

#endif  // SLAM_CPP__SIGMOID_LUT_HPP_
