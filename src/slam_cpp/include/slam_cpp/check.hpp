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

#ifndef SLAM_CPP__CHECK_HPP_
#define SLAM_CPP__CHECK_HPP_

/**
 * @file check.hpp
 * @brief SLAM_CHECK, an always-on check for input that can be wrong at run time.
 */

#include <stdexcept>
#include <string>

/**
 * @brief Throw if a condition on external input does not hold.
 *
 * Active in every build type. Meant for values that can be wrong at run time, such as
 * configuration; not for internal invariants.
 *
 * @param cond Condition that must hold.
 * @param msg  Message, an expression convertible to `std::string`; evaluated only when
 *             `cond` is false.
 * @throws std::invalid_argument with the text "<file>:<line>: <msg>" if `cond` is false.
 */
#define SLAM_CHECK(cond, msg) \
  do { \
    if (!(cond)) { \
      throw std::invalid_argument( \
              std::string(__FILE__) + ":" + \
              std::to_string(__LINE__) + ": " + (msg)); \
    } \
  } while (0)

#endif  // SLAM_CPP__CHECK_HPP_
