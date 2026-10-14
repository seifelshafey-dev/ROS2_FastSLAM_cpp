cmake_minimum_required(VERSION 3.8)
project(slam_cpp)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "No CMAKE_BUILD_TYPE given, defaulting to Release")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# C++17 required for constexpr LUT, structured bindings, std::optional
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependencies ---
find_package(Eigen3 REQUIRED)

# --- LIBRARY TARGET: slam_core ---
# Pure algorithm code. No ROS2 node logic. Tests link against this.
# This separation lets us test math without spinning up a ROS2 runtime.
add_library(slam_core INTERFACE)

target_include_directories(slam_core INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(slam_core INTERFACE Eigen3::Eigen)

# --- Testing ---
include(CTest)

if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  enable_testing()

  # Registers a GoogleTest executable.
  function(slam_add_gtest name source)
    add_executable(${name} ${source})
    target_link_libraries(${name} GTest::gtest_main)
    add_test(NAME ${name} COMMAND ${name})
  endfunction()

  # Example test: GoogleTest only, no RapidCheck. Extra args are CTest labels.
  function(slam_add_example_test name source)
    slam_add_gtest(${name} ${source})
    target_link_libraries(${name} slam_core)
    set_tests_properties(${name} PROPERTIES LABELS "${ARGN}")
  endfunction()

  # Unit tests link against slam_core
  slam_add_example_test(slam_types_test test/slam_types_test.cpp unit fast)
endif()
