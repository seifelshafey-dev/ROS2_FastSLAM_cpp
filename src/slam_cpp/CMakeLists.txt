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
add_library(slam_core SHARED
  src/occupancy_grid.cpp
  src/slam_config.cpp
  src/odometry.cpp
)

target_include_directories(slam_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(slam_core PUBLIC Eigen3::Eigen)

# --- Testing ---
include(CTest)

if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  enable_testing()

  option(SLAM_BUILD_PROPERTY_TESTS "Build the RapidCheck property tests" ON)

  # RapidCheck is added with EXCLUDE_FROM_ALL so its tests/examples are not built
  # and its install rules stay out of the package.
  if(SLAM_BUILD_PROPERTY_TESTS)
    include(FetchContent)
    fetchcontent_declare(rapidcheck
      GIT_REPOSITORY https://github.com/emil-e/rapidcheck.git
      GIT_TAG b2d9ed2dddefc4b84318d664b4f221eb792d89c7
      GIT_SHALLOW FALSE
    )
    set(RC_ENABLE_GTEST ON CACHE BOOL "" FORCE)
    fetchcontent_getproperties(rapidcheck)
    if(NOT rapidcheck_POPULATED)
      fetchcontent_populate(rapidcheck)
      add_subdirectory(${rapidcheck_SOURCE_DIR} ${rapidcheck_BINARY_DIR} EXCLUDE_FROM_ALL)
    endif()
  endif()

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

  # Property test: as above, plus RapidCheck.
  function(slam_add_property_test name source)
    slam_add_gtest(${name} ${source})
    target_link_libraries(${name} slam_core rapidcheck rapidcheck_gtest)
    set_tests_properties(${name} PROPERTIES LABELS "${ARGN}")
  endfunction()

  # Unit tests link against slam_core
  slam_add_example_test(occupancy_grid_test test/occupancy_grid_test.cpp unit fast)
  slam_add_example_test(odometry_test test/odometry_test.cpp unit fast)
  slam_add_example_test(slam_types_test test/slam_types_test.cpp unit fast)
  if(SLAM_BUILD_PROPERTY_TESTS)
    slam_add_property_test(slam_config_property_test test/slam_config_property_test.cpp prop fast)
    slam_add_property_test(odometry_property_test test/odometry_property_test.cpp prop fast)
  endif()
endif()
