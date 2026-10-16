cmake_minimum_required(VERSION 3.8)
project(slam_cpp)

option(SLAM_BUILD_ROS "Build the ROS 2 node and package it with ament" ON)
option(SLAM_BUILD_TOOLS "Build the ROS-free figure tools" OFF)

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
if(SLAM_BUILD_ROS)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(tf2_ros REQUIRED)
endif()
find_package(Eigen3 REQUIRED)
find_package(OpenMP)

# --- LIBRARY TARGET: slam_core ---
# Pure algorithm code. No ROS2 node logic. Tests link against this.
# This separation lets us test math without spinning up a ROS2 runtime.
add_library(slam_core SHARED
  src/occupancy_grid.cpp
  src/slam_config.cpp
  src/scan_matching.cpp
  src/particle_filter.cpp
  src/odometry.cpp
)

target_include_directories(slam_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(slam_core PUBLIC Eigen3::Eigen)

# OpenMP for parallel scan matching in particle_filter.cpp
if(OpenMP_CXX_FOUND)
  target_link_libraries(slam_core PUBLIC OpenMP::OpenMP_CXX)
  target_compile_definitions(slam_core PUBLIC SLAM_HAS_OPENMP)
endif()

if(SLAM_BUILD_ROS)
  # --- EXECUTABLE TARGET: slam_node ---
  # ROS2 wiring: subscriptions, publishers, TF, parameter loading.
  # Links against slam_core for the actual algorithms.
  #
  # NOTE: ament_target_dependencies uses "plain" target_link_libraries internally,
  # so we must also use plain mode for slam_core (no PRIVATE/PUBLIC keyword).
  # Mixing keyword and plain modes on the same target is a CMake error.
  add_executable(slam_node src/ros/slam_node.cpp)
  target_link_libraries(slam_node slam_core)
  ament_target_dependencies(slam_node
    rclcpp nav_msgs sensor_msgs geometry_msgs tf2_ros
  )

  # --- Install ---
  install(TARGETS slam_node DESTINATION lib/${PROJECT_NAME})

  install(TARGETS slam_core
    EXPORT export_slam_core
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  install(DIRECTORY include/ DESTINATION include)

  install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}/
  )

  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
  )

  ament_export_targets(export_slam_core HAS_LIBRARY_TARGET)
  if(OpenMP_CXX_FOUND)
    ament_export_dependencies(Eigen3 OpenMP)
  else()
    ament_export_dependencies(Eigen3)
  endif()
endif()

# --- Testing ---
if(NOT SLAM_BUILD_ROS)
  include(CTest)
endif()

if(BUILD_TESTING)
  if(SLAM_BUILD_ROS)
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)

    ament_lint_auto_find_test_dependencies()
  else()
    find_package(GTest REQUIRED)
    enable_testing()
  endif()

  set(SLAM_RC_PARAMS "seed=1" CACHE STRING
    "RapidCheck RC_PARAMS for the property tests, e.g. seed=1 (fixed, default) or max_success=1000")
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

  # Registers a GoogleTest executable with ament under ROS, plain CMake otherwise.
  function(slam_add_gtest name source)
    if(SLAM_BUILD_ROS)
      ament_add_gtest(${name} ${source})
    else()
      add_executable(${name} ${source})
      target_link_libraries(${name} GTest::gtest_main)
      add_test(NAME ${name} COMMAND ${name})
    endif()
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
    set_tests_properties(${name} PROPERTIES LABELS "${ARGN}"
      ENVIRONMENT "RC_PARAMS=${SLAM_RC_PARAMS}")
  endfunction()

  # Unit tests link against slam_core, NOT slam_node
  slam_add_example_test(occupancy_grid_test test/occupancy_grid_test.cpp unit fast)
  slam_add_example_test(scan_matching_test test/scan_matching_test.cpp unit fast)
  slam_add_example_test(particle_filter_test test/particle_filter_test.cpp unit fast)
  slam_add_example_test(support_room_test test/support/room_test.cpp unit fast)
  slam_add_example_test(slam_config_test test/slam_config_test.cpp unit fast)
  slam_add_example_test(odometry_test test/odometry_test.cpp unit fast)
  slam_add_example_test(slam_types_test test/slam_types_test.cpp unit fast)
  slam_add_example_test(scan_matching_integration_test test/scan_matching_integration_test.cpp integration fast)
  slam_add_example_test(particle_filter_integration_test test/particle_filter_integration_test.cpp integration)
  if(SLAM_BUILD_ROS)
    find_package(launch_testing_ament_cmake REQUIRED)
    add_launch_test(test/slam_node_launch_test.py TARGET slam_node_launch_test TIMEOUT 60)
    set_tests_properties(slam_node_launch_test PROPERTIES LABELS "launch")
  endif()
  if(SLAM_BUILD_PROPERTY_TESTS)
    slam_add_property_test(scan_matching_property_test test/scan_matching_property_test.cpp prop fast)
    slam_add_property_test(support_room_property_test test/support/room_property_test.cpp prop fast)
    slam_add_property_test(slam_config_property_test test/slam_config_property_test.cpp prop fast)
    slam_add_property_test(odometry_property_test test/odometry_property_test.cpp prop fast)
    slam_add_property_test(occupancy_grid_property_test test/occupancy_grid_property_test.cpp prop fast)
    slam_add_property_test(slam_types_property_test test/slam_types_property_test.cpp prop fast)
  endif()
endif()

if(SLAM_BUILD_TOOLS)
  add_subdirectory(tools)
endif()

if(SLAM_BUILD_ROS)
  ament_package()
endif()
