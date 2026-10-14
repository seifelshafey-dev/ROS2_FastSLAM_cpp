# Figure data tools: ROS-free, not installed. Enabled with -DSLAM_BUILD_TOOLS=ON.
foreach(tool cost_landscape_dump motion_cloud_dump filter_run_dump)
  add_executable(${tool} ${tool}.cpp)
  # slam_core carries Eigen and the OpenMP setting publicly.
  target_link_libraries(${tool} PRIVATE slam_core)
  target_include_directories(${tool} PRIVATE ${PROJECT_SOURCE_DIR}/test)
endforeach()

if(BUILD_TESTING)
  include(tests.cmake)
endif()
