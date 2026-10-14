# Copyright 2026 Seif ElShafey
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Smoke tests for the dump tools; included from CMakeLists.txt when
# SLAM_BUILD_TOOLS and BUILD_TESTING are on.
function(slam_tool_smoke tool header)
  add_test(NAME ${tool}_smoke
    COMMAND ${CMAKE_COMMAND}
      -DTOOL=$<TARGET_FILE:${tool}>
      -DOUT=${CMAKE_CURRENT_BINARY_DIR}/${tool}_smoke.csv
      "-DHEADER=${header}"
      "-DARGS=${ARGN}"
      -P ${CMAKE_CURRENT_LIST_DIR}/check_csv.cmake)
  set_tests_properties(${tool}_smoke PROPERTIES LABELS "full;tools")
endfunction()

slam_tool_smoke(cost_landscape_dump "map,dx,dy,cost")
slam_tool_smoke(motion_cloud_dump "step,particle,x,y,theta")
slam_tool_smoke(filter_run_dump
  "k,t,phase,gt_x,gt_y,gt_th,est_x,est_y,est_th,neff,resampled" --res 0.2)
