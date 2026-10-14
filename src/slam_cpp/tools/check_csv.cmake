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

# Runs TOOL with ARGS (';'-separated) writing OUT, then checks exit code 0, the
# first line equals HEADER and at least one data row follows.
# Usage: cmake -DTOOL=.. -DOUT=.. -DHEADER=.. [-DARGS=..] -P check_csv.cmake
file(REMOVE "${OUT}")
execute_process(COMMAND "${TOOL}" "${OUT}" ${ARGS} RESULT_VARIABLE rc)
if(NOT rc EQUAL 0)
  message(FATAL_ERROR "${TOOL} exited with ${rc}")
endif()
file(STRINGS "${OUT}" lines)
list(LENGTH lines n)
list(GET lines 0 first)
if(NOT first STREQUAL HEADER)
  message(FATAL_ERROR "header '${first}' != '${HEADER}'")
endif()
if(n LESS 2)
  message(FATAL_ERROR "no data rows in ${OUT}")
endif()
