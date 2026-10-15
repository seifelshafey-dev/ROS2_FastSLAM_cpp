#!/bin/bash
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

# Record one simulator run of the drive pattern into a zstd-compressed bag.
#
# Usage: record.sh <output bag directory>
#
# The workspace must be built and sourced. The simulator runs headless; a
# machine without a usable GPU or display may need the Mesa/GL environment
# variables set for Ignition rendering before this script is run.
# Nothing else should run on the machine while this records.

set -u

if [ $# -ne 1 ]; then
  echo "usage: $0 <output bag directory>" >&2
  exit 2
fi
bag=$1
here=$(cd "$(dirname "$0")" && pwd)

if [ -e "$bag" ]; then
  echo "output $bag already exists" >&2
  exit 1
fi
if pgrep -f '[i]gn gazebo' >/dev/null; then
  echo "a simulator is already running; stop it first" >&2
  exit 1
fi

status=0
stopped=0
bag_pid=
launch_pid=

# Stop the recorder, then the launch, then any simulator the launch left
# behind (the Ignition server can outlive it). Runs once. A recorder exit
# status of 0 after SIGINT is normal; any other status is a failure.
stop_all() {
  [ "$stopped" = 1 ] && return
  stopped=1
  if [ -n "$bag_pid" ]; then
    kill -INT "$bag_pid" 2>/dev/null
    wait "$bag_pid" || status=1
  fi
  # timeout runs the launch in its own process group, so signal it directly.
  if [ -n "$launch_pid" ]; then
    kill -INT "$launch_pid" 2>/dev/null
    wait "$launch_pid"
  fi
  leftover=$(pgrep -f '[i]gn gazebo')
  if [ -n "$leftover" ]; then
    # shellcheck disable=SC2086
    kill -INT $leftover 2>/dev/null
    for _ in 1 2 3 4 5 6 7 8 9 10; do
      leftover=$(pgrep -f '[i]gn gazebo')
      [ -z "$leftover" ] && break
      sleep 1
    done
    if [ -n "$leftover" ]; then
      echo "simulator needed SIGKILL: pid $leftover" >&2
      # shellcheck disable=SC2086
      kill -KILL $leftover 2>/dev/null
      sleep 1
      [ -z "$(pgrep -f '[i]gn gazebo')" ] || status=1
    fi
  fi
}
trap 'status=130; stop_all; exit 130' INT TERM

# Start recording before the launch so the bag holds the SLAM node's first
# update, which defines the map frame. With --use-sim-time the recorder
# stamps messages with /clock time.
ros2 bag record --use-sim-time --compression-mode file \
  --compression-format zstd -o "$bag" \
  /scan /odometry/filtered /diff_cont/odom /imu/data /tf /tf_static /clock \
  /ground_truth/poses /slam/best_pose /slam/map &
bag_pid=$!

sleep 3

if ! kill -0 "$bag_pid" 2>/dev/null; then
  echo "the recorder did not start" >&2
  status=1
  stop_all
  exit 1
fi

# Simulator, SLAM node and EKF. The simulator can run slower than real time,
# so the recording ends when the drive script ends, not on a wall-clock
# timeout; the timeouts are only a safety net.
timeout -s INT 1200 ros2 launch robot_bringup slam_sim.launch.py \
  gui:=false rviz:=false teleop:=false &
launch_pid=$!

sleep 50

# --foreground keeps drive.py in the terminal's process group so Ctrl-C reaches it.
timeout --foreground 600 python3 "$here/drive.py" || status=1
sleep 3

stop_all
exit "$status"
