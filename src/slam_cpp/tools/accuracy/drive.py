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

"""
Drive the simulated robot on the pattern used for the accuracy runs.

Pattern: 5 s forward at 0.3 m/s, then -0.4 rad/s until the 8.93 s period
ends, repeated. The pattern starts 5 s after the first scan and the first
odometry message have arrived. It stops at 144 s, or earlier when any
beam reads under 0.4 m. Uses simulation time.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

PERIOD = 8.93
FORWARD_TIME = 5.0
FORWARD_SPEED = 0.3
TURN_RATE = -0.4
START_DELAY = 5.0
RUN_TIME = 144.0
MIN_RANGE = 0.4


class Driver(Node):
    """Publishes the drive pattern on the unstamped velocity topic."""

    def __init__(self):
        super().__init__(
            'drive',
            parameter_overrides=[
                rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.pub = self.create_publisher(
            Twist, '/diff_cont/cmd_vel_unstamped', 10)
        self.have_scan = False
        self.have_odom = False
        self.min_range = 9.0
        self.start_time = None
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.create_subscription(
            Odometry, '/odometry/filtered', self.on_odom, 10)
        self.create_timer(0.1, self.tick)

    def on_odom(self, _msg):
        self.have_odom = True

    def on_scan(self, msg):
        self.have_scan = True
        ranges = [r for r in msg.ranges
                  if math.isfinite(r) and r > msg.range_min]
        self.min_range = min(ranges) if ranges else 9.0

    def tick(self):
        if not (self.have_scan and self.have_odom):
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.start_time is None:
            self.start_time = now + START_DELAY
            self.get_logger().info('start')
        t = now - self.start_time
        cmd = Twist()
        if t < 0:
            pass
        elif t > RUN_TIME or self.min_range < MIN_RANGE:
            if t <= RUN_TIME:
                self.get_logger().warn(f'minr {self.min_range}')
            self.pub.publish(cmd)
            raise SystemExit
        elif t % PERIOD < FORWARD_TIME:
            cmd.linear.x = FORWARD_SPEED
        else:
            cmd.angular.z = TURN_RATE
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = Driver()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass


if __name__ == '__main__':
    main()
