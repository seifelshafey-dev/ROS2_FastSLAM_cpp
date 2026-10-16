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

"""Launch test of slam_node: first pose, latched pose, map, map to odom pairing."""

import math
import time
import unittest

from geometry_msgs.msg import PoseStamped
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing.actions
import launch_testing.asserts
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan
import tf2_msgs.msg

P0 = (1.0, 0.5, 0.3)
N_BEAMS = 360
KNOWN = {int(math.floor(100.0 / (1.0 + math.exp(-k / 20.0)) + 0.5))
         for k in range(-100, 101) if k != 0}
WALLS = (-1.0, 3.0, -1.5, 2.5)  # x_min, x_max, y_min, y_max in the odom frame
INF = float('inf')


def generate_test_description():
    """Start slam_node with a small map and five particles."""
    node = Node(
        package='slam_cpp', executable='slam_node', output='screen',
        parameters=[{
            'map_size': 10.0, 'map_resolution': 0.1, 'num_particles': 5,
            'alpha1': 0.0, 'alpha2': 0.0, 'alpha3': 0.0, 'alpha4': 0.0,
            'motion_threshold_trans': 0.2, 'motion_threshold_rot': 1.0,
            'map_publish_rate': 10.0}])
    return LaunchDescription([node, launch_testing.actions.ReadyToTest()]), {}


def ranges_from(px, py, th):
    """Return the 360 wall distances seen from pose (px, py, th)."""
    inc = 2.0 * math.pi / N_BEAMS
    out = []
    for i in range(N_BEAMS):
        a = th - math.pi + i * inc
        dx, dy = math.cos(a), math.sin(a)
        ts = []
        if abs(dx) > 1e-12:
            ts.append(((WALLS[1] if dx > 0 else WALLS[0]) - px) / dx)
        if abs(dy) > 1e-12:
            ts.append(((WALLS[3] if dy > 0 else WALLS[2]) - py) / dy)
        out.append(min(t for t in ts if t > 0))
    return out


def yaw_of(q):
    """Yaw of a z-axis quaternion."""
    return 2.0 * math.atan2(q.z, q.w)


def angle_diff(a, b):
    """Difference a - b wrapped to [-pi, pi)."""
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


def inverse_of_p0():
    """Return (x, y, yaw) of the transform map to odom expected from P0."""
    c, s = math.cos(-P0[2]), math.sin(-P0[2])
    return (-(c * P0[0] - s * P0[1]), -(s * P0[0] + c * P0[1]), -P0[2])


class TestSlamNode(unittest.TestCase):
    """Steps run in name order and share state through the class."""

    @classmethod
    def setUpClass(cls):
        """Create the test node, its publishers and subscriptions."""
        rclpy.init()
        cls.node = rclpy.create_node('slam_node_test_client')
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cls.poses, cls.maps, cls.tfs = [], [], []
        cls.odom_pub = cls.node.create_publisher(Odometry, '/odometry/filtered', 10)
        cls.scan_pub = cls.node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        cls.pose_sub = cls.node.create_subscription(
            PoseStamped, '/slam/best_pose', cls.poses.append, latched)
        cls.map_sub = cls.node.create_subscription(
            OccupancyGrid, '/slam/map', cls.maps.append, latched)
        cls.tf_sub = cls.node.create_subscription(
            tf2_msgs.msg.TFMessage, '/tf', cls.tfs.append, 100)

    @classmethod
    def tearDownClass(cls):
        """Destroy the test node."""
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def spin(cls, seconds):
        """Spin the test node for a duration."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(cls.node, timeout_sec=0.05)

    @classmethod
    def send_odom(cls, pose):
        """Publish an odometry message and let it arrive."""
        m = Odometry()
        m.header.frame_id = 'odom'
        m.pose.pose.position.x, m.pose.pose.position.y = pose[0], pose[1]
        m.pose.pose.orientation.z = math.sin(pose[2] / 2.0)
        m.pose.pose.orientation.w = math.cos(pose[2] / 2.0)
        cls.odom_pub.publish(m)
        cls.spin(0.3)

    @classmethod
    def send_scan(cls, ranges):
        """Publish one scan."""
        m = LaserScan()
        m.header.frame_id = 'base_link'
        m.angle_min = -math.pi
        m.angle_increment = 2.0 * math.pi / N_BEAMS
        m.angle_max = m.angle_min + (N_BEAMS - 1) * m.angle_increment
        m.range_min, m.range_max = 0.1, 10.0
        m.ranges = [float(r) for r in ranges]
        cls.scan_pub.publish(m)

    @classmethod
    def scans_until(cls, ranges, done, timeout):
        """Publish the scan every 0.2 s until done() is true or the timeout ends."""
        end = time.monotonic() + timeout
        while time.monotonic() < end and not done():
            cls.send_scan(ranges)
            cls.spin(0.2)
        return done()

    def last_map_to_odom(self):
        """Return (x, y, yaw) of the latest map to odom transform seen."""
        for msg in reversed(self.tfs):
            for t in msg.transforms:
                if t.header.frame_id == 'map' and t.child_frame_id == 'odom':
                    return (t.transform.translation.x, t.transform.translation.y,
                            yaw_of(t.transform.rotation))
        self.fail('no map to odom transform received')

    def test_01_nothing_before_odometry(self):
        """Scans before any odometry give no pose and no map."""
        end = time.monotonic() + 10.0
        while time.monotonic() < end and not (
                self.odom_pub.get_subscription_count() and self.scan_pub.get_subscription_count()
                and self.node.count_publishers('/slam/best_pose')
                and self.node.count_publishers('/slam/map')
                and self.node.count_publishers('/tf')):
            self.spin(0.1)
        self.assertGreaterEqual(self.odom_pub.get_subscription_count(), 1)
        self.assertGreaterEqual(self.scan_pub.get_subscription_count(), 1)
        self.assertGreaterEqual(self.node.count_publishers('/slam/best_pose'), 1)
        self.scans_until(ranges_from(*P0), lambda: False, 1.5)
        self.assertEqual(len(self.poses), 0)
        self.assertEqual(len(self.maps), 0)

    def test_02_scan_without_valid_beam(self):
        """A scan with every range infinite gives no pose."""
        self.send_odom(P0)
        self.scans_until([INF] * N_BEAMS, lambda: False, 1.5)
        self.assertEqual(len(self.poses), 0)

    def test_03_first_pose_and_transform(self):
        """The first pose is exactly the origin; map to odom is the inverse of P0."""
        self.send_odom(P0)
        self.assertTrue(self.scans_until(ranges_from(*P0), lambda: self.poses, 10.0))
        p = self.poses[0].pose
        self.assertAlmostEqual(p.position.x, 0.0, delta=1e-9)
        self.assertAlmostEqual(p.position.y, 0.0, delta=1e-9)
        self.assertAlmostEqual(yaw_of(p.orientation), 0.0, delta=1e-9)
        n = len(self.tfs)
        end = time.monotonic() + 5.0
        while len(self.tfs) == n and time.monotonic() < end:
            self.spin(0.1)
        x, y, yaw = self.last_map_to_odom()
        ex, ey, eyaw = inverse_of_p0()
        self.assertAlmostEqual(ex, -1.103096, delta=1e-5)
        self.assertAlmostEqual(ey, -0.182148, delta=1e-5)
        self.assertAlmostEqual(x, ex, delta=1e-5)
        self.assertAlmostEqual(y, ey, delta=1e-5)
        self.assertAlmostEqual(angle_diff(yaw, eyaw), 0.0, delta=1e-5)

    def test_04_late_subscriber_gets_last_pose(self):
        """A transient-local subscriber created now receives the stored pose."""
        got = []
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sub = self.node.create_subscription(PoseStamped, '/slam/best_pose', got.append, latched)
        end = time.monotonic() + 2.0
        while not got and time.monotonic() < end:
            self.spin(0.05)
        self.node.destroy_subscription(sub)
        self.assertEqual(len(got), 1)
        self.assertAlmostEqual(got[0].pose.position.x, 0.0, delta=1e-9)
        self.assertAlmostEqual(got[0].pose.position.y, 0.0, delta=1e-9)

    def test_05_map(self):
        """The map has the configured geometry, unknown cells, free and occupied values."""
        end = time.monotonic() + 5.0
        while not self.maps and time.monotonic() < end:
            self.spin(0.1)
        self.assertTrue(self.maps)
        m = self.maps[-1]
        self.assertEqual((m.info.width, m.info.height), (100, 100))
        self.assertAlmostEqual(m.info.resolution, 0.1, places=6)
        self.assertAlmostEqual(m.info.origin.position.x, -5.0, delta=1e-9)
        self.assertAlmostEqual(m.info.origin.position.y, -5.0, delta=1e-9)
        values = set(m.data)
        print('map values present: %s' % sorted(values))
        for v in values:
            self.assertTrue(v == -1 or v in KNOWN, v)
        self.assertEqual(m.data[95 * 100 + 95], -1)
        self.assertTrue(0 <= m.data[50 * 100 + 55] < 50, m.data[50 * 100 + 55])
        self.assertTrue(any(v > 50 for v in values))

    def test_06_forward_motion(self):
        """After 0.25 m forward the second pose is near (0.25, 0, 0)."""
        c, s = math.cos(P0[2]), math.sin(P0[2])
        p1 = (P0[0] + 0.25 * c, P0[1] + 0.25 * s, P0[2])
        self.send_odom(p1)
        self.assertTrue(self.scans_until(ranges_from(*p1), lambda: len(self.poses) >= 2, 10.0))
        p = self.poses[1].pose
        dx, dy = p.position.x - 0.25, p.position.y
        dyaw = angle_diff(yaw_of(p.orientation), 0.0)
        print('step 6 error: dx=%.5f dy=%.5f dyaw=%.5f' % (dx, dy, dyaw))
        self.assertLess(math.hypot(dx, dy), 0.03)
        self.assertLess(abs(dyaw), 0.03)

    def test_07_transform_pairs_pose_with_its_odometry(self):
        """Odometry below the threshold changes neither the pose nor the transform."""
        c, s = math.cos(P0[2]), math.sin(P0[2])
        p2 = (P0[0] + 0.4 * c, P0[1] + 0.4 * s, P0[2])
        self.send_odom(p2)
        self.scans_until(ranges_from(*p2), lambda: False, 1.5)
        self.assertEqual(len(self.poses), 2)
        n = len(self.tfs)
        end = time.monotonic() + 5.0
        while len(self.tfs) == n and time.monotonic() < end:
            self.spin(0.1)
        x, y, yaw = self.last_map_to_odom()
        ex, ey, eyaw = inverse_of_p0()
        print('step 7 error: dx=%.5f dy=%.5f dyaw=%.5f' % (x - ex, y - ey, angle_diff(yaw, eyaw)))
        self.assertLess(math.hypot(x - ex, y - ey), 0.03)
        self.assertLess(abs(angle_diff(yaw, eyaw)), 0.03)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Checks made after the launch has stopped."""

    def test_exit_code(self, proc_info):
        """The node exits with code 0 on shutdown."""
        launch_testing.asserts.assertExitCodes(proc_info)
