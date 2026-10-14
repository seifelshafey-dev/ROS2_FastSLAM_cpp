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
Score a recorded run against the simulator's ground truth.

Usage: evaluate.py <bag_dir> [--out-prefix PATH] [--replay] [--map-gif FILE]
       [--map-png FILE]

The bag is a rosbag2 directory recorded with --use-sim-time, so record
times are simulation times. Ground truth and odometry are expressed in the
frame of their own pose at the first /slam/best_pose message, which is where
the map frame sits, before they are compared with the SLAM pose. Ground truth
and odometry are timed by the bag's record time, SLAM by the message header
stamp.

The first /slam/best_pose must be at the origin (the node's first update,
which defines the map frame); otherwise the script exits with status 1.

Prints the number of /slam/best_pose messages and the position error (rms, max, final;
all updates and moving updates only) and yaw rms for SLAM and odometry.
Writes <prefix>_trajectories.png, <prefix>_error.png and
<prefix>_metrics.json.

--replay plays the bag's /scan and /odometry/filtered into a fresh SLAM
node and scores the node's output instead of the recorded SLAM topics.
--map-gif writes an animation of every 5th /slam/map, cropped to the known
area of the last map, with the ground-truth path so far and the SLAM pose.
--map-png writes the last /slam/map, cropped the same way, with the true
wall faces of the simulator room and the SLAM trajectory.
"""

import argparse
import glob
import json
import math
import os
import signal
import subprocess
import tempfile
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import rosbag2_py  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402

MODEL_NAME = 'my_bot'
SLAM_TOPICS = ['/slam/best_pose', '/slam/map']
INPUT_TOPICS = ['/scan', '/odometry/filtered']
MOVING_SPEED = 0.01
MOVING_YAW_RATE = 0.01
RATE_WINDOW = 0.25
MAX_LINE_GAP = 1.0


def read_bag(path, topics):
    """Return {topic: [(time_s, message), ...]} for the wanted topics."""
    # The reader leaves a decompressed copy of each .zstd file next to it.
    before = set(glob.glob(os.path.join(path, '*.db3')))
    try:
        # A compressed bag needs the compression reader; an uncompressed one
        # (the --replay recording) is refused by it.
        compressed = bool(glob.glob(os.path.join(path, '*.zstd')))
        reader = (rosbag2_py.SequentialCompressionReader() if compressed
                  else rosbag2_py.SequentialReader())
        reader.open(
            rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('cdr', 'cdr'))
        types = {t.name: t.type for t in reader.get_all_topics_and_types()}
        classes = {t: get_message(types[t]) for t in topics if t in types}
        out = {t: [] for t in topics}
        while reader.has_next():
            topic, data, stamp_ns = reader.read_next()
            if topic in classes:
                out[topic].append((stamp_ns * 1e-9,
                                   deserialize_message(data, classes[topic])))
        del reader
    finally:
        for f in set(glob.glob(os.path.join(path, '*.db3'))) - before:
            os.remove(f)
    return out


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


class Track:
    """A pose time series (t, x, y, yaw) with interpolation."""

    def __init__(self, t, x, y, yaw):
        self.t = np.asarray(t, float)
        self.x = np.asarray(x, float)
        self.y = np.asarray(y, float)
        self.yaw = np.unwrap(np.asarray(yaw, float))

    def at(self, t):
        """Linear interpolation; yaw follows the shortest arc."""
        return (np.interp(t, self.t, self.x), np.interp(t, self.t, self.y),
                wrap(np.interp(t, self.t, self.yaw)))

    def moving(self, t):
        """Return true where the speed or yaw rate around t exceeds a threshold."""
        t0, t1 = t - RATE_WINDOW, t + RATE_WINDOW
        x0, y0, a0 = self.at(t0)
        x1, y1, a1 = self.at(t1)
        dt = t1 - t0
        speed = np.hypot(x1 - x0, y1 - y0) / dt
        rate = np.abs(wrap(a1 - a0)) / dt
        return (speed > MOVING_SPEED) | (rate > MOVING_YAW_RATE)


def to_frame(pose, ref):
    """Express pose (x, y, yaw arrays) in the frame of ref (x, y, yaw)."""
    x, y, a = pose
    rx, ry, ra = ref
    c, s = math.cos(ra), math.sin(ra)
    dx, dy = x - rx, y - ry
    return c * dx + s * dy, -s * dx + c * dy, wrap(a - ra)


def ground_truth_track(records):
    t, x, y, yaw = [], [], [], []
    for stamp, msg in records:
        for tr in msg.transforms:
            if tr.child_frame_id == MODEL_NAME:
                t.append(stamp)
                x.append(tr.transform.translation.x)
                y.append(tr.transform.translation.y)
                yaw.append(yaw_of(tr.transform.rotation))
    if not t:
        raise SystemExit(f'no ground-truth transform for {MODEL_NAME}')
    return Track(t, x, y, yaw)


def odometry_track(records):
    pose = [m.pose.pose for _, m in records]
    return Track([s for s, _ in records],
                 [p.position.x for p in pose], [p.position.y for p in pose],
                 [yaw_of(p.orientation) for p in pose])


def stats(err, yaw_err, moving):
    return {
        'rms': float(np.sqrt(np.mean(err ** 2))),
        'max': float(np.max(err)),
        'rms_moving': float(np.sqrt(np.mean(err[moving] ** 2))),
        'max_moving': float(np.max(err[moving])),
        'final': float(err[-1]),
        'yaw_rms': float(np.sqrt(np.mean(yaw_err ** 2))),
    }


def evaluate(slam_records, gt, odom):
    """Return metrics and the arrays needed for plotting."""
    stamps = np.array([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                       for _, m in slam_records])
    sl = (np.array([m.pose.position.x for _, m in slam_records]),
          np.array([m.pose.position.y for _, m in slam_records]),
          np.array([yaw_of(m.pose.orientation) for _, m in slam_records]))
    ref_gt = tuple(float(v) for v in gt.at(stamps[0]))
    ref_od = tuple(float(v) for v in odom.at(stamps[0]))
    g = to_frame(gt.at(stamps), ref_gt)
    o = to_frame(odom.at(stamps), ref_od)
    moving = gt.moving(stamps)
    result = {'updates': int(len(stamps)), 'moving_updates': int(moving.sum())}
    errors = {}
    for name, p in (('slam', sl), ('odometry', o)):
        err = np.hypot(p[0] - g[0], p[1] - g[1])
        result[name] = stats(err, wrap(p[2] - g[2]), moving)
        errors[name] = err
    return result, stamps, g, o, sl, errors, ref_gt


def plot(prefix, stamps, g, o, sl, errors, gt, ref_gt):
    full = to_frame((gt.x, gt.y, gt.yaw), ref_gt)
    fig, ax = plt.subplots(figsize=(7, 6))
    ax.plot(full[0], full[1], 'k-', label='ground truth')
    ax.plot(o[0], o[1], 'tab:orange', label='odometry')
    ax.plot(sl[0], sl[1], 'tab:blue', label='SLAM')
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.legend()
    fig.savefig(prefix + '_trajectories.png', dpi=150)
    plt.close(fig)
    fig, ax = plt.subplots(figsize=(8, 4))
    # Leave a gap in the line where consecutive updates are far apart.
    gaps = np.nonzero(np.diff(stamps) > MAX_LINE_GAP)[0] + 1
    t = np.insert(stamps - stamps[0], gaps, np.nan)
    ax.plot(t, np.insert(errors['odometry'], gaps, np.nan), 'tab:orange',
            label='odometry')
    ax.plot(t, np.insert(errors['slam'], gaps, np.nan), 'tab:blue',
            label='SLAM')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('position error (m)')
    ax.legend()
    fig.savefig(prefix + '_error.png', dpi=150)
    plt.close(fig)


def map_array(m):
    """Return (data, extent) of an OccupancyGrid; extent in metres."""
    w, h, res = m.info.width, m.info.height, m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    return (np.array(m.data, dtype=float).reshape(h, w),
            (ox, ox + w * res, oy, oy + h * res))


def known_box(m, margin=1.0):
    """Bounding box (x0, x1, y0, y1) of known cells plus a margin (m)."""
    data, (x0, x1, y0, y1) = map_array(m)
    rows, cols = np.nonzero(data != -1)
    res = m.info.resolution
    return (x0 + cols.min() * res - margin, x0 + (cols.max() + 1) * res + margin,
            y0 + rows.min() * res - margin, y0 + (rows.max() + 1) * res + margin)


def draw_map(ax, m, box):
    data, extent = map_array(m)
    data[data < 0] = 50
    ax.imshow(100 - data, cmap='gray', vmin=0, vmax=100, origin='lower',
              extent=extent)
    ax.set_xlim(box[0], box[1])
    ax.set_ylim(box[2], box[3])
    ax.set_aspect('equal')
    ax.set_xlabel('x in map (m)')
    ax.set_ylabel('y in map (m)')


def map_gif(path, maps, gt, ref_gt, stamps, sl):
    from matplotlib.animation import PillowWriter
    full = to_frame((gt.x, gt.y, gt.yaw), ref_gt)
    slam = Track(stamps, *sl)
    box = known_box(maps[-1][1])
    fig, ax = plt.subplots(figsize=(6, 6))
    writer = PillowWriter(fps=5)
    with writer.saving(fig, path, dpi=90):
        for stamp, m in maps[::5]:
            ax.clear()
            draw_map(ax, m, box)
            n = np.searchsorted(gt.t, stamp)
            ax.plot(full[0][:n], full[1][:n], 'r-', lw=1,
                    label='ground truth')
            x, y, _ = slam.at(stamp)
            ax.plot([x], [y], 'o', color='tab:blue', ms=5, label='SLAM pose')
            ax.legend(loc='upper right', fontsize=8)
            ax.set_title(f't = {stamp:.0f} s')
            writer.grab_frame()
    plt.close(fig)


def map_png(path, m, ref_gt, sl):
    """Draw the final map with the true room wall faces and the SLAM trajectory."""
    wx, wy = 4.925, 3.925
    corners = ([-wx, wx, wx, -wx, -wx], [-wy, -wy, wy, wy, -wy])
    wx, wy, _ = to_frame((np.array(corners[0]), np.array(corners[1]),
                          np.zeros(5)), ref_gt)
    fig, ax = plt.subplots(figsize=(7, 6))
    draw_map(ax, m, known_box(m))
    ax.plot(wx, wy, color='#D55E00', lw=1, ls='--', label='true wall faces')
    ax.plot(sl[0], sl[1], color='#0072B2', lw=1.5, label='SLAM trajectory')
    ax.legend(loc='upper right', fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def stop(*procs):
    for p in procs:
        if p.poll() is None:
            p.send_signal(signal.SIGINT)
    for p in procs:
        try:
            p.wait(timeout=20)
        except subprocess.TimeoutExpired:
            p.kill()


def replay(bag, tmp):
    """Run the bag's inputs through a fresh node; return the output bag."""
    out = os.path.join(tmp, 'replay')
    node = subprocess.Popen(
        ['ros2', 'launch', 'slam_cpp', 'slam_cpp.launch.py',
         'use_sim_time:=true'])
    time.sleep(5)
    rec = subprocess.Popen(
        ['ros2', 'bag', 'record', '--use-sim-time', '-o', out] + SLAM_TOPICS)
    time.sleep(3)
    play = subprocess.Popen(
        ['ros2', 'bag', 'play', bag, '--clock', '--topics'] + INPUT_TOPICS)
    try:
        play.wait()
        time.sleep(3)
    finally:
        stop(play, rec, node)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('bag')
    ap.add_argument('--out-prefix', default='eval')
    ap.add_argument('--replay', action='store_true')
    ap.add_argument('--map-gif')
    ap.add_argument('--map-png')
    args = ap.parse_args()

    data = read_bag(args.bag, ['/odometry/filtered', '/ground_truth/poses']
                    + SLAM_TOPICS)
    if args.replay:
        with tempfile.TemporaryDirectory() as tmp:
            data.update(read_bag(replay(args.bag, tmp), SLAM_TOPICS))
    gt = ground_truth_track(data['/ground_truth/poses'])
    odom = odometry_track(data['/odometry/filtered'])
    slam = data['/slam/best_pose']
    if not slam:
        raise SystemExit('no /slam/best_pose messages')
    p0 = slam[0][1].pose
    x0, y0, a0 = p0.position.x, p0.position.y, yaw_of(p0.orientation)
    if max(abs(x0), abs(y0)) > 1e-3 or abs(a0) > 1e-3:
        raise SystemExit(
            'first /slam/best_pose is not at the origin '
            f'({x0:.3f}, {y0:.3f}, {a0:.3f}); '
            "the bag misses the node's first update")

    result, stamps, g, o, sl, errors, ref_gt = evaluate(slam, gt, odom)
    plot(args.out_prefix, stamps, g, o, sl, errors, gt, ref_gt)
    with open(args.out_prefix + '_metrics.json', 'w') as f:
        json.dump(result, f, indent=2)
    print(f"poses {result['updates']} "
          f"(moving {result['moving_updates']})")
    for name in ('slam', 'odometry'):
        r = result[name]
        print(f"{name:9s} rms {r['rms']:.3f} m  max {r['max']:.3f} m  "
              f"rms moving {r['rms_moving']:.3f} m  "
              f"max moving {r['max_moving']:.3f} m  "
              f"final {r['final']:.3f} m  yaw rms {r['yaw_rms']:.3f} rad")
    if args.map_gif:
        map_gif(args.map_gif, data['/slam/map'], gt, ref_gt, stamps, sl)
    if args.map_png:
        map_png(args.map_png, data['/slam/map'][-1][1], ref_gt, sl)


if __name__ == '__main__':
    main()
