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

"""Draw trajectories, position error and effective sample size of runs."""

import argparse

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

COLORS = ['#0072B2', '#D55E00', '#009E73', '#CC79A7', '#E69F00']


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv', nargs='+', help='filter_run_dump output files')
    ap.add_argument('--out', required=True)
    ap.add_argument('--labels', nargs='+')
    ap.add_argument('--neff', action='store_true',
                    help='add the effective sample size panel')
    ap.add_argument('--particles', type=int, default=80)
    a = ap.parse_args()
    labels = a.labels or [f'run {i + 1}' for i in range(len(a.csv))]
    if len(labels) != len(a.csv):
        ap.error('--labels needs one label per csv')

    runs = [np.genfromtxt(c, delimiter=',', names=True, dtype=None,
                          encoding=None) for c in a.csv]
    n = 3 if a.neff else 2
    fig, axes = plt.subplots(1, n, figsize=(5 * n, 4.8))
    ax = axes[0]
    ax.plot(runs[0]['gt_x'], runs[0]['gt_y'], '-', color='#444444', lw=2.5,
            label='ground truth')
    for i, (r, lab) in enumerate(zip(runs, labels)):
        ax.plot(r['est_x'], r['est_y'], '-', color=COLORS[i % 5], lw=1.2,
                label=lab)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.legend(fontsize=9)

    ax = axes[1]
    for i, (r, lab) in enumerate(zip(runs, labels)):
        e = np.hypot(r['est_x'] - r['gt_x'], r['est_y'] - r['gt_y'])
        ax.plot(r['t'], e, color=COLORS[i % 5], label=lab)
    ax.set_xlabel('time (s)')
    ax.set_ylabel('position error (m)')
    ax.legend(fontsize=9)

    if a.neff:
        ax = axes[2]
        for i, (r, lab) in enumerate(zip(runs, labels)):
            c = COLORS[i % 5]
            ax.plot(r['t'], r['neff'], color=c, lw=1, label=lab)
            m = r['resampled'] == 1
            ax.plot(r['t'][m], r['neff'][m], 'v', color=c, ms=5)
        if any((r['resampled'] == 1).any() for r in runs):
            ax.plot([], [], 'v', color='#444444', label='resampled')
        else:
            ax.text(0.5, 0.2, 'no resampling in this run',
                    transform=ax.transAxes, ha='center')
        ax.axhline(a.particles / 2, color='#444444', ls='--', lw=1,
                   label=f'N/2 = {a.particles / 2:g}')
        ax.set_xlabel('time (s)')
        ax.set_ylabel('effective sample size')
        ax.set_ylim(0, a.particles * 1.05)
        ax.legend(fontsize=9)
    fig.tight_layout()
    fig.savefig(a.out, dpi=150)


if __name__ == '__main__':
    main()
