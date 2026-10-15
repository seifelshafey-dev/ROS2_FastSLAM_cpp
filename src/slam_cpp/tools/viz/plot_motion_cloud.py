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

"""Draw the particle cloud at each step of a scripted motion."""

import argparse

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv', help='columns step,particle,x,y,theta')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()

    d = np.genfromtxt(a.csv, delimiter=',', names=True)
    steps = np.unique(d['step'])
    fig, ax = plt.subplots(figsize=(8, 5))
    cm = plt.get_cmap('viridis')
    for i, s in enumerate(steps):
        m = (d['step'] == s) & (d['particle'] >= 0)
        ax.scatter(d['x'][m], d['y'][m], s=8, alpha=0.5, linewidths=0,
                   color=cm(i / max(len(steps) - 1, 1)),
                   label=f'step {int(s)}')
    p = d[d['particle'] < 0]
    ax.plot(p['x'], p['y'], '-o', color='#D55E00', ms=3, lw=1.2,
            label='noise-free path')
    ax.set_aspect('equal')
    ax.margins(0.05)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.legend(loc='center left', bbox_to_anchor=(1.01, 0.5), fontsize=8,
              markerscale=2)
    fig.tight_layout()
    fig.savefig(a.out, dpi=150)


if __name__ == '__main__':
    main()
