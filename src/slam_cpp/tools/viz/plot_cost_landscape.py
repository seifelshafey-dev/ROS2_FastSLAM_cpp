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

"""Draw the joint cost around the true pose for a one-scan and a full map."""

import argparse

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv', help='columns map,dx,dy,cost')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()

    with open(a.csv) as f:
        rows = [r.strip().split(',') for r in f.readlines()[1:] if r.strip()]
    fig, axes = plt.subplots(1, 2, figsize=(11, 5), sharey=True)
    for ax, (key, label) in zip(axes, (('one', 'one-scan map'),
                                       ('full', 'full map'))):
        d = np.array([[float(v) for v in r[1:]] for r in rows if r[0] == key])
        xs, ys = np.unique(d[:, 0]), np.unique(d[:, 1])
        order = np.lexsort((d[:, 0], d[:, 1]))
        z = d[order, 2].reshape(len(ys), len(xs))
        im = ax.pcolormesh(xs, ys, z, cmap='viridis', shading='nearest')
        ax.contour(xs, ys, z, 10, colors='white', linewidths=0.5)
        j = np.unravel_index(np.argmin(z), z.shape)
        ax.plot(0, 0, 'o', mfc='none', mec='#E69F00', ms=12, mew=2,
                label='true pose')
        ax.plot(xs[j[1]], ys[j[0]], 'x', color='#D55E00', ms=10, mew=2,
                label='minimum')
        ax.set_aspect('equal')
        ax.set_xlabel('dx from true pose (m)')
        ax.text(0.03, 0.95, label, transform=ax.transAxes, color='white',
                va='top', fontsize=11)
        fig.colorbar(im, ax=ax, label='joint cost', shrink=0.8)
    axes[0].set_ylabel('dy from true pose (m)')
    axes[0].legend(loc='lower left', fontsize=9)
    fig.tight_layout()
    fig.savefig(a.out, dpi=150)


if __name__ == '__main__':
    main()
