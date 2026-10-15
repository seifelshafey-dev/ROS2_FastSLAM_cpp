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

"""Draw the map, odom and base_link frames and the transforms between them."""

import argparse

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


def compose(a, b):
    """Return pose b expressed in the frame in which pose a is given."""
    c, s = np.cos(a[2]), np.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1],
            a[2] + b[2])


def frame(ax, pose, name, color, ln=0.6):
    x, y, th = pose
    for ang, ls in ((th, '-'), (th + np.pi / 2, '--')):
        ax.annotate('', (x + ln * np.cos(ang), y + ln * np.sin(ang)), (x, y),
                    arrowprops=dict(arrowstyle='->', color=color, lw=2,
                                    ls=ls))
    ax.text(x - 0.1, y - 0.1, name, color=color, fontsize=12, ha='right',
            va='top')


def link(ax, a, b, text, color, dx, dy):
    ax.annotate('', b[:2], a[:2],
                arrowprops=dict(arrowstyle='->', color=color, lw=1.2,
                                ls=':', shrinkA=8, shrinkB=8))
    ax.text((a[0] + b[0]) / 2 + dx, (a[1] + b[1]) / 2 + dy, text,
            color=color, fontsize=10, ha='center')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    out = ap.parse_args().out

    m = (0.0, 0.0, 0.0)
    t_mo = (1.0, 0.8, 0.3)      # map -> odom
    t_ob = (2.0, 1.0, 0.6)      # odom -> base_link
    mb = compose(t_mo, t_ob)    # map -> base_link

    fig, ax = plt.subplots(figsize=(8, 5.5))
    frame(ax, m, 'map', '#0072B2')
    frame(ax, t_mo, 'odom', '#009E73')
    frame(ax, mb, 'base_link', '#D55E00')
    link(ax, m, t_mo, 'map->odom\n(1.0, 0.8) m, 0.3 rad', '#009E73', 0.9, -0.5)
    link(ax, t_mo, mb, 'odom->base_link\n(2.0, 1.0) m, 0.6 rad in odom',
         '#D55E00', 1.5, -0.4)
    link(ax, m, mb, 'map->base_link =\n(map->odom) o (odom->base_link)\n'
         f'({mb[0]:.2f}, {mb[1]:.2f}) m, {mb[2]:.1f} rad', '#0072B2', -1.3, 0.5)
    ax.plot([], [], '-', color='#444444', label='x axis')
    ax.plot([], [], '--', color='#444444', label='y axis')
    ax.legend(loc='lower right', fontsize=9)
    ax.set_aspect('equal')
    ax.set_xlim(-2.5, 5.0)
    ax.set_ylim(-1.2, 3.6)
    ax.set_xlabel('x in map (m)')
    ax.set_ylabel('y in map (m)')
    fig.tight_layout()
    fig.savefig(out, dpi=150)


if __name__ == '__main__':
    main()
