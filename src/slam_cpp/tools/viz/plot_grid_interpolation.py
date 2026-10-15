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

"""Draw a cell's value at its centre and the bilinear blend of four cells."""

import argparse

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    out = ap.parse_args().out

    qx, qy = 1.8, 1.3
    fx, fy = qx - 0.5 - 1, qy - 0.5 - 0  # offsets from centre (1.5, 0.5)
    pts = [(1.5, 0.5, (1 - fx) * (1 - fy)), (2.5, 0.5, fx * (1 - fy)),
           (1.5, 1.5, (1 - fx) * fy), (2.5, 1.5, fx * fy)]

    fig, ax = plt.subplots(figsize=(6, 6))
    for i in range(4):
        ax.axvline(i, color='#999999', lw=1)
        ax.axhline(i, color='#999999', lw=1)
    ax.plot([c + 0.5 for c in range(3) for _ in range(3)],
            [r + 0.5 for _ in range(3) for r in range(3)],
            's', color='#666666', ms=5, label='cell centre (value sits here)')
    for x, y, w in pts:
        ax.plot([qx, x], [qy, y], '-', color='#D55E00', lw=1)
        ax.plot(x, y, 's', color='#0072B2', ms=10)
        ax.annotate(f'w = {w:.2f}', (x, y), (x + 0.05, y - 0.2),
                    color='#0072B2', fontsize=11)
    ax.plot(qx, qy, 'o', color='#D55E00', ms=9, label='query point')
    ax.annotate('', (0.5, 0.5), (0, 0),
                arrowprops=dict(arrowstyle='->', color='#009E73', lw=2))
    ax.text(0.03, 0.62, 'sample origin\n(half a cell in)', color='#009E73',
            fontsize=10, va='bottom')
    ax.text(1.5, 2.1, 'a cell covers [i, i+1)\nin x and in y', fontsize=10,
            color='#444444', ha='center')
    ax.set_xlim(-0.2, 3.2)
    ax.set_ylim(-0.2, 3.2)
    ax.set_aspect('equal')
    ax.set_xlabel('x (cells)')
    ax.set_ylabel('y (cells)')
    ax.set_xticks(range(4))
    ax.set_yticks(range(4))
    ax.legend(loc='lower right', fontsize=9)
    fig.tight_layout()
    fig.savefig(out, dpi=150)


if __name__ == '__main__':
    main()
