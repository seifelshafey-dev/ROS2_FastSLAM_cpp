# Design

These pages give the reasoning behind the code: the formulas, what each symbol is in the code, the units and frames, and the choices I made where the code could have done something else. The pages in [architecture](../architecture/overview.md) say what the code does now. These say why, and how I checked it.

## Pages

- [grid.md](grid.md): how a point becomes a cell, what a cell stores, and why a cell's value sits at its centre when the matcher reads the map.
- [scan-matching.md](scan-matching.md): the cost of a pose, the coarse search, the Levenberg-Marquardt step and the proposal covariance.
- [motion-model.md](motion-model.md): the turn, move, turn model, the size of the noise and how the covariance is carried.
- [weights-resampling.md](weights-resampling.md): the particle weight, the effective sample size, resampling, the best particle and copy on write.
- [map-odom.md](map-odom.md): how odometry is split into controls and how the `map` to `odom` transform is built from one moment.
- [scan-density.md](scan-density.md): why a scan with too few points on a wall breaks the first matches, with the measurements, what I changed and what is still open.

## How each page is built

Each page opens with what it settles. Each section gives formulas in plain text, with the code name of every symbol, its unit and its frame. Where a choice could go two ways, I say which: floor or round, cell corner or cell centre, inclusive or exclusive. Where the code differs from a book or a paper, I say how. I do not claim the two are equivalent.

Each section ends in one of two ways. Either it names the test that checks it, as `Suite.test` with the file, or it says that no test checks it and that it rests on the argument above. I opened each test I name. A test name is only as good as what the test asserts, so where a test checks less than the section claims, I say so.

Numbers on these pages are ones I measured and recorded. I give them with their conditions. Most are from offline runs on a synthetic room, a few from the simulator. I have not run any of this on a physical robot. I mark a statement as my reading when I did not test it.

Where I describe Hector SLAM, GMapping, FastSLAM 2.0 or Probabilistic Robotics, I describe what I understood from the papers and the book. I did not re-check equation numbers when I wrote these pages, so I give none.

## Where to go next

- [../architecture/limits.md](../architecture/limits.md) lists what the package does not do, with measurements.
- [../architecture/config.md](../architecture/config.md) lists every setting and what it trades.
- [../architecture/core.md](../architecture/core.md) is the short description of the core that these pages derive.
