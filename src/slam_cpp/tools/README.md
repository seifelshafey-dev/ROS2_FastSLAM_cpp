# Tools

Programs that produce the figures in `docs/media/` and the accuracy numbers. None is installed with the package.

## What is here

- `cost_landscape_dump`, `motion_cloud_dump`, `filter_run_dump` (C++, no ROS): write CSV files from the core library. Built only with `-DSLAM_BUILD_TOOLS=ON`.
- `viz/`: Python scripts (numpy and matplotlib) that draw a PNG from those CSV files, plus `plot_grid_interpolation.py` and `plot_frames.py`, which need no input, and `overview.dot`, a Graphviz file.
- `accuracy/`: the recording and scoring tools for the accuracy page. `record.sh <bag>` records one simulator run of `drive.py`; `evaluate.py <bag>` scores it against the simulator's ground truth, and draws the trajectory and error figures. With `--map-gif` and `--map-png` it also draws the demo GIF and the map with the true walls.

## Build the dump tools

In a colcon build from the workspace root:

    colcon build --packages-select slam_cpp --cmake-args -DSLAM_BUILD_TOOLS=ON

The binaries are then in `build/slam_cpp/tools/`. Without ROS, configure `src/slam_cpp` directly with `-DSLAM_BUILD_ROS=OFF -DSLAM_BUILD_TOOLS=ON`; the binaries are in `tools/` under that build directory.

## Redraw the figures

Run everything from the workspace root (the directory that holds `build/` and `docs/`). `B` below is `build/slam_cpp/tools`, `V` is `src/slam_cpp/tools/viz`. Set `V=src/slam_cpp/tools/viz` and `B=build/slam_cpp/tools` in the shell first.

    mkdir -p docs/media /tmp/figdata
    dot -Tpng -Gdpi=150 $V/overview.dot -o docs/media/overview.png
    python3 $V/plot_grid_interpolation.py --out docs/media/grid-interpolation.png
    python3 $V/plot_frames.py --out docs/media/frames.png

    $B/cost_landscape_dump /tmp/figdata/cost.csv
    python3 $V/plot_cost_landscape.py /tmp/figdata/cost.csv --out docs/media/cost-landscape.png

    $B/motion_cloud_dump /tmp/figdata/cloud.csv
    python3 $V/plot_motion_cloud.py /tmp/figdata/cloud.csv --out docs/media/motion-cloud.png

    $B/filter_run_dump /tmp/figdata/run.csv
    python3 $V/plot_filter_run.py /tmp/figdata/run.csv --labels "this run" --neff --out docs/media/filter-run.png


For two runs in one figure (for example 180 and 360 points), pass both CSV files and `--labels`:

    $B/filter_run_dump /tmp/figdata/b2.csv --res 0.05 --beam-step 2
    $B/filter_run_dump /tmp/figdata/b1.csv --res 0.05 --beam-step 1
    python3 $V/plot_filter_run.py /tmp/figdata/b2.csv /tmp/figdata/b1.csv --labels "180 points" "360 points" --out docs/media/scan-density.png

`plot_filter_run.py --particles N` sets the N/2 line (default 80). Do not run the dump tools while a simulator is running.

The accuracy figures, the demo and the map with the true walls come from live run 1, the `/slam/best_pose` and map messages recorded in its bag. Record three runs into a directory outside the repository (run from the repository root, workspace built and sourced). `record.sh` refuses a bag directory that already exists, so each run needs a new name. It stops everything it started on Ctrl-C and exits non-zero if the drive or the recorder failed:

    mkdir -p ~/slam_bags
    src/slam_cpp/tools/accuracy/record.sh ~/slam_bags/run1
    src/slam_cpp/tools/accuracy/record.sh ~/slam_bags/run2
    src/slam_cpp/tools/accuracy/record.sh ~/slam_bags/run3
    python3 src/slam_cpp/tools/accuracy/evaluate.py ~/slam_bags/run1 --out-prefix docs/media/accuracy --map-gif docs/media/demo.gif --map-png docs/media/map-walls.png

To test a change on the same inputs without the simulator, add `--replay`: it plays the recorded `/scan` and `/odometry/filtered` into a fresh node and scores that.
