# Environment

## What it runs on

- Ubuntu 22.04 and ROS 2 Humble.
- Ignition Fortress for the simulator in `robot_bringup`.
- C++17, CMake 3.8 or newer for the library (3.11 or newer with the property tests, which fetch RapidCheck), Eigen 3, OpenMP when the compiler has it. Without OpenMP the matching loop runs on one thread.
- The build uses `-Wall -Wextra -Wpedantic` on GCC and Clang and defaults to a Release build when no type is given ([`CMakeLists.txt`](../../src/slam_cpp/CMakeLists.txt)).

## What I have tested

I built and ran the tests with two compilers on Ubuntu 22.04 under WSL2:

- GCC 11.4: the `slam_cpp` package with ROS, and the core alone without ROS. The result I have on record for this compiler is older than the latest changes to the code, and I have not repeated it since.
- GCC 12: `slam_cpp` and `robot_bringup` with ROS, built in the configuration the container uses. These runs were on the host with the same compiler version as the image, not inside the container.

I have not tried Clang, another Ubuntu release or another ROS 2 distribution. I have run the system only in simulation, never on a physical robot.

## Building without ROS

`-DSLAM_BUILD_ROS=OFF` builds `slam_core` and its tests with plain CMake. No ROS or ament package is needed. It needs Eigen 3, GoogleTest and, for the property tests, `git`, because CMake fetches RapidCheck at a pinned commit. `-DSLAM_BUILD_PROPERTY_TESTS=OFF` skips RapidCheck. The `core-no-ros` job in [`ci.yml`](../../.github/workflows/ci.yml) holds the exact commands. That job also compiles each header on its own, to show that none relies on another being included first.

This split is the check that the core is free of ROS: the no-ROS build contains no ROS header. No test checks it beyond that build succeeding.

## Tests

GoogleTest, with RapidCheck for the property tests. Each test has one of the labels `unit`, `integration` or `prop`. All but the particle filter integration test also have `fast`, so `ctest -L fast` skips that one. The labels are set in [`CMakeLists.txt`](../../src/slam_cpp/CMakeLists.txt). Under ROS the suite is run by `colcon test`, which also runs the linters.

## Container

[`docker/Dockerfile`](../../docker/Dockerfile) has three stages: `base`, `ci` and `sim`.

- `base`: the pinned `ros:humble-ros-base-jammy` image.

- `ci`: the build and test environment. It starts from `base`, takes Ubuntu and ROS packages from dated snapshots, and adds GCC 12, CMake, Eigen, GoogleTest, Doxygen and the ament lint packages. Sources are mounted at `/ws/src`, not copied in.
- `sim`: `ci` plus Ignition Fortress bridge and control packages, RViz, `robot_localization` and software rendering, for `robot_bringup`. Both images build. The core builds and its tests pass in `ci`. The simulator starts headless in `sim`.

[`docker/compose.yaml`](../../docker/compose.yaml) defines a `dev` service for `ci` and a `sim` service for `sim`, with the build directories in volumes.

## CI

[`.github/workflows/ci.yml`](../../.github/workflows/ci.yml) defines four jobs, all on `ubuntu-22.04`.

- `fast`: on a push to `main` and on draft pull requests. Builds `slam_cpp` in the `ci` image and runs the tests labelled `fast`, without the property tests.
- `full`: on pull requests that are ready for review. A Release leg builds both packages and runs every test. A Debug leg builds `slam_cpp` with assertions on and runs the `unit` tests.
- `doxygen`: runs `doxygen Doxyfile`, which fails on any warning.
- `core-no-ros`: the no-ROS build above.

[`.github/workflows/image.yml`](../../.github/workflows/image.yml) builds and pushes the `ci` image when `docker/` changes. The jobs above pull that image.

Changes that touch only `docs/` or Markdown files do not trigger `ci.yml`.
