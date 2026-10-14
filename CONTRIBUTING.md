# Contributing

Issues and pull requests are welcome.

## Reporting a problem

Open an issue. Include:

- your ROS distro and Ubuntu version;
- how you built it (`colcon build`, the container, or the no-ROS build) and the compiler;
- the command you ran;
- the output you got and the output you expected;
- the commit you built.

For a change, describe what you want and why before you write it.

## Before a pull request

1. Build the workspace ([getting started](docs/guide/getting-started.md)).
2. Run `colcon test` and `colcon test-result --verbose`. The linters run with it and must be green ([testing](docs/guide/testing.md)).
3. Run the core without ROS: `cmake -S src/slam_cpp -B build/core -DSLAM_BUILD_ROS=OFF -DBUILD_TESTING=ON`, then `cmake --build build/core -j2` and `ctest --test-dir build/core --output-on-failure`.
4. Run `mkdir -p build && doxygen Doxyfile` from the repository root. It must print no warning.
5. Add or update a test with any code change. Keep the pull request to one topic.

I review every pull request. CI must be green before I merge.

## Conventions

1. C++17. A signature in the header and in the source match exactly, `const` included. Return by value, never a reference to a local.
2. Keep the core free of ROS. Only `src/slam_cpp/src/ros/slam_node.cpp` includes a ROS header. The no-ROS build above is the check.
3. Validate every config field with `SLAM_CHECK` in `validate` (`src/slam_cpp/src/slam_config.cpp`). A bad value then fails at start-up with a message naming the field. A new field gets a rule and a test in the same change.
4. Do not hand-set a number that depends on another setting. A value that follows from the sensor, the grid or another setting is computed in code. A relation between two settings is a `SLAM_CHECK` where both are known. A real choice stays in config as a ratio or a count of cells.
5. Every public symbol has a Doxygen comment written for the caller: what you pass, what you get back including the failure value, the constraints, and units and frames. No history and no notes on how the code works inside.
6. Tests use GoogleTest, and RapidCheck for property tests ([testing](docs/guide/testing.md) lists the labels).

## Commit subjects

Use `type(scope): subject`, in lower case and the imperative:

- `fix(scan_matching): read cell values at cell centres`
- `test(slam_config): keep the copy of the shipped values in step with the parameter file`
- `docs: architecture, design and guide pages`

## Licence

Contributions are licensed under Apache 2.0, the licence of the project. See [LICENSE](LICENSE).
