# Testing

This page says how I run the tests, what the labels select, and what CI runs. I state no test counts, because they change with every test I add. Run the commands and read what they print.

## The suite

From the workspace root, after `colcon build` ([getting-started.md](getting-started.md)):

```bash
colcon test
colcon test-result --verbose
```

`colcon test` runs the GoogleTest executables and the linters for both packages. `colcon test-result --verbose` prints the summary and the detail of each failure; `--all` lists every result file. To test one package:

```bash
colcon test --packages-select slam_cpp
colcon test-result --verbose
```

## Labels

Each test in [`src/slam_cpp/CMakeLists.txt`](../../src/slam_cpp/CMakeLists.txt) carries labels. Select by label with `--ctest-args`.

| Label | What it selects |
|---|---|
| `unit` | The example tests of one module: occupancy grid, scan matching, particle filter, the synthetic room helper, config, odometry and angle helpers. |
| `integration` | Tests that run several modules together: `scan_matching_integration_test` and `particle_filter_integration_test`. |
| `prop` | The RapidCheck property tests: scan matching, the room helper, config and odometry. |
| `fast` | Every test except `particle_filter_integration_test`. It includes the property tests. |

`particle_filter_integration_test` runs the full filter on a 24 m path, so it is the one test outside `fast`. Every test has `unit`, `integration` or `prop`, and every test but that one also has `fast`.

```bash
colcon test --packages-select slam_cpp --ctest-args -L fast
colcon test --packages-select slam_cpp --ctest-args -L unit
colcon test-result --verbose
```

## Property tests

The `prop` tests use [RapidCheck](https://github.com/emil-e/rapidcheck) with GoogleTest. They generate many random inputs and check a stated property of each result. CMake fetches RapidCheck with `git`, at the commit pinned in `CMakeLists.txt`, the first time you configure.

The tests run with a fixed seed by default: CMake sets `RC_PARAMS=seed=1` for each of them through the cache variable `SLAM_RC_PARAMS`, so two runs generate the same inputs. For a longer random run, set any RapidCheck configuration string, for example `-DSLAM_RC_PARAMS=max_success=1000`. The Release leg of the `full` CI job does this. The CTest property overrides an `RC_PARAMS` set in your shell. To reproduce a failure from a random-seed run, reconfigure with `-DSLAM_RC_PARAMS="seed=N max_success=1000"`, or run the test binary directly with `RC_PARAMS=seed=N`.

To skip them, configure with `-DSLAM_BUILD_PROPERTY_TESTS=OFF`:

```bash
colcon build --packages-select slam_cpp --cmake-args -DSLAM_BUILD_PROPERTY_TESTS=OFF
```

To build them without network access, point CMake at a local clone. Check out the commit pinned in `CMakeLists.txt` in that clone first.

```bash
colcon build --packages-select slam_cpp \
  --cmake-args -DFETCHCONTENT_SOURCE_DIR_RAPIDCHECK=/path/to/rapidcheck
```

I build with this flag and a local clone on my machine.

## The core without ROS

The core, its tests and nothing else build with plain CMake. No ROS or ament package is needed, only Eigen 3, GoogleTest and, for the property tests, `git`. Run this from the repository root:

```bash
cmake -S src/slam_cpp -B build/core -DSLAM_BUILD_ROS=OFF -DBUILD_TESTING=ON
cmake --build build/core -j2
ctest --test-dir build/core --output-on-failure
```

Add `-DSLAM_BUILD_TOOLS=ON` to also build the figure tools and their smoke tests; `src/slam_cpp/tools/README.md` says what they write.

Add `-L fast` or another label to the `ctest` line to select tests. Because this build contains no ROS header, it is also the check that the core is free of ROS. No other test checks that.

## Linters

Under ROS, `colcon test` also runs the `ament_lint_common` linters on both packages: the CMake, XML, Python, `cppcheck` and `uncrustify` checks that package provides. Both packages run every linter `ament_lint_common` provides, including the copyright check, `cpplint` and `uncrustify`; every source file carries the Apache 2.0 header. On my machine the skipped results were all `cppcheck`.

## CI

[`.github/workflows/ci.yml`](../../.github/workflows/ci.yml) defines four jobs, all on Ubuntu 22.04. The first three run in the `ci` container image ([`docker/Dockerfile`](../../docker/Dockerfile)).

| Job | When | What it runs |
|---|---|---|
| `fast` | A push to `main`, and draft pull requests | Builds `slam_cpp` in Release with the property tests off, and runs the `fast` label. |
| `full` | Pull requests that are ready for review | A Release leg builds `slam_cpp` and `robot_bringup` and runs every test. A Debug leg builds `slam_cpp` with optimisation and assertions on, with the property tests off, and runs the `unit` label. |
| `doxygen` | Every run of the workflow | Runs `doxygen Doxyfile`, which fails on any warning. |
| `core-no-ros` | Every run of the workflow | The no-ROS build above, then compiles each header alone to show that none relies on another being included first. |

A change that touches only `docs/` or Markdown files does not start `ci.yml`. The `fast` job's property-test switch means the `fast` label alone does not exercise RapidCheck in CI; the Release leg of `full` (which also passes `-DSLAM_RC_PARAMS=max_success=1000`) and the `core-no-ros` job do.

## Doxygen

The API pages come from the comments in the public headers and the node. Doxygen needs `graphviz` too, because `Doxyfile` turns on class diagrams. This works from the repository root on a host with both installed. It does not work in the compose container, which mounts only `src`.

```bash
mkdir -p build && doxygen Doxyfile
```

The output goes under `build/docs`. The run fails on any warning, including a public symbol without a comment.
