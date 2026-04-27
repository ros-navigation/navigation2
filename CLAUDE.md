# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Overview

This is **Nav2**, the ROS 2 navigation stack — a metapackage of ~35+ ROS 2 packages (`nav2_*`, `opennav_*`) that together provide localization, planning, control, behaviors, recovery, costmaps, BT-driven orchestration, and waypoint following for mobile robots. It is built with `ament_cmake` and consumed via `colcon` from a parent ROS 2 workspace (this repo lives at `<ws>/src/navigation2`).

The `navigation2/` package itself is just a metapackage (see `navigation2/package.xml`); real code lives in the sibling `nav2_*` directories.

## Build, Test, Lint

All commands run from the **workspace root** (`<ws>/`, the parent of this repo), not from inside `navigation2/`. They assume a sourced ROS 2 distro (e.g. `source /opt/ros/$ROS_DISTRO/setup.bash`).

```bash
# Build the whole stack (or pass --packages-up-to <pkg> to scope it)
colcon build --symlink-install

# Build a single package and its deps
colcon build --packages-up-to nav2_smac_planner --symlink-install

# After building, source the overlay before running tests/nodes
source install/setup.bash

# Run the canonical test suite (skips flaky system tests, runs only linters in nav2_system_tests)
src/navigation2/tools/run_test_suite.bash

# Test a single package
colcon test --packages-select nav2_smac_planner
colcon test-result --verbose --test-result-base build/nav2_smac_planner

# Run a specific gtest binary directly (faster iteration than colcon test)
./build/nav2_smac_planner/test_smac_2d
# or via ctest with a regex:
ctest --test-dir build/nav2_smac_planner -R test_smac_2d -V

# Linters only (ament_cpplint, ament_uncrustify, ament_lint_cmake, etc. are wired via package.xml)
colcon test --packages-select <pkg> --ctest-args -R "lint|cpplint|uncrustify|cmake"
```

System tests in `nav2_system_tests` are flaky / heavyweight and are intentionally **skipped** by `run_test_suite.bash` except for `test_dynamic_obstacle` (run with retries via `tools/ctest_retry.bash`). When running locally, expect them to need a working Gazebo/sim setup.

`tools/skip_keys.txt` is consumed by CI rosdep to skip non-ROS keys; reuse it locally with `rosdep install --from-paths src --ignore-src -r -y --skip-keys "$(cat src/navigation2/tools/skip_keys.txt)"`.

## Architecture — the big picture

Nav2 is plumbed together at runtime as a set of **lifecycle nodes** orchestrated by `nav2_lifecycle_manager`, with high-level navigation logic expressed as **Behavior Trees** rather than hardcoded state machines. Nearly every algorithmic component is a **pluginlib plugin** loaded by name from a YAML param file. The patterns matter more than any single file:

- **`nav2_core`** — pure abstract interfaces (`GlobalPlanner`, `Controller`, `Smoother`, `Behavior`, `ProgressChecker`, `GoalChecker`, costmap layers, route plugins, etc.). Every algorithm package implements one of these and exports a pluginlib XML. To add an algorithm, implement the interface, export the plugin, register it in the relevant server's YAML.
- **Servers (lifecycle nodes) that host plugins**:
  - `nav2_planner` (`PlannerServer`) hosts `GlobalPlanner` plugins → `nav2_navfn_planner`, `nav2_smac_planner`, `nav2_theta_star_planner`.
  - `nav2_controller` (`ControllerServer`) hosts `Controller` plugins → `nav2_dwb_controller`, `nav2_mppi_controller`, `nav2_regulated_pure_pursuit_controller`, `nav2_graceful_controller`, `nav2_rotation_shim_controller`.
  - `nav2_behaviors` hosts recovery `Behavior` plugins (spin, backup, wait, drive_on_heading, assisted_teleop).
  - `nav2_smoother` hosts `Smoother` plugins → `nav2_smoother`, `nav2_constrained_smoother`, smac smoother.
  - `nav2_costmap_2d` hosts `Layer` plugins (static, obstacle, voxel, inflation, range, denoise, plugin-container) and `LayeredCostmap`. Most servers each own one or more `Costmap2DROS` instances.
  - `nav2_bt_navigator` runs the high-level Behavior Tree(s) — XML files in `nav2_bt_navigator/behavior_trees/` — built from BT nodes in `nav2_behavior_tree` (action clients to the servers above, plus condition/control/decorator nodes).
  - `nav2_route` is the higher-level graph/route planner (loads GeoJSON graphs from `nav2_route/graphs/`).
  - `nav2_waypoint_follower`, `nav2_collision_monitor`, `nav2_velocity_smoother`, `nav2_docking` (`opennav_docking`), `nav2_following` (`opennav_following`) are additional lifecycle nodes.
- **`nav2_lifecycle_manager`** drives the configure→activate→deactivate→cleanup transitions of all the servers in a configured order. Anything that misbehaves on transitions will hang the whole stack.
- **`nav2_bringup`** — the canonical entry point. `launch/` and `params/nav2_params.yaml` show how all the pieces are wired together; read these first when tracking down "where does X get configured".
- **Shared infrastructure**:
  - `nav2_util` — lifecycle node base class, service clients, geometry utils, robot utils. Used everywhere.
  - `nav2_ros_common` — wrappers/typedefs over `rclcpp` used by lifecycle servers (publishers, subscriptions, action servers/clients).
  - `nav2_msgs` — all custom msgs/srvs/actions (`NavigateToPose`, `ComputePathToPose`, `FollowPath`, etc.).
  - `nav2_common` — Python build helpers and CMake macros (`nav2_package()` etc.) used in nearly every `CMakeLists.txt`.
  - `nav2_voxel_grid` — voxel data structure used by costmap voxel layer.
  - `nav2_simple_commander` — Python API on top of the action interfaces; useful for examples and integration tests but not part of the runtime data path.

### Cross-cutting conventions worth knowing before editing

- **Lifecycle, not raw rclcpp**: servers derive from `nav2_util::LifecycleNode`. Publishers/subscribers/timers must be created in `on_configure` / `on_activate`, torn down in `on_deactivate` / `on_cleanup`. Don't allocate ROS entities in constructors.
- **Time source**: code uses **ROS time** (sim-time aware). Recent commit (`cd75eae`) moved clocks off `system_clock`. When comparing times, prefer `rclcpp::Time` consistently — see commit `4f8677c` for the disambiguation pattern between `rclcpp::Time` and `builtin_interfaces::msg::Time`. Don't mix.
- **Plugin registration**: every plugin needs (a) `PLUGINLIB_EXPORT_CLASS` in the .cpp, (b) a `<plugin>` XML file (e.g. `*_plugin.xml`), (c) `pluginlib_export_plugin_description_file()` in `CMakeLists.txt`, (d) `<export><nav2_core plugin="..."/></export>` in `package.xml`. Missing any one of these silently breaks loading at runtime.
- **CMake style**: packages call `nav2_package()` from `nav2_common` to apply the project-wide compile flags / C++17 settings. Don't reinvent flag handling.
- **Linting**: ament_cpplint + ament_uncrustify are enforced by CI. Run `colcon test` on a package before pushing — uncrustify violations are common when manually editing.
- **DCO sign-off**: per `CONTRIBUTING.md`, every commit must include a `Signed-off-by:` line (use `git commit -s`). PRs without it will fail the DCO check.

### Where to look for things

- "How is the stack launched and parameterized end-to-end?" → `nav2_bringup/launch/` + `nav2_bringup/params/nav2_params.yaml`.
- "What's the high-level navigate-to-pose flow?" → BT XMLs in `nav2_bt_navigator/behavior_trees/` + BT action nodes in `nav2_behavior_tree/plugins/action/`.
- "How do I add a new planner/controller/behavior/layer?" → find the closest existing plugin in the matching package, copy its `package.xml` exports, plugin XML, and CMake stanza; implement the `nav2_core` interface.
- "Where's the test for X?" → each package has its own `test/` directory; system-wide integration tests live in `nav2_system_tests`.

## Distros and CI

Targets ROS 2 **Humble**, **Jazzy**, and **Kilted** (see badge table in `README.md`). `nav2_route` is Kilted-only. CI runs via GitHub Actions (`.github/workflows/`) and CircleCI; the Dockerfile in this repo is the canonical reproducible build environment (`docker build -t nav2:latest ./`).

## Reference docs

Authoritative user-facing docs live at <https://docs.nav2.org/> — configuration, plugin tutorials, migration guides. API reference: <https://api.nav2.org/>. When a question is about *how to use* (rather than how the code is organized), prefer those over inferring from source.
