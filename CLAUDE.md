# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Nav2 is the official ROS 2 navigation framework, built on a **plugin-based architecture** using `pluginlib` and orchestrated by **BehaviorTree.CPP** behavior trees. All components are ROS 2 lifecycle nodes.

## Build Commands

```bash
# Build with colcon (from workspace root)
colcon build --symlink-install

# Build a single package
colcon build --packages-select nav2_costmap_2d

# Build with specific ROS distro sourcing
source /opt/ros/rolling/setup.bash && colcon build --symlink-install

# Enable code coverage
colcon build --cmake-args -DCOVERAGE_ENABLED=ON
```

The `nav2_package()` cmake macro in `nav2_common/cmake/nav2_package.cmake` sets C++20, `-Wall -Wextra -Wpedantic -Werror` (treated as errors), and optional coverage flags.

## Test Commands

```bash
# Run all tests for a package
colcon test --packages-select nav2_costmap_2d

# Run a single C++ test by name (after building)
cd build/nav2_costmap_2d && ctest -R <test_name> -V

# Run system tests (require ROS networking)
colcon test --packages-select nav2_system_tests

# See test results
colcon test-result --verbose
```

Tests use `nav2_add_gtest()`, `nav2_add_pytest_test()`, `nav2_add_gmock()` from `nav2_common/cmake/nav2_package.cmake`. Each package has its own `test/` directory with unit, integration, and regression subdirs.

## Lint Commands

```bash
# Run all pre-commit checks
pre-commit run -a

# Run specific linters (ament_* tools)
ament_cpplint
ament_uncrustify
ament_flake8
ament_pep257
ament_xmllint

# Install pre-commit hooks
pre-commit install
```

CI runs these via `pre-commit` + `ament_lint_general` matrix in `.github/workflows/lint.yml`. mypy config is in `tools/pyproject.toml`.

## Architecture

### Plugin Interfaces (nav2_core)

All navigation algorithms implement interfaces defined in `nav2_core/include/nav2_core/`:

- **Controller** — computes velocity commands from a path
- **GlobalPlanner** — plans a path from start to goal on the costmap
- **Smoother** — smooths planned paths
- **GoalChecker** — determines if a goal has been reached
- **ProgressChecker** — detects if the robot is making progress
- **WaypointTaskExecutor** — handles waypoint-following task logic
- **NavigatorBase** — interfaces for BT navigator plugins

Plugins are loaded via `pluginlib::ClassLoader` with XML plugin descriptors (e.g., `plugins.xml` in each package).

### Orchestration (nav2_bt_navigator)

`BtNavigator` (`nav2_bt_navigator`) is the central node. It:
1. Loads behavior tree XML and BT node plugin libraries via `nav2_behavior_tree::BehaviorTreeEngine`
2. Exposes ROS 2 action servers (`NavigateToPose`, `NavigateThroughPoses`, etc.)
3. Ticks the BT each cycle to orchestrate planning → smoothing → controlling → recovery

### Behavior Tree (nav2_behavior_tree)

BT XMLs are in `nav2_bringup/graphs/` (JSON graph format) and compiled into XML at runtime. BT node plugins live in `nav2_behavior_tree/plugins/` organized by type:

- **actions/** — 40+ BT action nodes (compute path, follow path, spin, back up, etc.)
- **conditions/** — 20+ BT condition nodes (goal reached, is stuck, battery low, etc.)
- **controls/** — 6 control flow nodes (recovery, pipeline sequence, round robin, etc.)
- **decorators/** — 7 decorator nodes (rate controller, speed controller, goal updater, etc.)

Custom BT nodes are registered via the `BT_BUILTIN_PLUGINS` macro in `nav2_behavior_tree/plugins_list.hpp`.

### Key Package Roles

| Package | Role |
|---------|------|
| `nav2_costmap_2d` | 2D costmap layers (static, obstacle, inflation, etc.) |
| `nav2_smac_planner` | Hybrid-A*, State Lattice, 2D A* planners |
| `nav2_navfn_planner` | NavFn-based global planner |
| `nav2_regulated_pure_pursuit_controller` | RPP path tracking controller |
| `nav2_mppi_controller` | Model Predictive Path Integral controller |
| `nav2_dwb_controller` | DWB local planner framework |
| `nav2_behaviors` | Recovery behaviors (back up, spin, wait, assisted teleop, etc.) |
| `nav2_amcl` | Adaptive Monte Carlo Localization |
| `nav2_collision_monitor` | Safety collision detection (polygons, circles, pointclouds, scans) |
| `nav2_smoother` + `nav2_constrained_smoother` | Path smoothing (simple, Savitzky-Golay, constrained optimization) |
| `nav2_bringup` | Launch files, `nav2_params.yaml`, maps, rviz configs, BT graphs |
| `nav2_util` | Shared utilities (path utils, geometry, costmap helpers, lifecycle client) |
| `nav2_simple_commander` | Python API (`BasicNavigator`) for programmatic Nav2 usage |
| `nav2_lifecycle_manager` | Brings up/down lifecycle nodes in order |

### Controllers Sub-Architecture

`nav2_controller` is the controller wrapper node. It loads a controller plugin and handles the action server for `FollowPath`. Controller plugins live in separate packages:
- `nav2_regulated_pure_pursuit_controller`
- `nav2_mppi_controller`
- `nav2_dwb_controller`
- `nav2_rotation_shim_controller`
- `nav2_graceful_controller`

`nav2_rotation_shim_controller` wraps another controller to add "rotate in place first" behavior. The `controller_selector_node` BT action can dynamically switch controllers at runtime.

### Parameter Handling

`nav2_ros_common/lifecycle_node.hpp` provides `nav2::LifecycleNode` which extends `rclcpp_lifecycle::LifecycleNode` with `declare_or_get_parameter()` and other Nav2-specific utilities. Parameters cascade from `nav2_params.yaml` loaded by the bringup launch.

## Commit Conventions

- DCO required: every commit must include `Signed-off-by: ...` line
- Follow existing commit message style: lowercase, imperative mood, package prefix (e.g., `[smac] fix heuristic lookup`)
