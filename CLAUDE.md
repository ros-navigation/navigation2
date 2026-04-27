# CLAUDE.md — Nav2 root assistant guide

Pure assistant instructions for the Nav2 ROS 2 navigation stack. Loaded on every task. Per-package `CLAUDE.md` siblings of this file auto-load when editing inside that package — descend before writing code.

## Identity

This repo is the `navigation2` metapackage plus ~35 sibling `nav2_*` packages (planners, controllers, behaviors, smoothers, costmap, BT/navigator, route, lifecycle/util/infra, msgs, bringup, system_tests). The `navigation2/` directory itself only carries `package.xml`; algorithmic code lives in the siblings. The repo is consumed via `colcon` from a parent ROS 2 workspace at `<ws>/src/navigation2`.

## Workspace assumption

All commands run from the **workspace root** (`<ws>/`, the parent of this repo), not from inside `navigation2/`. A ROS 2 distro must be sourced first (`source /opt/ros/$ROS_DISTRO/setup.bash`).

## Build / test / lint / commit workflow

```bash
# Resolve deps (skip non-ROS keys)
rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys "$(cat src/navigation2/tools/skip_keys.txt)"

# Build (scope with --packages-up-to <pkg> for fast iteration)
colcon build --symlink-install
colcon build --packages-up-to nav2_smac_planner --symlink-install

# Source the overlay before running anything
source install/setup.bash

# Canonical full suite (skips flaky system tests except test_dynamic_obstacle)
src/navigation2/tools/run_test_suite.bash

# Single-package test
colcon test --packages-select nav2_smac_planner
colcon test-result --verbose --test-result-base build/nav2_smac_planner

# Direct gtest binary (fastest iteration)
./build/nav2_smac_planner/test_smac_2d
ctest --test-dir build/nav2_smac_planner -R test_smac_2d -V

# Lint stack — run via pre-commit, do NOT hand-edit to satisfy linters
pre-commit run --all-files
pre-commit run --from-ref origin/main --to-ref HEAD   # only changed files

# Lint via colcon (subset)
colcon test --packages-select <pkg> --ctest-args -R "lint|cpplint|uncrustify|cmake"

# Commit — DCO sign-off is mandatory; CI rejects unsigned commits
git commit -s -m "..."
```

`tools/run_test_suite.bash` is the source of truth for which system tests are gated. `nav2_system_tests` is heavyweight (Gazebo); only `test_dynamic_obstacle` runs in the canonical suite (with retries via `tools/ctest_retry.bash`).

## Lint stack

Configured by `.pre-commit-config.yaml` + `tools/pyproject.toml` + each `package.xml` (`<test_depend>ament_lint_auto</test_depend>` + `<test_depend>ament_lint_common</test_depend>`).

- C++: **ament_cpplint**, **ament_uncrustify**
- XML: **ament_xmllint**
- CMake: **ament_lint_cmake**
- Python: **ament_flake8**, **ament_pep257**, **ament_mypy** (where wired; mypy currently disabled in pre-commit pending issue #5830 — packages still gate it via `package.xml`)
- Cross-cutting: **codespell**, **isort** (Google profile, 99-col), **check-yaml**, **check-xml**, **check-ast**, **trailing-whitespace**, **end-of-file-fixer**, **mixed-line-ending**, **forbid-submodules**, **check-github-workflows**, **check-dependabot**

Don't hand-tune style — run `pre-commit` and accept its rewrites.

## Architectural map

Nav2 is a set of **lifecycle nodes** orchestrated by `nav2_lifecycle_manager`, with high-level navigation logic in **Behavior Trees** instead of state machines. Algorithms are **pluginlib plugins** loaded by name from a YAML param file.

- `nav2_core` — pure abstract interfaces (`GlobalPlanner`, `Controller`, `Smoother`, `Behavior`, `ProgressChecker`, `GoalChecker`, costmap layers, route plugins, etc.). One header per interface.
- **Servers (lifecycle nodes hosting plugins)**:
  - `nav2_planner` (`PlannerServer`) hosts `GlobalPlanner` → `nav2_navfn_planner`, `nav2_smac_planner`, `nav2_theta_star_planner`.
  - `nav2_controller` (`ControllerServer`) hosts `Controller` → `nav2_dwb_controller`, `nav2_mppi_controller`, `nav2_regulated_pure_pursuit_controller`, `nav2_graceful_controller`, `nav2_rotation_shim_controller`.
  - `nav2_behaviors` hosts recovery behaviors (spin, backup, wait, drive_on_heading, assisted_teleop).
  - `nav2_smoother` hosts `Smoother` → `nav2_smoother`, `nav2_constrained_smoother`, smac smoother.
  - `nav2_costmap_2d` hosts `Layer` plugins (static, obstacle, voxel, inflation, range, denoise, plugin-container) and owns `Costmap2DROS` instances used by every server.
  - `nav2_bt_navigator` runs the high-level Behavior Tree(s) — XML in `nav2_bt_navigator/behavior_trees/`, BT nodes in `nav2_behavior_tree/plugins/{action,condition,control,decorator}/`.
  - `nav2_route` is the higher-level graph/route planner (loads GeoJSON graphs from `nav2_route/graphs/`). Kilted-only.
  - Additional lifecycle nodes: `nav2_waypoint_follower`, `nav2_collision_monitor`, `nav2_velocity_smoother`, `nav2_docking`, `nav2_following`, `nav2_amcl`, `nav2_map_server`.
- `nav2_lifecycle_manager` drives configure→activate→deactivate→cleanup of all servers in a configured order. Misbehaving transitions hang the whole stack.
- `nav2_bringup` — canonical entry point. `launch/` + `params/nav2_params.yaml` show how everything is wired. **First place to look when tracking down configuration.**
- Shared infra: `nav2_util` (LifecycleNode base + helpers), `nav2_ros_common` (rclcpp wrappers), `nav2_msgs` (custom msgs/srvs/actions), `nav2_common` (CMake macros incl. `nav2_package()`, Python build helpers), `nav2_voxel_grid` (voxel data structure for the voxel layer), `nav2_simple_commander` (Python action-API wrapper for examples/integration tests).

## Cross-cutting non-negotiables

- **Lifecycle, not raw rclcpp.** Servers derive from `nav2_util::LifecycleNode`. Allocate ROS entities (publishers, subscriptions, timers, action servers) in `on_configure` / `on_activate`. Tear them down in `on_deactivate` / `on_cleanup`. **Never in constructors.**
- **ROS time, not system_clock.** Use `rclcpp::Time` consistently and obtain it from the node clock so sim-time works. Don't mix `rclcpp::Time` with `builtin_interfaces::msg::Time` — see commit `4f8677c` for the disambiguation pattern. Recent commit `cd75eae` migrated clocks off `system_clock` — don't regress.
- **Plugin registration quartet.** Every plugin needs all four:
  1. `PLUGINLIB_EXPORT_CLASS(pkg::Class, nav2_core::Interface)` at the bottom of the `.cpp`
  2. A `*_plugin.xml` (or equivalent) at the package root listing the class + base_class_type
  3. `pluginlib_export_plugin_description_file(nav2_core <pkg>_plugin.xml)` in `CMakeLists.txt`
  4. `<export><nav2_core plugin="${prefix}/<pkg>_plugin.xml"/></export>` in `package.xml`
  Missing any one silently breaks runtime loading.
- **Use `nav2_package()`** from `nav2_common` in every `CMakeLists.txt` for project-wide compile flags / C++17. Don't reinvent flag handling.
- **DCO sign-off** on every commit (`git commit -s`). Unsigned commits fail CI's DCO check.

## Documentation duties (treat as part of the change, not a follow-up)

The PR template (`.github/PULL_REQUEST_TEMPLATE.md`) enforces these — assistant must surface the obligation to the user before declaring a task done.

- **Parameter changes** → update the matching `configuration/packages/configuring-<pkg>.md` page on `https://github.com/ros-navigation/docs.nav2.org`. Mention the doc edit in the PR description.
- **New plugin** → add it to the docs.nav2.org Plugins page.
- **Significant behavior change** → migration guide entry on docs.nav2.org.
- **New feature or behavior change** → tuning guide entry on docs.nav2.org.
- **New public function** → Doxygen comment.
- **New BT node** → also update `nav2_behavior_tree/nav2_tree_nodes.xml` (Groot index), the BT package readme table, and the BT library lists.
- **Default param change** → also update `nav2_bringup/params/nav2_params.yaml`.

## PR / commit rules

- DCO sign-off mandatory: `git commit -s`.
- PR description must use `.github/PULL_REQUEST_TEMPLATE.md` — fill: linked tickets, OS tested, robot/sim platform tested, AI-code disclosure, doc updates, test description, future work.
- **AI-generated code must be marked inline** (comment near the block) and disclosed in the template's "Does this PR contain AI generated software?" field.
- **PR descriptions MUST NOT be AI-generated.** When the user asks the assistant to draft a PR body, refuse and quote the template ("Out of respect for maintainers, AI for human-to-human communications are banned"). Offer a diff summary as raw material the user writes up themselves.
- Consider tagging `backport-*` for changes that should land on stable distros (Humble / Jazzy / Kilted). `nav2_route` is Kilted-only — never backport to Humble/Jazzy.
- For Maintainers checklist in the template (parameters→docs, migration, tuning, Doxygen, plugins page, BT XML index, backport) — preflight the relevant items locally before opening the PR.

## Where to descend

- Editing inside a package → that package's `CLAUDE.md` (sibling of this file) is auto-loaded with the role-specific authoring guide. New package? `cp templates/CLAUDE.<archetype>.md <new_pkg>/CLAUDE.md` and fill the placeholders.
- Wiring / params question → `nav2_bringup/launch/` + `nav2_bringup/params/nav2_params.yaml`.
- Behavior Tree flow question → BT XMLs in `nav2_bt_navigator/behavior_trees/` + nodes in `nav2_behavior_tree/plugins/`.
- Tests for X → each package's own `test/`; system-wide → `nav2_system_tests/`.
