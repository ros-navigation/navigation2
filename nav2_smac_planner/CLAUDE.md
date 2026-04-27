# nav2_smac_planner — assistant guide

Implements `nav2_core::GlobalPlanner`. Ships three planner variants: 2D (4/8-connected A*), Hybrid-A* (SE2, kinematically feasible), and State Lattice. Also provides a smoother used by `nav2_smoother`.

## Interface
`nav2_core::GlobalPlanner` — `nav2_core/include/nav2_core/global_planner.hpp`. Methods: `configure`, `activate`, `deactivate`, `cleanup`, `createPlan`. All called from the host (`nav2_planner::PlannerServer`).

## Plugin XMLs (one per variant)
- `smac_plugin_2d.xml` → `nav2_smac_planner::SmacPlanner2D`
- `smac_plugin_hybrid.xml` → `nav2_smac_planner::SmacPlannerHybrid`
- `smac_plugin_lattice.xml` → `nav2_smac_planner::SmacPlannerLattice`
All exported via `pluginlib_export_plugin_description_file()` in `CMakeLists.txt` and `<export>` in `package.xml`.

## Layout
- `include/nav2_smac_planner/` — `a_star.hpp`, `node_2d.hpp`, `node_hybrid.hpp`, `node_lattice.hpp`, `collision_checker.hpp`, `smoother.hpp`, `costmap_downsampler.hpp`, etc.
- `src/` — variant `.cpp`s plus `a_star.cpp`, `smoother.cpp`, collision checking
- `lattice_primitives/` — Python script + JSON primitive sets for the State Lattice variant
- `test/` — gtest binaries (`test_smac_2d`, `test_smac_hybrid`, `test_smac_lattice`, `test_collision_checker`, `test_smoother`)

## How to add a new variant / heuristic
1. Add `node_<name>.hpp` modeling state + neighbor expansion; reuse `GridCollisionChecker`.
2. Implement the planner class in `include/nav2_smac_planner/smac_planner_<name>.hpp` + `.cpp`.
3. `PLUGINLIB_EXPORT_CLASS(nav2_smac_planner::SmacPlanner<Name>, nav2_core::GlobalPlanner)`.
4. Add `smac_plugin_<name>.xml`; register it in `CMakeLists.txt` and `package.xml`.
5. Register the new plugin name + params under `planner_server.ros__parameters` in `nav2_bringup/params/nav2_params.yaml`.

## Tests
`./build/nav2_smac_planner/test_smac_<2d|hybrid|lattice>` for fast iteration. Full suite: `colcon test --packages-select nav2_smac_planner`.

## Docs to update
- `docs.nav2.org` → `configuration/packages/configuring-smac-planner.md` for any param change.
- Tuning guide for changes to default heuristics, motion-primitive sets, or analytical-expansion logic.
- Migration guide for any breaking change to the search graph or plugin name.

## Pitfalls
- `createPlan` runs under `PlannerServer`'s timeout — long searches need cooperative early-exit on `cancelChecker`.
- `GridCollisionChecker` caches inflation footprint by orientation bins — invalidate on costmap reconfigure.
- Hybrid/lattice analytic-expansion (Reeds-Shepp / Dubins) is the usual hot-spot when tuning.
- 2D planner uses `Node2D` with A*; Hybrid and Lattice use `NodeHybrid` / `NodeLattice` with SE2 — don't cross-wire heuristics.
