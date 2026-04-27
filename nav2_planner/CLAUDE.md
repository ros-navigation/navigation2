# nav2_planner — assistant guide

`PlannerServer` lifecycle node — hosts `nav2_core::GlobalPlanner` plugins. Not a planner itself; the algorithms live in `nav2_navfn_planner`, `nav2_smac_planner`, `nav2_theta_star_planner`.

## Responsibilities
- Loads planners by name from `planner_server.planner_plugins` via `pluginlib::ClassLoader<nav2_core::GlobalPlanner>`.
- Provides actions: `nav2_msgs/action/ComputePathToPose`, `ComputePathThroughPoses`, `ComputePathsToPoses`.
- Owns the global `Costmap2DROS` instance shared by all planner plugins.

## Lifecycle contract
- `on_configure`: declare params, instantiate plugins, configure costmap, create action servers.
- `on_activate`: activate publishers + lifecycle-activate the costmap.
- `on_deactivate` / `on_cleanup`: tear down in reverse.
- Constructor stays empty.

## Layout
- `include/nav2_planner/` — `planner_server.hpp`
- `src/` — `planner_server.cpp`, `main.cpp`
- `test/` — server-level integration tests

## How to add a new feature
- New planner action variant → add the action definition in `nav2_msgs`, add the server endpoint here, expose a BT node in `nav2_behavior_tree/plugins/action/`.
- New cross-cutting param (e.g. costmap update behavior) → add to `planner_server.ros__parameters` block in `nav2_bringup/params/nav2_params.yaml`.

## Tests
`colcon test --packages-select nav2_planner`

## Docs to update
- `docs.nav2.org` → `configuration/packages/configuring-planner-server.md`.
- Migration guide for any action signature change.

## Pitfalls
- A planner plugin throwing the wrong exception (`nav2_core/planner_exceptions.hpp`) makes `PlannerServer` report the wrong error code to the BT — be strict about exception types.
- The costmap is shared across plugins — long planner runs delay the next plugin in the same server.
- Action cancellation must propagate to plugin via `cancelChecker` — never block in `createPlan`.
