# nav2_dwb_controller — assistant guide

Composite metapackage for the Dynamic Window Approach controller. Implements `nav2_core::Controller`. Subpackages:

- `dwb_core` — controller skeleton + plugin base classes (`TrajectoryGenerator`, `Goal/Critic`, `TrajectoryCritic`)
- `dwb_critics` — built-in critics (Obstacle, PathAlign, PathDist, GoalAlign, GoalDist, etc.)
- `dwb_plugins` — built-in trajectory generators (StandardTrajectoryGenerator, LimitedAccelGenerator) + goal checkers
- `nav_2d_msgs` — 2D pose/twist message types
- `nav_2d_utils` — helpers (parameter handling, conversions)
- `costmap_queue` — BFS queue used by costmap-aware critics

## Interface
`nav2_core::Controller` (`dwb_core::DWBLocalPlanner`) — hosted by `nav2_controller::ControllerServer`.

## Plugin XMLs
- `dwb_core/local_planner_plugin.xml` → `dwb_core::DWBLocalPlanner`
- `dwb_plugins/plugins.xml` → trajectory generators + goal checkers
- `dwb_critics/<name>.xml` → individual critic plugins
- Critics are their own pluginlib base (`dwb_core::TrajectoryCritic`) — separate registration from controllers.

## Layout
Each subpackage is a full nav2 package: `include/`, `src/`, `test/`, its own `CMakeLists.txt` + `package.xml`. Treat them independently.

## How to add a new critic / generator
1. Derive from `dwb_core::TrajectoryCritic` (or `TrajectoryGenerator` / `GoalChecker`).
2. `PLUGINLIB_EXPORT_CLASS` against the matching base class.
3. Register the `<class>` block in the relevant XML; export it from that subpackage's `CMakeLists.txt` and `package.xml`.

## Tests
`colcon test --packages-select dwb_core dwb_critics dwb_plugins nav_2d_msgs nav_2d_utils costmap_queue`

## Docs to update
- `docs.nav2.org` → `configuration/packages/configuring-dwb-controller.md`.
- Plugins page when adding a critic / generator.

## Pitfalls
- Critics are scored **per trajectory candidate** — keep `scoreTrajectory` cheap.
- DWB is parameter-heavy; default params live in `nav2_bringup/params/nav2_params.yaml` under `controller_server.FollowPath` — keep both in sync.
- Subpackage boundaries are real: a critic in `dwb_critics` cannot include a header from `dwb_plugins` without a circular dep.
