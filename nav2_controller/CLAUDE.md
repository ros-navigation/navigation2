# nav2_controller — assistant guide

`ControllerServer` lifecycle node — hosts `nav2_core::Controller`, `nav2_core::ProgressChecker`, and `nav2_core::GoalChecker` plugins. Not a controller itself; algorithms live in `nav2_dwb_controller`, `nav2_mppi_controller`, `nav2_regulated_pure_pursuit_controller`, `nav2_graceful_controller`, `nav2_rotation_shim_controller`.

## Responsibilities
- Loads controllers + goal checkers + progress checkers by name from YAML via three separate `pluginlib::ClassLoader` instances.
- Provides `nav2_msgs/action/FollowPath`.
- Owns the local `Costmap2DROS`.
- Default progress checker (`SimpleProgressChecker`) and goal checker (`SimpleGoalChecker`, `StoppedGoalChecker`) ship from this package.

## Plugin XML
- `plugins.xml` — registers `nav2_controller::SimpleProgressChecker`, `nav2_controller::PoseProgressChecker`, `nav2_controller::SimpleGoalChecker`, `nav2_controller::StoppedGoalChecker`, `nav2_controller::PositionGoalChecker`.

## Lifecycle contract
Standard server pattern (see root CLAUDE.md). Costmap must be activated **before** controllers tick.

## Layout
- `include/nav2_controller/` — `controller_server.hpp`, `plugins/<name>.hpp`
- `src/` — server + default progress/goal checkers
- `test/`

## How to add a goal checker / progress checker
1. Header in `include/nav2_controller/plugins/`, derive from `nav2_core::GoalChecker` or `nav2_core::ProgressChecker`.
2. `PLUGINLIB_EXPORT_CLASS(...)` against the matching base class.
3. Add `<class>` block to `plugins.xml`.
4. Register in `nav2_bringup/params/nav2_params.yaml` under `controller_server.<goal|progress>_checker_plugin(s)`.

## Tests
`colcon test --packages-select nav2_controller`

## Docs to update
- `docs.nav2.org` → `configuration/packages/configuring-controller-server.md`, plus per-plugin pages for new checkers.
- Plugins page on new checker.

## Pitfalls
- Three pluginlib base classes share the server — easy to register a controller against the wrong base. Triple-check `base_class_type` in `plugins.xml`.
- `FollowPath` cancellation must propagate to the active controller's `computeVelocityCommands` — don't swallow `CancelCheck`.
- Goal-checker / progress-checker thresholds live in their own plugin namespaces — easy to confuse with controller params.
