# nav2_behaviors — assistant guide

Hosts recovery / utility `nav2_core::Behavior` plugins (spin, backup, wait, drive_on_heading, assisted_teleop). Acts as both **server** (`BehaviorServer`) and ships the default behavior plugins.

## Interface
`nav2_core::Behavior` — `nav2_core/include/nav2_core/behavior.hpp`. Each behavior is an action server; lifecycle is driven by `BehaviorServer`.

## Plugin XML
- `behavior_plugin.xml` — registers Spin, BackUp, Wait, DriveOnHeading, AssistedTeleop.

## Layout
- `include/nav2_behaviors/` — `behavior_server.hpp`, `timed_behavior.hpp` (base templated CRTP), `plugins/<name>.hpp`
- `src/` — server + per-behavior implementations
- `test/` — gtest binaries per behavior

## How to add a new behavior
1. Header in `include/nav2_behaviors/plugins/` deriving from `TimedBehavior<ActionT>`.
2. Define a new action in `nav2_msgs/action/<Name>.action`.
3. Implement `onRun`, `onCycleUpdate` (timer tick), and lifecycle hooks.
4. `PLUGINLIB_EXPORT_CLASS(nav2_behaviors::<Name>, nav2_core::Behavior)`.
5. Add a `<class>` block to `behavior_plugin.xml`.
6. Register the behavior name + action name under `behavior_server.ros__parameters` in `nav2_bringup/params/nav2_params.yaml`.
7. Add a BT node in `nav2_behavior_tree/plugins/action/` so it can be invoked from a tree.

## Tests
`./build/nav2_behaviors/test_<name>` per behavior; `colcon test --packages-select nav2_behaviors`.

## Docs to update
- `docs.nav2.org` → `configuration/packages/configuring-behavior-server.md` for params.
- Plugins page when adding a new behavior.
- Add the new BT node to the BT XML index (Groot) and BT readme tables.

## Pitfalls
- `TimedBehavior::onCycleUpdate` runs at action rate — non-blocking only.
- Behaviors share `BehaviorServer`'s costmap subscription — cancel on preempt rather than re-subscribing.
- Action result codes must come from `nav2_msgs` — don't invent local enums.
