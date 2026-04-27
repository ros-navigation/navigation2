# nav2_bt_navigator — assistant guide

The top-of-stack lifecycle node that runs Behavior Trees. Hosts `nav2_core::BehaviorTreeNavigator` plugins (NavigateToPose, NavigateThroughPoses, route-based variants).

## Interface
`nav2_core::BehaviorTreeNavigator` — `nav2_core/include/nav2_core/behavior_tree_navigator.hpp`. Each navigator plugin owns its action server + tree.

## Plugin XML
- `navigator_plugins.xml` — registers the shipped navigators (`nav2_bt_navigator::NavigateToPoseNavigator`, `nav2_bt_navigator::NavigateThroughPosesNavigator`).

## Layout
- `include/nav2_bt_navigator/` — `bt_navigator.hpp` (server), `navigators/<name>.hpp`
- `src/` — server + navigator implementations
- `behavior_trees/` — default XML trees (`navigate_to_pose_w_replanning_and_recovery.xml`, etc.) — edited as part of feature changes
- `test/`

## How to add a new navigator
1. Header in `include/nav2_bt_navigator/navigators/<name>.hpp` deriving from `BehaviorTreeNavigator<ActionT>`.
2. Implement `getDefaultBTFilepath`, `goalReceived`, `goalCompleted`, `onLoop`, `onPreempt`.
3. `PLUGINLIB_EXPORT_CLASS(nav2_bt_navigator::<Name>Navigator, nav2_core::NavigatorBase)`.
4. Add a `<class>` block to `navigator_plugins.xml`.
5. Ship a default tree under `behavior_trees/` and reference it via the navigator's `default_<name>_bt_xml` param in `nav2_bringup/params/nav2_params.yaml`.

## Tests
`colcon test --packages-select nav2_bt_navigator`

## Docs to update
- `docs.nav2.org` → `configuration/packages/configuring-bt-navigator.md`, navigator reference pages, behavior-tree usage docs.
- Plugins page if a new navigator is added.

## Pitfalls
- Default tree XMLs in `behavior_trees/` are user-facing — never break their structure without a migration guide entry.
- BT blackboard keys (`goal`, `path`, etc.) are part of the implicit contract between the navigator and the tree — coordinate renames with `nav2_behavior_tree` BT nodes.
- The navigator's action server lifecycle is driven by `BTNavigator` — do not start your own action server inside a navigator plugin.
