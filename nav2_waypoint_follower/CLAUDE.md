# nav2_waypoint_follower — assistant guide

Lifecycle server that drives a sequence of waypoints by repeatedly invoking `NavigateToPose`. Hosts `nav2_core::WaypointTaskExecutor` plugins (e.g. wait, photo capture, input-at-waypoint).

## Interface
`nav2_core::WaypointTaskExecutor` — `nav2_core/include/nav2_core/waypoint_task_executor.hpp`. Called between waypoints to perform per-waypoint side effects.

## Plugin XML
- `plugins.xml` — registers shipped executors (`WaitAtWaypoint`, `PhotoAtWaypoint`, `InputAtWaypoint`).

## Layout
- `include/nav2_waypoint_follower/` — `waypoint_follower.hpp` (server), `plugins/<name>.hpp`
- `src/` — server + plugin impls
- `test/`

## How to add a new task executor
1. Header in `include/nav2_waypoint_follower/plugins/`, derive from `nav2_core::WaypointTaskExecutor`.
2. Implement `initialize`, `processAtWaypoint`.
3. `PLUGINLIB_EXPORT_CLASS(nav2_waypoint_follower::<Name>, nav2_core::WaypointTaskExecutor)`.
4. Add `<class>` to `plugins.xml`.
5. Register under `waypoint_follower.ros__parameters` in `nav2_bringup/params/nav2_params.yaml`.

## Tests
`colcon test --packages-select nav2_waypoint_follower`

## Docs to update
- `docs.nav2.org` → `configuration/packages/configuring-waypoint-follower.md`.
- Plugins page on new executor.

## Pitfalls
- `processAtWaypoint` blocks the next-waypoint dispatch — keep it bounded; long tasks belong in a separate action.
- If `stop_on_failure` is true and an executor reports failure, the whole follow is aborted — surface task failures via the right return code, not exceptions.
