# nav2_collision_monitor — assistant guide

Independent lifecycle node that watches sensor inputs (laser, point clouds, range, polygon) against configurable safety **polygons** and brakes / scales / stops the velocity commands flowing through `nav2_velocity_smoother` (or downstream). Safety-critical — change carefully.

## What it does
- Subscribes to source sensors (configurable types: scan, pointcloud2, range, polygon).
- Maintains polygon zones with an action: `stop`, `slowdown`, `limit`, `approach`.
- Publishes a modified `cmd_vel` (or `cmd_vel_in` → `cmd_vel_out` chain) when a zone is violated.
- Emits collision-detection diagnostics.

## Layout
- `include/nav2_collision_monitor/` — `collision_monitor_node.hpp`, `collision_detector_node.hpp`, source headers (`scan.hpp`, `pointcloud.hpp`, `range.hpp`, `polygon_source.hpp`), polygon shapes
- `src/` — node + source impls
- `test/`

## How to add a new source / polygon shape
1. Header in `include/nav2_collision_monitor/<name>.hpp` deriving from `Source` (or `Polygon`).
2. Implement sensor parsing → list of points in robot frame (with timestamp).
3. Register in the source factory / polygon factory.
4. Document the new params in `nav2_bringup/params/nav2_params.yaml`.

## Tests
`colcon test --packages-select nav2_collision_monitor`

## Docs to update
- `docs.nav2.org` → `configuration/packages/configuring-collision-monitor.md`.
- Migration guide for any change to the action enum or velocity-chain wiring (this is safety code).

## Pitfalls
- Time-synchronization across sources: TF lookups must use the source timestamp, not `now`.
- Latency budget on the cmd_vel pass-through is small (~ms) — never log/allocate in the hot path.
- A misconfigured polygon (e.g. zero-area) silently disables protection — validate at `on_configure`.
