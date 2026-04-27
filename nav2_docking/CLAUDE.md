# nav2_docking — assistant guide

Metapackage for the **OpenNav Docking** subsystem. Subpackages:

- `opennav_docking` — `DockingServer` lifecycle node + default `ChargingDock` plugins.
- `opennav_docking_core` — pluginlib base classes (`ChargingDock`, `NonChargingDock`).
- `opennav_docking_bt` — BT nodes that invoke the docking actions.

## Interface
`opennav_docking_core::ChargingDock` (and `NonChargingDock`) — pluginlib bases for docking-station drivers.

## Plugin XML
- `opennav_docking/plugins.xml` — registers shipped dock plugins (e.g. `SimpleChargingDock`).

## Layout
Each subpackage is a full nav2 package with `include/`, `src/`, `test/`, its own `CMakeLists.txt` and `package.xml`. Treat them independently.

## How to add a new dock driver
1. Subclass `ChargingDock` (or `NonChargingDock`) from `opennav_docking_core`. Implement detection, refinement, charging state.
2. `PLUGINLIB_EXPORT_CLASS(...)` against the matching base.
3. Add `<class>` block to `opennav_docking/plugins.xml` (or your own package's plugin XML).
4. Register the dock under `docking_server.ros__parameters` in `nav2_bringup/params/nav2_params.yaml`.
5. Add a BT node in `opennav_docking_bt` if you need direct BT access; otherwise the `DockRobot` / `UndockRobot` actions cover it.

## Tests
`colcon test --packages-select opennav_docking opennav_docking_core opennav_docking_bt`

## Docs to update
- `docs.nav2.org` → docking tutorials and `configuration/packages/configuring-docking-server.md`.
- Plugins page on new dock driver.

## Pitfalls
- Detection refinement runs in a tight control loop — keep `getRefinedPose` cheap.
- Charging-state polling vs. dock-departure timing is the main source of bugs — use the lifecycle of the docking action precisely.
- Subpackage boundaries are real (see `nav2_dwb_controller` notes); don't cross-include.
