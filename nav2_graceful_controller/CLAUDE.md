# nav2_graceful_controller — assistant guide

Implements `nav2_core::Controller` with a graceful pose-following law (Park & Kuipers). Good for omnidirectional / differential platforms approaching SE2 goals.

## Interface
`nav2_core::Controller` — hosted by `nav2_controller::ControllerServer`.

## Plugin XML
- `graceful_controller_plugin.xml` → `nav2_graceful_controller::GracefulController`.

## Layout
- `include/nav2_graceful_controller/` — `graceful_controller.hpp`, `ego_polar_coords.hpp`, `smooth_control_law.hpp`, `path_handler.hpp`, `parameter_handler.hpp`
- `src/` — control law + path handling
- `test/`

## How to change behavior
- Control law gains (`k_phi`, `k_delta`, `beta`, `lambda`) → `smooth_control_law.hpp/.cpp`. Document gain changes in tuning guide.
- Pose-projection (motion target) → `path_handler.hpp/.cpp`.

## Tests
`colcon test --packages-select nav2_graceful_controller`

## Docs to update
- `docs.nav2.org` → `configuration/packages/configuring-graceful-controller.md`.
- Tuning guide for gain changes.

## Pitfalls
- The control law assumes goal pose is reachable in SE2 — wrap with a planner that produces orientation-aware paths (e.g. smac hybrid).
- Singularities at small `delta` — the implementation linearizes; test edge cases near goal.
