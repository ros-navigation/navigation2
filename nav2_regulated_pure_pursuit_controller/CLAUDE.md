# nav2_regulated_pure_pursuit_controller — assistant guide

Implements `nav2_core::Controller` with Regulated Pure Pursuit (RPP). Lightweight, popular default for many platforms.

## Interface
`nav2_core::Controller` — hosted by `nav2_controller::ControllerServer`.

## Plugin XML
- `nav2_regulated_pure_pursuit_controller.xml` → `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`.

## Layout
- `include/nav2_regulated_pure_pursuit_controller/` — `regulated_pure_pursuit_controller.hpp`, `parameter_handler.hpp`, `path_handler.hpp`, `collision_checker.hpp`, `regulation_functions.hpp`
- `src/` — controller + helpers
- `test/` — `test_regulated_pp.cpp` and helpers

## How to change behavior
- Lookahead, regulation, curvature → `regulation_functions.hpp` / `.cpp`
- Path following / pruning → `path_handler.hpp`
- Collision check during rollout → `collision_checker.hpp` (uses costmap inflation)

## Tests
`./build/nav2_regulated_pure_pursuit_controller/test_regulated_pp`
`colcon test --packages-select nav2_regulated_pure_pursuit_controller`

## Docs to update
- `docs.nav2.org` → `configuration/packages/configuring-regulated-pp.md` for params.
- Tuning guide for changes to lookahead / regulation defaults.

## Pitfalls
- `computeVelocityCommands` is on the controller hot path — keep allocations out of it.
- Curvature & velocity regulation interacts with the goal checker and progress checker — test the full chain when changing thresholds.
