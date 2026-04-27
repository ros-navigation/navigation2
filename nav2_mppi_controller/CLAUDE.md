# nav2_mppi_controller — assistant guide

Implements `nav2_core::Controller` with Model Predictive Path Integral control. Plugin-rich: hosts its own sub-plugins (critics, motion models, trajectory validators).

## Interface
`nav2_core::Controller` — `nav2_core/include/nav2_core/controller.hpp`. Hosted by `nav2_controller::ControllerServer`. Method that matters: `computeVelocityCommands` (called at controller frequency).

## Plugin XMLs
- `mppic.xml` → `nav2_mppi_controller::MPPIController` (the controller itself)
- `critics.xml` → critic plugins (path-align, path-angle, goal, obstacles, etc.)
- `motion_models.xml` → DiffDrive / Omni / Ackermann
- `trajectory_validators.xml` → trajectory-level constraint plugins

Each is its own pluginlib base class — don't conflate them.

## Layout
- `include/nav2_mppi_controller/` — `controller.hpp`, `optimizer.hpp`, `motion_models.hpp`, `critic_function.hpp`, `trajectory_validator.hpp`
- `include/nav2_mppi_controller/critics/` and `src/critics/` — critic implementations (one class per critic)
- `include/nav2_mppi_controller/trajectory_validators/` and `src/trajectory_validators/`
- `benchmark/` — perf harness; rebuild + run when changing the optimizer hot path
- `test/` — gtest binaries (`test_optimizer`, `test_critics`, `test_motion_models`, `test_trajectory_validators`)

## How to add a new critic
1. New `.hpp` in `include/nav2_mppi_controller/critics/` deriving from `CriticFunction`; override `initialize` + `score`.
2. Impl in `src/critics/`.
3. `PLUGINLIB_EXPORT_CLASS(nav2_mppi_controller::critics::<Name>Critic, mppi::critics::CriticFunction)`.
4. Add a `<class>` block to `critics.xml`.
5. Register the new critic name + weights in the controller's param block.

## Tests
`./build/nav2_mppi_controller/test_optimizer` (etc.)
`colcon test --packages-select nav2_mppi_controller`

## Docs to update
- `docs.nav2.org` → `configuration/packages/configuring-mppic.md` for params.
- Tuning guide is a major source of MPPI traffic — update for any default change or new critic.
- Plugins page when adding a critic / motion model / validator.

## Pitfalls
- `computeVelocityCommands` is called at controller frequency (~20-30 Hz). The hot path is the optimizer's tensor evaluation — profile via `benchmark/` before merging changes.
- Critics see the **rolled-out trajectory tensor**, not a single trajectory — vectorize with xtensor; loops will tank the controller.
- The optimizer is single-threaded by design; do not introduce thread spawns inside it.
