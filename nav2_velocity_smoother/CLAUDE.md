# nav2_velocity_smoother — assistant guide

Lifecycle node that low-pass-filters / acceleration-limits the `cmd_vel` stream from controllers / behaviors before it reaches the base. Sits between controller output and robot driver.

## What it does
- Subscribes `cmd_vel` (input topic configurable).
- Applies linear/angular velocity + acceleration + deadband limits.
- Publishes the smoothed `cmd_vel_smoothed`.
- Provides odometry-aware feedback when configured.

## Layout
- `include/nav2_velocity_smoother/` — `velocity_smoother.hpp`
- `src/` — implementation
- `test/`

## How to change behavior
- Smoothing kernel / saturation logic → `velocity_smoother.cpp`.
- New limit type → extend the param schema and the kernel; document in `nav2_bringup/params/nav2_params.yaml`.

## Tests
`colcon test --packages-select nav2_velocity_smoother`

## Docs to update
- `docs.nav2.org` → `configuration/packages/configuring-velocity-smoother.md`.
- Tuning guide for default limit changes.

## Pitfalls
- Update rate vs. controller rate mismatch causes lag — `smoothing_frequency` should be ≥ controller rate.
- Disabling the smoother entirely (zero limits) is a footgun — produce a clear warning at `on_configure`.
- Sits between `nav2_collision_monitor` and the base in some configurations — coordinate topic names with the safety chain.
