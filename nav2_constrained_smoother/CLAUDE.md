# nav2_constrained_smoother — assistant guide

Implements `nav2_core::Smoother` using a Ceres-based optimization with curvature, smoothness, and obstacle-avoidance constraints. Targets SE2 paths (e.g. from `nav2_smac_planner`).

## Interface
`nav2_core::Smoother` — hosted by `nav2_smoother::SmootherServer`.

## Plugin XML
- `nav2_constrained_smoother.xml` → `nav2_constrained_smoother::ConstrainedSmoother`.

## Layout
- `include/nav2_constrained_smoother/` — `constrained_smoother.hpp`, `smoother.hpp`, `smoother_cost_function.hpp`, `options.hpp`
- `src/` — Ceres problem assembly + plugin glue
- `test/`

## How to change behavior
- Cost terms / weights → `smoother_cost_function.hpp` (Ceres `AutoDiffCostFunction`).
- Solver settings → `options.hpp` + `nav2_bringup/params/nav2_params.yaml`.

## Tests
`colcon test --packages-select nav2_constrained_smoother`

## Docs to update
- `docs.nav2.org` → `configuration/packages/configuring-constrained-smoother.md`.
- Tuning guide for cost-weight changes.

## Pitfalls
- Ceres residuals must be cheap — they're evaluated O(iters × points × terms).
- AutoDiff requires templated functors; do not call non-template ROS APIs from inside the cost function.
- Solver divergence usually means weights are mis-balanced — log Ceres summary on failure rather than silently returning.
