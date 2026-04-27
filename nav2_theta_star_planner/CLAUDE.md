# nav2_theta_star_planner — assistant guide

Implements `nav2_core::GlobalPlanner` with Theta* (any-angle path planning on a grid). Produces shorter, less-jagged paths than NavFn at modest extra cost.

## Interface
`nav2_core::GlobalPlanner` — hosted by `nav2_planner::PlannerServer`.

## Plugin XML
- `theta_star_planner.xml` → `nav2_theta_star_planner::ThetaStarPlanner`.

## Layout
- `include/nav2_theta_star_planner/` — `theta_star.hpp`, `theta_star_planner.hpp`
- `src/` — `theta_star.cpp` (LOS check + parent-update), `theta_star_planner.cpp` (plugin wrapper)
- `test/` — `test_theta_star_planner.cpp`

## How to change behavior
- LOS / parent-update logic → `src/theta_star.cpp`. Tune `how_many_corners` and `w_*` cost weights via the planner param block in `nav2_bringup/params/nav2_params.yaml`.

## Tests
`./build/nav2_theta_star_planner/test_theta_star_planner`
`colcon test --packages-select nav2_theta_star_planner`

## Docs to update
- `docs.nav2.org` → `configuration/packages/configuring-thetastar.md` for params.

## Pitfalls
- The any-angle parent-update is the source of most regressions — keep the LOS check inclusive of the goal cell.
- Output paths are sparse (corners only). Downstream consumers expecting dense waypoints (e.g. some controllers) need a smoother in the chain.
