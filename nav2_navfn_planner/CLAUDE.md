# nav2_navfn_planner — assistant guide

Implements `nav2_core::GlobalPlanner` using NavFn (Dijkstra / A*) on a 2D costmap. Default global planner shipped by `nav2_bringup`.

## Interface
`nav2_core::GlobalPlanner` — `nav2_core/include/nav2_core/global_planner.hpp`. Hosted by `nav2_planner::PlannerServer`.

## Plugin XML
- `global_planner_plugin.xml` → `nav2_navfn_planner::NavfnPlanner`.

## Layout
- `include/nav2_navfn_planner/` — `navfn_planner.hpp`, `navfn.hpp` (core algorithm)
- `src/` — `navfn.cpp` (potential-field expansion), `navfn_planner.cpp` (plugin wrapper)
- `test/` — `test_navfn_planner.cpp`

## How to change behavior
- Algorithm tweaks → `src/navfn.cpp` (potential-field PDE solver). Be careful: this is a port of the classic ROS 1 NavFn — comparing vs. that reference is the canonical regression test.
- Plugin-level changes (smoothing, tolerance, params) → `src/navfn_planner.cpp`.

## Tests
`./build/nav2_navfn_planner/test_navfn_planner`
`colcon test --packages-select nav2_navfn_planner`

## Docs to update
- `docs.nav2.org` → `configuration/packages/configuring-navfn.md` for params.
- Migration guide for any breaking change (this is the default planner — extra caution).

## Pitfalls
- `use_astar` param flips Dijkstra↔A* in the same code path — test both when changing core math.
- Planner returns failure on disconnected start/goal — error propagation must use the right exception from `nav2_core/planner_exceptions.hpp` so `PlannerServer` reports a proper goal failure.
