# nav2_dstar_lite_planner

D* Lite global planning plugin for Nav2. Implements the D* Lite incremental heuristic
search algorithm, which reuses previous search results when the costmap changes,
making it efficient for dynamic environments with frequent replanning.

## Features

- Incremental replanning: only recomputes affected portions of the search space
- 8-connected (holonomic) grid search
- Configurable periodic full replan to escape local minima
- Path switching hysteresis to prevent planner oscillation
- Dynamic parameter reconfiguration

## Parameters

All parameters are namespaced under the planner plugin name.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `allow_unknown` | bool | true | Whether to plan through unknown space |
| `max_iterations` | int | 100000 | Max iterations per computeShortestPath (-1 = unlimited) |
| `replan_interval` | int | 10 | Incremental replans before forced full replan |
| `hysteresis_factor` | double | 1.05 | New path cost must be lower than old / factor to switch |
| `terminal_checking_interval` | int | 5000 | Iterations between cancel checks |
| `use_final_approach_orientation` | bool | false | Keep start orientation at path end |

## Usage

Add to your Nav2 planner server configuration:

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["DStarLite"]
    DStarLite:
      plugin: "nav2_dstar_lite_planner/DStarLitePlanner"
      DStarLite.allow_unknown: true
      DStarLite.max_iterations: 100000
      DStarLite.replan_interval: 10
      DStarLite.hysteresis_factor: 1.05
      DStarLite.terminal_checking_interval: 5000
      DStarLite.use_final_approach_orientation: false
```
