# nav2_bringup — assistant guide

Canonical entry point for running Nav2. Contains `launch/` files, default `params/nav2_params.yaml`, default maps and worlds, and RViz configs. **Source of truth for "how is X configured by default".**

## Layout
- `launch/` — `bringup_launch.py`, `tb3_simulation_launch.py`, `localization_launch.py`, `navigation_launch.py`, etc. Composable; `bringup_launch.py` includes the others.
- `params/nav2_params.yaml` — default parameter set for **every** Nav2 server. New plugins MUST add a default block here.
- `params/nav2_multirobot_params_<n>.yaml` — multi-robot variants.
- `maps/` — example occupancy maps used by simulations and integration tests.
- `worlds/` — Gazebo worlds.
- `rviz/` — RViz configs.
- `urdf/` — example robot models.
- `test/` — launch + integration tests.

## How to wire a new plugin / param
1. Decide which server hosts it.
2. Add a new key under `<server>.ros__parameters` in `params/nav2_params.yaml`. Include a sensible default and inline comments for non-obvious knobs.
3. If the plugin needs new launch arguments, expose them via `DeclareLaunchArgument` in the relevant `*_launch.py` and propagate via `param_substitutions`.
4. Verify the bringup boots clean: `ros2 launch nav2_bringup tb3_simulation_launch.py`.

## Lint / type-check
Launch files are `.launch.py`. Subject to **ament_flake8**, **ament_pep257**, **ament_mypy** (mypy currently disabled in pre-commit, but `package.xml` still gates it).

## Tests
`colcon test --packages-select nav2_bringup`

## Docs to update
- `docs.nav2.org` → `getting_started/index.md` and `configuration/index.md` if launch / param defaults change in user-visible ways.
- Tuning guide if a default param value moves.
- Migration guide if a launch argument is renamed or removed.

## Pitfalls
- `nav2_params.yaml` is the most-read file in the project — clarity beats cleverness in comments and defaults.
- Multi-robot variants drift from the single-robot file easily — keep them in sync after parameter additions.
- Launch composability: prefer `IncludeLaunchDescription` over copy-pasting nodes between launch files.
- Default params must work on the example TB3 world out-of-the-box; if a change requires retuning, ship the new defaults too.
