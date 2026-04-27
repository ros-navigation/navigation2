# nav2_simple_commander — assistant guide

Pure-Python (`ament_python`) wrapper around the Nav2 action API. Used by examples, tutorials, integration tests, and external scripts. **Not on the runtime data path.**

## Public surface
- `nav2_simple_commander.robot_navigator.BasicNavigator` — primary class. Methods: `goToPose`, `goThroughPoses`, `followWaypoints`, `followPath`, `spin`, `backup`, `assistedTeleop`, `dock`, `undock`, etc.
- `nav2_simple_commander.line_iterator.LineIterator` — Bresenham helper.
- `nav2_simple_commander.costmap_2d.PyCostmap2D` — Python view of `OccupancyGrid` for analysis.
- `nav2_simple_commander.demos.*` — example scripts (security_demo, picking_demo, etc.).

## Layout
- `nav2_simple_commander/` — Python module
- `test/` — pytest tests
- `setup.py` — entry points for demo scripts
- `package.xml` declares ament_python build type + flake8/pep257/mypy lint deps

## Lint
Subject to **ament_flake8**, **ament_pep257**, **ament_mypy**, **codespell**, **isort**. mypy strict; ROS message stubs configured in `tools/pyproject.toml`.

## How to extend
- New high-level method on `BasicNavigator` → wrap a Nav2 action (`NavigateToPose`, `FollowPath`, etc.) using `rclpy` action client. Mirror the pattern of `goToPose`.
- New demo → add under `nav2_simple_commander/demos/`, register a console script in `setup.py`.

## Tests
`colcon test --packages-select nav2_simple_commander`

## Docs to update
- `docs.nav2.org` → tutorial pages (`tutorials/*`) — Simple Commander is heavily featured. Any public API change cascades into ~5 tutorial pages.

## Pitfalls
- This is the user-facing Python API — backwards compatibility is sacred. Deprecate (don't remove) old method names; keep signatures additive.
- Don't pull runtime logic out of C++ servers into this package — it's a *client*, not a participant in the data path.
- mypy strict-mode complains about untyped ROS message imports; add overrides in `tools/pyproject.toml` rather than scattering `# type: ignore`.
