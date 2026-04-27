# nav2_smoother — assistant guide

Both the **smoother server** (`SmootherServer`) and ships the default `SimpleSmoother` + `SavitzkyGolaySmoother` plugins implementing `nav2_core::Smoother`.

## Interface
`nav2_core::Smoother` — `nav2_core/include/nav2_core/smoother.hpp`. Hosted by `nav2_smoother::SmootherServer`.

## Plugin XML
- `plugins.xml` — registers `nav2_smoother::SimpleSmoother`, `nav2_smoother::SavitzkyGolaySmoother`.

## Layout
- `include/nav2_smoother/` — `nav2_smoother.hpp` (server), `simple_smoother.hpp`, `savitzky_golay_smoother.hpp`
- `src/` — server + plugin implementations
- `test/`

## How to add a new smoother variant
1. Header in `include/nav2_smoother/`, implement `nav2_core::Smoother::smooth(path, max_time)`.
2. `PLUGINLIB_EXPORT_CLASS(nav2_smoother::<Name>, nav2_core::Smoother)`.
3. Add `<class>` block to `plugins.xml`.
4. Register under `smoother_server.ros__parameters.smoother_plugins` in `nav2_bringup/params/nav2_params.yaml`.

## Tests
`colcon test --packages-select nav2_smoother`

## Docs to update
- `docs.nav2.org` → `configuration/packages/configuring-smoother-server.md`.
- Plugins page on new smoother.

## Pitfalls
- `smooth` must respect `max_time` — return whatever you have if budget runs out; do not block.
- Smoothers must preserve start and goal poses exactly (downstream goal-checker expects them).
- For trajectory smoothing of SE2 paths use `nav2_constrained_smoother` or smac's smoother — this package targets 2D paths.
