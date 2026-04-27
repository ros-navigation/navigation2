# nav2_amcl — assistant guide

Adaptive Monte Carlo Localization. Lifecycle node providing 2D pose estimation against a static map. Hosts pluggable motion-model and laser-model plugins.

## Plugin interfaces
- `nav2_amcl::MotionModel` — base for differential / omnidirectional motion models.
- `nav2_amcl::LaserScanner` (or sensor model) — base for beam / likelihood-field laser models.

## Plugin XML
- `plugins.xml` — registers shipped motion + sensor models.

## Layout
- `include/nav2_amcl/` — `amcl_node.hpp`, `motion_model/`, `sensors/`, `pf/` (particle filter), `map/`
- `src/` — node, particle filter, models
- `test/` — large gtest suite

## How to add a new motion / sensor model
1. Header in `include/nav2_amcl/motion_model/` or `sensors/`, derive from the appropriate base.
2. Implement `odometryUpdate` (motion) or `getProb`/`updateSensor` (laser).
3. `PLUGINLIB_EXPORT_CLASS(...)` against the matching base.
4. Add `<class>` block to `plugins.xml`.
5. Register the model name in `nav2_bringup/params/nav2_params.yaml` under `amcl.ros__parameters`.

## Tests
`colcon test --packages-select nav2_amcl`

## Docs to update
- `docs.nav2.org` → `configuration/packages/configuring-amcl.md`.
- Plugins page on new motion / sensor model.

## Pitfalls
- This is a port of the classic ROS 1 AMCL — preserve numerical behavior. Compare-vs-ROS-1 is the canonical regression check.
- Particle-cloud size + resampling cadence interaction is the hot path; test with high update rates.
- `set_initial_pose` topic vs. service is a common configuration mismatch — surface it.
