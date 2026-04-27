# <PKG_NAME> — assistant guide

Lifecycle server hosting `<Interface>` plugins. Derives from `nav2_util::LifecycleNode`.

## Responsibilities
- Loads plugins by name from YAML (param `<plugin_param_name>`) via `pluginlib::ClassLoader<nav2_core::<Interface>>`.
- Provides the `<ActionName>` action (`nav2_msgs/action/<ActionName>`).
- Owns its own `Costmap2DROS` instance(s) where applicable.

## Lifecycle contract
- `on_configure`: declare params, instantiate plugins, create publishers/subscribers/action server.
- `on_activate`: activate publishers, lifecycle-activate child costmap.
- `on_deactivate` / `on_cleanup`: tear down in reverse order.
- Never allocate ROS entities in the constructor.

## Layout
- `include/<pkg>/` — server header
- `src/` — server implementation + `main.cpp`
- `test/` — gtest binaries

## How to add a new feature
- New action / service → declare in `nav2_msgs`, add the server endpoint here, expose a BT node in `nav2_behavior_tree` if user-facing.
- New plugin slot on this server → extend `nav2_core` with the new interface, add the `pluginlib::ClassLoader`, document the param in `nav2_bringup/params/nav2_params.yaml`.

## Tests
- `./build/<pkg>/<test_name>`
- `colcon test --packages-select <pkg>`

## Docs to update when touching this package
- `docs.nav2.org` → `configuration/packages/configuring-<pkg>.md` for params, `concepts/index.md` if architecture shifts.
- Migration guide for any non-backwards-compatible API or param change.

## Pitfalls
- A plugin throwing during `configure`/`activate` will hang the lifecycle manager and the whole stack — surface errors via the right exception type from `nav2_core/*_exceptions.hpp`.
- Don't share `Costmap2DROS` across servers — each server owns its own.
