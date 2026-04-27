# nav2_ros_common — assistant guide

Shared `rclcpp` typedefs / wrappers used by lifecycle servers (publishers, subscriptions, action servers/clients). Exists to reduce boilerplate and centralize lifecycle integration. Header-only / thin wrappers.

## Public surface
- Lifecycle-aware publisher / subscription typedefs.
- Action server / client wrappers tuned for `nav2_util::LifecycleNode`.
- QoS profile constants.

## Layout
- `include/nav2_ros_common/` — public headers only (no `src/` of consequence)
- `test/` — small surface

## How to add / change
- New typedef / wrapper → header in `include/nav2_ros_common/`. Keep them minimal — anything stateful belongs in `nav2_util`.
- ABI changes affect every server that includes the wrapper.

## Tests
`colcon test --packages-select nav2_ros_common`

## Docs to update
- `docs.nav2.org` migration guide for any public-API change.
- Doxygen on every new wrapper.

## Pitfalls
- Same caveat as `nav2_util`: depended on transitively by every server.
- Don't import rclcpp internals — public rclcpp API only, so wrappers stay portable across distros.
