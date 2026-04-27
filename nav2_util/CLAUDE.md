# nav2_util — assistant guide

Foundational shared library. Used by **every** Nav2 server. Public API changes ripple through the whole stack — extra caution.

## Public surface (most-used)
- `nav2_util::LifecycleNode` — base class for every lifecycle server in Nav2. Wraps `rclcpp_lifecycle::LifecycleNode` with bond, dynamic params, lifecycle helpers.
- `nav2_util::ServiceClient<T>` / `ServiceServer<T>` — typed wrappers.
- `nav2_util::SimpleActionServer<ActionT>` — wrapper over `rclcpp_action::Server` with single-goal preemption semantics used everywhere in Nav2.
- Robot / geometry utils: `robot_utils.hpp` (transform helpers), `geometry_utils.hpp`, `node_utils.hpp` (param declaration helpers).
- `nav2_util::LineIterator`, `nav2_util::PolygonUtils`.

## Layout
- `include/nav2_util/` — public headers
- `src/` — implementations
- `test/` — gtest binaries (one per util module)

## How to add / change
- New utility → header in `include/nav2_util/`, impl in `src/`, register the new source in `CMakeLists.txt` (it's already grouped into a single library target).
- Modifying `LifecycleNode` or `SimpleActionServer` → audit downstream consumers across the whole repo before changing signatures (`grep -r "nav2_util::LifecycleNode" --include="*.hpp"`).

## Tests
`./build/nav2_util/test_<name>` per module.
`colcon test --packages-select nav2_util`

## Docs to update
- `docs.nav2.org` → migration guide for any public-API change.
- Doxygen comments on every new public function.

## Pitfalls
- Every Nav2 server transitively depends on this — an ABI break cascades into every dependent rebuild.
- Prefer additive changes; deprecate (don't remove) old method signatures.
- `LifecycleNode` constructor side effects are forbidden — this base class is exactly where the "no ROS in ctor" rule matters most.
