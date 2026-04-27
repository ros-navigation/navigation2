# nav2_following — assistant guide

Metapackage hosting the **OpenNav Following** subsystem. Currently a single subpackage:

- `opennav_following` — lifecycle node + action interface for following a moving target (person, robot, AR tag, etc.).

## What it does
- Subscribes to a target pose stream.
- Issues controller goals to maintain a configurable offset / formation behind the target.
- Exposes `FollowObject` (or similar) action used by BT nodes.

## Layout
- `opennav_following/include/`, `src/`, `test/` — standard nav2 package layout

## How to extend
- Following policy / spacing → server logic.
- New target type → wrap the pose stream as a topic the following server already supports, or add a new subscriber type.

## Tests
`colcon test --packages-select opennav_following`

## Docs to update
- `docs.nav2.org` → following-feature tutorials and configuration page.
- Migration guide for action-signature changes.

## Pitfalls
- Target loss handling is the main correctness issue — define the timeout behavior explicitly (stop / search / preempt).
- Following on a robot without `nav2_collision_monitor` is unsafe — document the safety dependency.
