# nav2_core — assistant guide

**Pure abstract interfaces.** Defines every pluginlib base class in Nav2. No implementations, no ROS time, no costmap dependencies — only public interfaces and exception types. Touched only when adding a new plugin family or changing a contract.

## Interfaces shipped (one header each)
- `global_planner.hpp` → `nav2_core::GlobalPlanner` (planners)
- `controller.hpp` → `nav2_core::Controller`
- `smoother.hpp` → `nav2_core::Smoother`
- `behavior.hpp` → `nav2_core::Behavior`
- `progress_checker.hpp` → `nav2_core::ProgressChecker`
- `goal_checker.hpp` → `nav2_core::GoalChecker`
- `behavior_tree_navigator.hpp` → `nav2_core::BehaviorTreeNavigator`
- `waypoint_task_executor.hpp` → `nav2_core::WaypointTaskExecutor`
- `path_handler.hpp` → `nav2_core::PathHandler`
- `*_exceptions.hpp` — typed exception hierarchies for planner / controller / smoother / route. **Servers depend on these to produce correct error codes.**

## How to add a new plugin family
1. Add the new abstract class header under `include/nav2_core/`. Keep it virtual-only; no impl.
2. Add a matching `*_exceptions.hpp` file with a base `<Family>Exception : public std::runtime_error` and at least the canonical subtypes.
3. Add the host server (`pluginlib::ClassLoader<nav2_core::<Family>>`) in the relevant server package.
4. Update the migration guide on `docs.nav2.org` — adding an interface family is a major event.

## How to extend an existing interface
- **Adding a virtual method = ABI break.** Either provide a default impl, or stage in two phases: deprecate first, remove next release; either path requires a migration guide entry.
- New exception types → add to the existing hierarchy, keep base class additive.

## Tests
None — interfaces only. Implementations are tested in their concrete packages.

## Docs to update
- `docs.nav2.org` migration guide for any change here. Plugins page if a new interface family lands.

## Pitfalls
- Every interface change is felt across the whole stack and by every downstream third-party plugin.
- Don't reach for `rclcpp` types in these headers — keep them dependency-light. Use forward declarations / `std::shared_ptr` where possible.
- Exception hierarchies are part of the contract — `PlannerServer` (etc.) catches and dispatches based on type. Renames break error reporting silently.
