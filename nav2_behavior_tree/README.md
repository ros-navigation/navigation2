# nav2_behavior_tree

The nav2_behavior_tree module provides:
* A C++ template class for integrating ROS2 actions into Behavior Trees,
* Navigation-specific behavior tree nodes, and
* a generic BehaviorTreeEngine class that simplifies the integration of BT processing into ROS2 nodes.

This module is used by the nav2_bt_navigator to implement a ROS2 node that executes navigation Behavior Trees. The nav2_behavior_tree module uses the [Behavior-Tree.CPP library](https://github.com/BehaviorTree/BehaviorTree.CPP) for the core Behavior Tree processing.

## The bt_action_node Template and the Behavior Tree Engine

The [bt_action_node template](include/nav2_behavior_tree/bt_action_node.hpp) allows one to easily integrate a ROS2 action into a BehaviorTree. To do so, one derives from the BtActionNode template, providing the action message type. For example,

```C++
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"

class FollowPathAction : public BtActionNode<nav2_msgs::action::FollowPath>
{
    ...
};
```

The resulting node must be registered with the factory in the Behavior Tree engine in order to be available for use in Behavior Trees executed by this engine.

```C++
BehaviorTreeEngine::BehaviorTreeEngine()
{
    ...

  factory_.registerNodeType<nav2_behavior_tree::FollowPathAction>("FollowPath");

    ...
}
```

Once a new node is registered with the factory, it is now available to the BehaviorTreeEngine and can be used in Behavior Trees. For example, the following simple XML description of a BT shows the FollowPath node in use:

```XML
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="root">
      <ComputePathToPose goal="${goal}"/>
      <FollowPath path="${path}" controller_property="FollowPath"/>
    </Sequence>
  </BehaviorTree>
</root>
```
The BehaviorTree engine has a run method that accepts an XML description of a BT for execution:

```C++
  BtStatus run(
    BT::Blackboard::Ptr & blackboard,
    const std::string & behavior_tree_xml,
    std::function<void()> onLoop,
    std::function<bool()> cancelRequested,
    std::chrono::milliseconds loopTimeout = std::chrono::milliseconds(10));
```

See the code in the [BT Navigator](../nav2_bt_navigator/src/bt_navigator.cpp) for an example usage of the BehaviorTreeEngine.

## Navigation-Specific Behavior Tree Nodes

The nav2_behavior_tree package provides several navigation-specific nodes that are pre-registered and can be included in Behavior Trees.

| BT Node   |      Type      |  Description |
|----------|:-------------|------|
| Backup |  Action | Invokes the BackUp ROS2 action server, which causes the robot to back up to a specific pose. This is used in nav2 Behavior Trees as a recovery behavior. The nav2_recoveries module implements the BackUp action server. |
| ComputePathToPose |    Action   | Invokes the ComputePathToPose ROS2 action server, which is implemented by the nav2_planner module. The server address can be remapped using the `server_name` input port. |
| ComputePathThroughPoses |    Action   | Invokes the ComputePathThroughPoses ROS2 action server, which is implemented by the nav2_planner module. The server address can be remapped using the `server_name` input port. |
| RemovePassedGoals |    Action   | Removes `goals` from a vector of intermediary goal poses which the robot is either approaching or has passed within a tolerance of to prevent replanning to initial points. |
| FollowPath | Action |Invokes the FollowPath ROS2 action server, which is implemented by the controller plugin modules loaded. The server address can be remapped using the `server_name` input port. |
| GoalReached | Condition | Checks the distance to the goal, if the distance to goal is less than the pre-defined threshold, the tree returns SUCCESS, otherwise it returns FAILURE. |
| IsStuck | Condition | Determines if the robot is not progressing towards the goal. If the robot is stuck and not progressing, the condition returns SUCCESS, otherwise it returns FAILURE. |
| TransformAvailable | Condition | Checks if a TF transform is available. Returns failure if it cannot be found. Once found, it will always return success. Useful for initial condition checks. |
| GoalUpdated | Condition | Checks if the global navigation goal has changed in the blackboard. Returns failure if the goal is the same, if it changes, it returns success. |
| IsBatteryLow | Condition | Checks if battery is low by subscribing to a sensor_msgs/BatteryState topic and checking if battery voltage/percentage is below a specified minimum value. |
| NavigateToPose | Action | Invokes the NavigateToPose ROS2 action server, which is implemented by the bt_navigator module. |
| NavigateThroughPoses | Action | Invokes the NavigateThroughPoses ROS2 action server, which is implemented by the bt_navigator module. |
| RateController | Decorator | A node that throttles the tick rate for its child. The tick rate can be supplied to the node as a parameter. The node returns RUNNING when it is not ticking its child. Currently, in the navigation stack, the `RateController` is used to adjust the rate at which the `ComputePathToPose` and `GoalReached` nodes are ticked. |
| DistanceController | Decorator | A node that controls the tick rate for its child based on the distance traveled. The distance to be traveled before replanning can be supplied to the node as a parameter. The node returns RUNNING when it is not ticking its child. Currently, in the navigation stack, the `DistanceController` is used to adjust the rate at which the `ComputePathToPose` and `GoalReached` nodes are ticked. |
| SpeedController | Decorator | A node that controls the tick rate for its child based on the current robot speed. This decorator offers the most flexibility as the user can set the minimum/maximum tick rate which is adjusted according to the current robot speed. |
| SingleTrigger | Decorator | A node that triggers its child node(s) only a single time. It returns `FAILURE` in all other ticks. |
| RecoveryNode | Control | The RecoveryNode is a control flow node with two children.  It returns SUCCESS if and only if the first child returns SUCCESS. The second child will be executed only if the first child returns FAILURE. If the second child SUCCEEDS, then the first child will be executed again. The user can specify how many times the recovery actions should be taken before returning FAILURE. In nav2, the RecoveryNode is included in Behavior Trees to implement recovery actions upon failures.
| Spin | Action | Invokes the Spin ROS2 action server, which is implemented by the nav2_recoveries module. This action is used in nav2 Behavior Trees as a recovery behavior. |
| PipelineSequence | Control | Ticks the first child till it succeeds, then ticks the first and second children till the second one succeeds. It then ticks the first, second, and third children until the third succeeds, and so on, and so on. If at any time a child returns RUNNING, that doesn't change the behavior. If at any time a child returns FAILURE, that stops all children and returns FAILURE overall.|
| ClearEntireCostmap | Action | Invokes the ClearEntireCostmap ROS2 service server of costmap_2d_ros. The service address needs to be specified using the `service_name` input port, typically `local_costmap/clear_entirely_local_costmap` or `global_costmap/clear_entirely_global_costmap`. This action is used in nav2 Behavior Trees as a recovery behavior. |
| ClearCostmapExceptRegion | Action | Invokes the ClearCostmapExceptRegion ROS2 service server of costmap_2d_ros. It will clear all the costmap apart a square area centered on the robot of side size equal to the `reset_distance` input port. The service address needs to be specified using the `service_name` input port, typically `local_costmap/clear_except_local_costmap` or `global_costmap/clear_except_global_costmap`. This action is used in nav2 Behavior Trees as a recovery behavior. |
| ClearCostmapAroundRobot | Action | Invokes the ClearCostmapAroundRobot ROS2 service server of costmap_2d_ros. It will only clear the costmap on a square area centered on the robot of side size equal to the `reset_distance` input port. The service address needs to be specified using the `service_name` input port, typically `local_costmap/clear_around_local_costmap` or `global_costmap/clear_around_global_costmap`. This action is used in nav2 Behavior Trees as a recovery behavior. |
| PlannerSelector | Action | The PlannerSelector behavior is used to switch the planner that will be used by the planner server. It subscribes to a topic "planner_selector" to get the decision about what planner must be used. It is usually used before of the ComputePathToPoseAction. The selected_planner output port is passed to planner_id input port of the ComputePathToPoseAction.
| ControllerSelector | Action | The ControllerSelector behavior is used to switch the controller that will be used by the controller server. It subscribes to a topic "controller_selector" to get the decision about what controller must be used. It is usually used before of the FollowPath. The selected_controller output port is passed to controller_id input port of the FollowPath.
| GoalCheckerSelector | Action | The GoalCheckerSelector behavior is used to switch the current goal checker of the controller_server. It subscribes to a topic "goal_checker_selector" to get the decision about what goal_checker must be used. It is usually used before of the FollowPath. The selected_goal_checker output port is passed to goal_checker_id input port of the FollowPath.


For more information about the behavior tree nodes that are available in the default BehaviorTreeCPP library, see documentation here: https://www.behaviortree.dev/bt_basics/
