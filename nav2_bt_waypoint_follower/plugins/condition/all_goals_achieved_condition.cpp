#include <string>

#include "behaviortree_cpp_v3/condition_node.h"

namespace nav2_bt_waypoint_follower
{

class AllGoalsAchievedCondition : public BT::ConditionNode
{
public:
  AllGoalsAchievedCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf)
  {
  }

  AllGoalsAchievedCondition() = delete;

  BT::NodeStatus tick() override
  {
    int64_t current_waypoint_idx = config().blackboard->get<int64_t>("current_waypoint_idx");
    int64_t num_waypoints = config().blackboard->get<int64_t>("num_waypoints");

    if (current_waypoint_idx >= num_waypoints - 1) {
      bool goal_achieved;
      if (!getInput("goal_achieved", goal_achieved)) {
        goal_achieved = false;
      }
      if (goal_achieved) {
        return BT::NodeStatus::SUCCESS;
      }
    }

    return BT::NodeStatus::FAILURE;
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<bool>("goal_achieved", "Has the goal been achieved?"),};
  }
};

}  // namespace nav2_bt_waypoint_follower

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_bt_waypoint_follower::AllGoalsAchievedCondition>("AllGoalsAchieved");
}