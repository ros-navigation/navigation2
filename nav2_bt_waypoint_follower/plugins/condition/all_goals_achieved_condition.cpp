#include <string>

#include "nav2_bt_waypoint_follower/plugins/condition/all_goals_achieved_condition.hpp"

namespace nav2_bt_waypoint_follower
{

AllGoalsAchievedCondition::AllGoalsAchievedCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
}

BT::NodeStatus AllGoalsAchievedCondition::tick()
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

}  // namespace nav2_bt_waypoint_follower

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_bt_waypoint_follower::AllGoalsAchievedCondition>("AllGoalsAchieved");
}