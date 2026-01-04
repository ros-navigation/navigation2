#include "nav2_behavior_tree/plugins/condition/is_goal_nearby_condition.hpp"
#include "nav2_util/geometry_utils.hpp"

namespace nav2_behavior_tree
{

IsGoalNearbyCondition::IsGoalNearbyCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
}

bool IsGoalNearbyCondition::isRobotInGoalProximity(
  const nav_msgs::msg::Path& goal_path,
  const double& prox_thr)
{
  return nav2_util::geometry_utils::calculate_path_length(goal_path, 0) < prox_thr;
}


BT::NodeStatus IsGoalNearbyCondition::tick()
{
    nav_msgs::msg::Path path;
    double prox_thr;
    getInput("path", path);
    getInput("proximity_threshold", prox_thr);
    if (!path.poses.empty()){
        if(isRobotInGoalProximity(path,prox_thr)){
            return BT::NodeStatus::SUCCESS;
        }
    }
    return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsGoalNearbyCondition>("IsGoalNearby");
}
