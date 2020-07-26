#ifndef NAV2_WAYPOINT_FOLLOWER__PLUGINS__ACTION__GET_NEXT_GOAL_ACTION_HPP_
#define NAV2_WAYPOINT_FOLLOWER__PLUGINS__ACTION__GET_NEXT_GOAL_ACTION_HPP_

#include <string>

#include "behaviortree_cpp_v3/condition_node.h"

namespace nav2_bt_waypoint_follower
{

class AllGoalsAchievedCondition : public BT::ConditionNode
{
public:
  AllGoalsAchievedCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  AllGoalsAchievedCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<bool>("goal_achieved", "Has the goal been achieved?"),};
  }
};

}  // namespace nav2_bt_waypoint_follower

#endif  // NAV2_WAYPOINT_FOLLOWER__PLUGINS__ACTION__GET_NEXT_GOAL_ACTION_HPP_
