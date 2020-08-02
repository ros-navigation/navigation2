#ifndef NAV2_BT_WAYPOINT_FOLLOWER__PLUGINS__ACTION__GET_NEXT_GOAL_ACTION_HPP_
#define NAV2_BT_WAYPOINT_FOLLOWER__PLUGINS__ACTION__GET_NEXT_GOAL_ACTION_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_bt_waypoint_follower
{

class GetNextGoalAction : public BT::SyncActionNode
{
public:
  GetNextGoalAction(
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  GetNextGoalAction() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
        "goals",
        "Destinations to plan to"),
      BT::BidirectionalPort<bool>("goal_achieved", "Has the goal been achieved?"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination to plan to")};
  }
};

}  // namespace nav2_bt_waypoint_follower

#endif  // NAV2_BT_WAYPOINT_FOLLOWER__PLUGINS__ACTION__GET_NEXT_GOAL_ACTION_HPP_
