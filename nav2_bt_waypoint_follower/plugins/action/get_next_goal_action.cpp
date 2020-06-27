#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_bt_waypoint_follower
{

class GetNextGoal : public BT::SyncActionNode
{
public:
  GetNextGoal(
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(action_name, conf)
  {
  }

  GetNextGoal() = delete;

  BT::NodeStatus tick() override
  {
    std::vector<geometry_msgs::msg::PoseStamped> goals;
    if (!getInput("goals", goals)) {
      return BT::NodeStatus::FAILURE;
    }
    int64_t current_waypoint_idx = config().blackboard->get<int64_t>("current_waypoint_idx");
    int64_t num_waypoints = config().blackboard->get<int64_t>("num_waypoints");

    bool goal_achieved;
    if (!getInput("goal_achieved", goal_achieved)) {
      goal_achieved = false;
    }

    if (!goal_achieved) {
      return BT::NodeStatus::SUCCESS;
    }

    if (current_waypoint_idx >= num_waypoints - 1) {
      return BT::NodeStatus::FAILURE;
    }

    setOutput("goal_achieved", false);
    setOutput("goal", goals[current_waypoint_idx + 1]);
    config().blackboard->set("current_waypoint_idx", current_waypoint_idx + 1);
    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("goals", "Destinations to plan to"),
            BT::BidirectionalPort<bool>("goal_achieved", "Has the goal been achieved?"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination to plan to")};
  }
};

}  // namespace nav2_bt_waypoint_follower

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_bt_waypoint_follower::GetNextGoal>("GetNextGoal");
}