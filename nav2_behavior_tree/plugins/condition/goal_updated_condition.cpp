#ifndef NAV2_BEHAVIOR_TREE__GOAL_UPDATED_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__GOAL_UPDATED_CONDITION_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/robot_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_behavior_tree
{

class GoalUpdatedCondition : public BT::ConditionNode
{
public:
  GoalUpdatedCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf), initialized_(false)
  {
  }

  GoalUpdatedCondition() = delete;

  BT::NodeStatus tick() override
  {
    if (!initialized_) {
      initialize();
    }

    if (isGoalUpdated()) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

  void initialize()
  {
    goal_ = config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal");
    initialized_ = true;
  }

  bool
  isGoalUpdated()
  {
    auto current_goal = config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal");
    if (goal_ != current_goal) {
      goal_ = current_goal;
      return true;
    } else {
      return false;
    }
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

private:
  geometry_msgs::msg::PoseStamped goal_;
  bool initialized_;
};

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::GoalUpdatedCondition>("GoalUpdated");
}

#endif  // NAV2_BEHAVIOR_TREE__GOAL_UPDATED_CONDITION_HPP_
