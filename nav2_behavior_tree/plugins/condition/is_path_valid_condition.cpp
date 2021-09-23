#include <string>
#include "nav2_behavior_tree/plugins/condition/is_path_valid_condition.hpp"


namespace nav2_behavior_tree
{
IsPathValidCondition::IsPathValidCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  client = node_->create_client<nav2_msgs::srv::IsPathValid>("is_path_valid");
}

BT::NodeStatus IsPathValidCondition::tick()
{

  nav_msgs::msg::Path path;
  getInput("path", path);

  auto request = std::make_shared<nav2_msgs::srv::IsPathValid::Request>();

  request->path = path;

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node_, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_DEBUG(node_->get_logger(), "Got response");

    if (result.get()->is_valid) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  } else {
    RCLCPP_DEBUG(node_->get_logger(), "No response");
    return BT::NodeStatus::FAILURE;
  }
}


} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsPathValidCondition>("IsPathValid");
}
