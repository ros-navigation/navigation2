#include <string>

#include "nav2_behavior_tree/plugins/condition/is_in_allowed_area.hpp"

namespace nav2_behavior_tree
{

IsInAllowedArea::IsInAllowedArea(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  is_in_allowed_area_(false)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
  callback_group_executor_thread = std::thread([this]() {callback_group_executor_.spin();});

  std::string topic_name;
  getInput<std::string>("topic_name", topic_name);

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  cart_in_allowed_area_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    topic_name,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&IsInAllowedArea::onCartInAllowedAreaReceived, this, std::placeholders::_1),
    sub_option);

  RCLCPP_INFO_ONCE(node_->get_logger(), "Initialized an IsInAllowedArea BT node. Waiting on cart_in_allowed_area");
}

IsInAllowedArea::~IsInAllowedArea()
{
  RCLCPP_DEBUG(node_->get_logger(), "Shutting down IsInAllowedArea BT node");
  callback_group_executor_.cancel();
  callback_group_executor_thread.join();
}

void IsInAllowedArea::onCartInAllowedAreaReceived(const typename std_msgs::msg::Bool::SharedPtr msg)
{
  RCLCPP_INFO_ONCE(node_->get_logger(), "Got cart_in_allowed_area");

  is_in_allowed_area_ = msg->data;
}

BT::NodeStatus IsInAllowedArea::tick()
{
  if (is_in_allowed_area_) {
    RCLCPP_INFO(node_->get_logger(), "Cart in the mapped area");
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(node_->get_logger(), "Cart outside of the mapped area");
  return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsInAllowedArea>("IsInAllowedArea");
}
