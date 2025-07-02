#include <string>
#include <memory>
#include "nav2_behavior_tree/plugins/condition/is_following_path_condition.hpp"
#include "nav2_msgs/msg/tracking_error.hpp"

namespace nav2_behavior_tree
{

IsFollowingPathCondition::IsFollowingPathCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  node_(nullptr),
  error_sub_(nullptr),
  callback_group_(nullptr),
  callback_group_executor_(),
  error_topic_(),
  max_error_(1.0),
  last_tracking_error_(0.0),
  initialized_(false),
  received_first_error_(false)
{
  std::cout << "IsFollowingPathCondition created" << std::endl;
}

IsFollowingPathCondition::~IsFollowingPathCondition() = default;

void IsFollowingPathCondition::initialize()
{
  if (initialized_) {return;}

  getInput("error_topic", error_topic_);
  getInput("max_error", max_error_);

  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);

  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  error_sub_ = node_->create_subscription<nav2_msgs::msg::TrackingError>(
    error_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&IsFollowingPathCondition::onErrorReceived, this, std::placeholders::_1),
    sub_option);

  callback_group_executor_.spin_some(std::chrono::nanoseconds(1));
  initialized_ = true;
}

void IsFollowingPathCondition::onErrorReceived(const nav2_msgs::msg::TrackingError::SharedPtr msg)
{
  std::cout << "onErrorReceived called! tracking_error=" << msg->tracking_error << std::endl;
  last_tracking_error_ = msg->tracking_error;
  received_first_error_ = true;
}

BT::NodeStatus IsFollowingPathCondition::tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }
  callback_group_executor_.spin_some();

  if (!received_first_error_) {
    RCLCPP_INFO(node_->get_logger(), "Waiting for first tracking_error message...");
    // For a condition node, return FAILURE if no data yet
    return BT::NodeStatus::FAILURE;
  }

  // Optionally, update max_error_ if it can change at runtime
  getInput("max_error", max_error_);

  if (last_tracking_error_ <= max_error_) {
    std::cout << last_tracking_error_ << ", " << max_error_ << std::endl;
    RCLCPP_INFO(
      node_->get_logger(),
      "Path OK: current_error (%.2f) <= max_error (%.2f). Returning SUCCESS.",
      last_tracking_error_, max_error_);
    return BT::NodeStatus::SUCCESS;
  }

  std::cout << last_tracking_error_ << ", " << max_error_ << std::endl;
  RCLCPP_WARN(
    node_->get_logger(),
    "Path DIVERGED: current_error (%.2f) > max_error (%.2f). Returning FAILURE.",
    last_tracking_error_, max_error_);
  return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsFollowingPathCondition>("IsFollowingPath");
}
