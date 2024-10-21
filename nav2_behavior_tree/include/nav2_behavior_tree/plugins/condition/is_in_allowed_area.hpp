#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_IN_ALLOWED_AREA_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_IN_ALLOWED_AREA_HPP_

#include <string>
#include <atomic>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "std_msgs/msg/bool.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::ConditionNode that subscribes to cart_in_allowed_area topic and returns the result
 */
class IsInAllowedArea : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::IsInAllowedArea
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  IsInAllowedArea(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsInAllowedArea() = delete;

  /**
   * @brief A destructor for nav2_behavior_tree::IsInAllowedArea
   */
  ~IsInAllowedArea() override;

  /**
   * @brief Callback function for odom topic
   * @param msg Shared pointer to std_msgs::msg::Bool::SharedPtr message
   */
  void onCartInAllowedAreaReceived(const typename std_msgs::msg::Bool::SharedPtr msg);

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts() {return {
    BT::InputPort<std::string>("topic_name")
  };}

private:
  // The node that will be used for any ROS operations
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  std::thread callback_group_executor_thread;

  std::atomic<bool> is_in_allowed_area_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cart_in_allowed_area_sub_;

  std::string topic_name_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_IN_ALLOWED_AREA_HPP_
