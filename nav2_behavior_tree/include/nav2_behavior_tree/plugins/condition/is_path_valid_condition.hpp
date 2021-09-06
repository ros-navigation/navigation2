#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__VALID_PATH_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__VALID_PATH_CONDITION_HPP_

#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"

namespace nav2_behavior_tree
{
/**
 * @brief A BT::ConditionNode that returns SUCCESS
 * if the path is still valid and returns FAILURE otherwise
 */
class ValidPathCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::ValidPathCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  ValidPathCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  ValidPathCondition() = delete;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "Path to follow")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<nav2_msgs::srv::IsPathValid>::SharedPtr client;
};
}

#endif //NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__VALID_PATH_CONDITION_HPP_
