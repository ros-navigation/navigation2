#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__ARE_POSES_NEAR_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__ARE_POSES_NEAR_CONDITION_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "behaviortree_cpp/condition_node.h"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::ConditionNode that returns SUCCESS when a specified goal
 * is reached and FAILURE otherwise
 */
class ArePosesNear : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::ArePosesNearCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  ArePosesNear(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  ArePosesNear() = delete;

  /**
   * @brief A destructor for nav2_behavior_tree::ArePosesNearCondition
   */
  ~ArePosesNear() override = default;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Function to read parameters and initialize class variables
   */
  void initialize();

  /**
   * @brief Checks if the current robot pose lies within a given distance from the goal
   * @return bool true when goal is reached, false otherwise
   */
  bool arePosesNearby();

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("ref_pose", "Reference Pose"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose", "Target Pose to compare"),
      BT::InputPort<double>("tolerance", 0.5, "Distance tolerance to return success"),
      BT::InputPort<std::string>("global_frame", "Global frame ID for normalization (if needed)")
    };
  }

protected:
  
  // Pointers to the Nav2 environment
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // Internal variables
  std::string global_frame_;
  double transform_tolerance_;
  bool initialized_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__ARE_POSES_NEAR_CONDITION_HPP_