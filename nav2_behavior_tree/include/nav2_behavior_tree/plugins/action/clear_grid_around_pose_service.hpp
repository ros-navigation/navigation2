#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CLEAR_GRID_AROUND_POSE_SERVICE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CLEAR_GRID_AROUND_POSE_SERVICE_HPP_

#include <string>

#include "nav2_behavior_tree/bt_service_node.hpp"
#include "nav2_msgs/srv/clear_grid_around_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A nav2_behavior_tree::BtServiceNode class that wraps nav2_msgs::srv::ClearGridAroundPose
 */
class ClearGridAroundPoseService : public BtServiceNode<nav2_msgs::srv::ClearGridAroundPose>
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::ClearGridAroundPoseService
   * @param service_node_name Service name this node creates a client for
   * @param conf BT node configuration
   */
  ClearGridAroundPoseService(
    const std::string & service_node_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief The main override required by a BT service
   * @return BT::NodeStatus Status of tick execution
   */
  void on_tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<double>(
          "reset_distance", 1,
          "Distance from the robot under which obstacles are cleared"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>(
          "pose", "The pose to clear around"),
      });
  }
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CLEAR_GRID_AROUND_POSE_SERVICE_HPP_
