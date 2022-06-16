#pragma once

#include <string>
#include <vector>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_bt_navigator/navigator.hpp"
#include "cmr_msgs/action/navigate_fleet_nodes.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/odometry_utils.hpp"

namespace nav2_bt_navigator
{

/**
 * @class NavigateFleetNodesNavigator
 * @brief A navigator for navigating to a a bunch of intermediary poses
 */
class NavigateFleetNodesNavigator
  : public nav2_bt_navigator::Navigator<cmr_msgs::action::NavigateFleetNodes>
{
public:
  using ActionT = cmr_msgs::action::NavigateFleetNodes;
  typedef std::vector<geometry_msgs::msg::PoseStamped> Goals;

  /**
   * @brief A constructor for NavigateFleetNodesNavigator
   */
  NavigateFleetNodesNavigator()
  : Navigator() {}

  /**
   * @brief A configure state transition to configure navigator's state
   * @param node Weakptr to the lifecycle node
   * @param odom_smoother Object to get current smoothed robot's speed
   */
  bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr node,
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) override;

  /**
   * @brief Get action name for this navigator
   * @return string Name of action server
   */
  std::string getName() override {return std::string("navigate_fleet_nodes");}

  /**
   * @brief Get navigator's default BT
   * @param node WeakPtr to the lifecycle node
   * @return string Filepath to default XML
   */
  std::string getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;

protected:
  /**
   * @brief A callback to be called when a new goal is received by the BT action server
   * Can be used to check if goal is valid and put values on
   * the blackboard which depend on the received goal
   * @param goal Action template's goal message
   * @return bool if goal was received successfully to be processed
   */
  bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override;

  /**
   * @brief A callback that defines execution that happens on one iteration through the BT
   * Can be used to publish action feedback
   */
  void onLoop() override;

  /**
   * @brief A callback that is called when a preempt is requested
   */
  void onPreempt(ActionT::Goal::ConstSharedPtr goal) override;

  /**
   * @brief A callback that is called when a the action is completed, can fill in
   * action result message or indicate that this action is done.
   * @param result Action template result message to populate
   */
  void goalCompleted(typename ActionT::Result::SharedPtr result) override;

  /**
   * @brief Goal pose initialization on the blackboard
   */
  void initializeGoalPoses(ActionT::Goal::ConstSharedPtr goal);

  rclcpp::Time start_time_;
  std::string goals_blackboard_id_;
  std::string path_blackboard_id_;

  // Odometry smoother object
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother_;
};

}  // namespace nav2_bt_navigator

