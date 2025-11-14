// Copyright (c) 2025 Dexory

#ifndef NAV2_CONTROLLER__PLUGINS__AXIS_GOAL_CHECKER_HPP_
#define NAV2_CONTROLLER__PLUGINS__AXIS_GOAL_CHECKER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_core/goal_checker.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

namespace nav2_controller
{

/**  * @class AxisGoalChecker
  * @brief Goal Checker plugin that checks progress along the axis defined by the
  * 2 last poses of the pathalong the axis defined by the last segment of the path to the goal.
  *
  * This class can be configured to allow overshoot past the goal if the is_overshoot_valid
  *  parameter is set to true (which it is false by default).
  */
class AxisGoalChecker : public nav2_core::GoalChecker
{
public:
  AxisGoalChecker();
  // Standard GoalChecker Interface
  void initialize(
    const nav2::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void reset() override;
  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
    const std::optional<geometry_msgs::msg::Pose> & before_goal_pose,
    const geometry_msgs::msg::Twist & velocity) override;
  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance) override;

protected:
  double segment_axis_goal_tolerance_;
  bool is_overshoot_valid_;
  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::string plugin_name_;

  /**
   * @brief Callback executed when a parameter change is detected
   * @param parameters list of changed parameters
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
};

}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__PLUGINS__AXIS_GOAL_CHECKER_HPP_
