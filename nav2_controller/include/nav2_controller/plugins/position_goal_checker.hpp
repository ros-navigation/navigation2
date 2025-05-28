// Copyright (c) 2025 Prabhav Saxena
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_CONTROLLER__PLUGINS__POSITION_GOAL_CHECKER_HPP_
#define NAV2_CONTROLLER__PLUGINS__POSITION_GOAL_CHECKER_HPP_

#include <string>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace nav2_controller
{

/**
     * @class PositionGoalChecker
     * @brief Goal Checker plugin that only checks XY position, ignoring orientation
     */
class PositionGoalChecker : public nav2_core::GoalChecker
{
public:
  PositionGoalChecker();
  ~PositionGoalChecker() override = default;

  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void reset() override;

  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) override;

  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance) override;

  /**
       * @brief Set the XY goal tolerance
       * @param tolerance New tolerance value
       */
  void setXYGoalTolerance(double tolerance);

protected:
  double xy_goal_tolerance_;
  double xy_goal_tolerance_sq_;
  bool stateful_;
  bool position_reached_;
  std::string plugin_name_;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  /**
       * @brief Callback executed when a parameter change is detected
       * @param parameters list of changed parameters
       */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
};

}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__PLUGINS__POSITION_GOAL_CHECKER_HPP_
