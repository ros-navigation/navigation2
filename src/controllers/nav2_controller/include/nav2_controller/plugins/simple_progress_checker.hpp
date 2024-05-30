// Copyright (c) 2019 Intel Corporation
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

#ifndef NAV2_CONTROLLER__PLUGINS__SIMPLE_PROGRESS_CHECKER_HPP_
#define NAV2_CONTROLLER__PLUGINS__SIMPLE_PROGRESS_CHECKER_HPP_

#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/progress_checker.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

namespace nav2_controller
{
/**
* @class SimpleProgressChecker
* @brief This plugin is used to check the position of the robot to make sure
* that it is actually progressing towards a goal.
*/

class SimpleProgressChecker : public nav2_core::ProgressChecker
{
public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name) override;
  bool check(geometry_msgs::msg::PoseStamped & current_pose) override;
  void reset() override;

protected:
  /**
   * @brief Calculates robots movement from baseline pose
   * @param pose Current pose of the robot
   * @return true, if movement is greater than radius_, or false
   */
  bool isRobotMovedEnough(const geometry_msgs::msg::Pose2D & pose);
  /**
   * @brief Resets baseline pose with the current pose of the robot
   * @param pose Current pose of the robot
   */
  void resetBaselinePose(const geometry_msgs::msg::Pose2D & pose);

  static double pose_distance(
    const geometry_msgs::msg::Pose2D &,
    const geometry_msgs::msg::Pose2D &);

  rclcpp::Clock::SharedPtr clock_;

  double radius_;
  rclcpp::Duration time_allowance_{0, 0};

  geometry_msgs::msg::Pose2D baseline_pose_;
  rclcpp::Time baseline_time_;

  bool baseline_pose_set_{false};
  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::string plugin_name_;

  /**
   * @brief Callback executed when a paramter change is detected
   * @param parameters list of changed parameters
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
};
}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__PLUGINS__SIMPLE_PROGRESS_CHECKER_HPP_
