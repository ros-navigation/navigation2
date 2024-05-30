// Copyright (c) 2023 Dexory
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

#ifndef NAV2_CONTROLLER__PLUGINS__POSE_PROGRESS_CHECKER_HPP_
#define NAV2_CONTROLLER__PLUGINS__POSE_PROGRESS_CHECKER_HPP_

#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav2_controller/plugins/simple_progress_checker.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_controller
{
/**
* @class PoseProgressChecker
* @brief This plugin is used to check the position and the angle of the robot to make sure
* that it is actually progressing or rotating towards a goal.
*/

class PoseProgressChecker : public SimpleProgressChecker
{
public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name) override;
  bool check(geometry_msgs::msg::PoseStamped & current_pose) override;

protected:
  /**
   * @brief Calculates robots movement from baseline pose
   * @param pose Current pose of the robot
   * @return true, if movement is greater than radius_, or false
   */
  bool isRobotMovedEnough(const geometry_msgs::msg::Pose2D & pose);

  static double poseAngleDistance(
    const geometry_msgs::msg::Pose2D &,
    const geometry_msgs::msg::Pose2D &);

  double required_movement_angle_;

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

#endif  // NAV2_CONTROLLER__PLUGINS__POSE_PROGRESS_CHECKER_HPP_
