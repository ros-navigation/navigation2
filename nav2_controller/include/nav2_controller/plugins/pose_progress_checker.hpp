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
#include "nav2_ros_common/lifecycle_node.hpp"

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
  /**
   * @brief Construct a new Pose Progress Checker object
   */
  PoseProgressChecker() = default;

  /**
   * @brief Destroy the Pose Progress Checker object
   */
  ~PoseProgressChecker();

  /**
   * @brief Initialize the goal checker
   * @param parent Weak pointer to the lifecycle node
   * @param plugin_name Name of the plugin
   */
  void initialize(
    const nav2::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name) override;

  /**
   * @brief Checks if the robot has moved compare to previous
   * @param current_pose Current pose of the robot
   * @return true, if the robot has moved enough, false otherwise
   */
  bool check(geometry_msgs::msg::PoseStamped & current_pose) override;

protected:
  /**
   * @brief Calculates robots movement from baseline pose
   * @param pose Current pose of the robot
   * @return true, if movement is greater than radius_, or false
   */
  bool isRobotMovedEnough(const geometry_msgs::msg::Pose & pose);

  /**
   * @brief Calculates angle difference between two poses
   * @param pose1 First pose
   * @param pose2 Second pose
   * @return Angle difference in radians
   */
  static double poseAngleDistance(
    const geometry_msgs::msg::Pose &,
    const geometry_msgs::msg::Pose &);

  double required_movement_angle_;

  // Dynamic parameters handler
  std::mutex mutex_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_params_handler_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handler_;
  std::string plugin_name_;
  nav2::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("pose_progress_checker")};

  /**
   * @brief Validate incoming parameter updates before applying them.
   * This callback is triggered when one or more parameters are about to be updated.
   * It checks the validity of parameter values and rejects updates that would lead
   * to invalid or inconsistent configurations
   * @param parameters List of parameters that are being updated.
   * @return rcl_interfaces::msg::SetParametersResult Result indicating whether the update is accepted.
   */
  rcl_interfaces::msg::SetParametersResult validateParameterUpdatesCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  /**
   * @brief Apply parameter updates after validation
   * This callback is executed when parameters have been successfully updated.
   * It updates the internal configuration of the node with the new parameter values.
   * @param parameters List of parameters that have been updated.
   */
  void updateParametersCallback(const std::vector<rclcpp::Parameter> & parameters);
};
}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__PLUGINS__POSE_PROGRESS_CHECKER_HPP_
