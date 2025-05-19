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
#include <memory>
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
  struct Parameters
  {
    double required_movement_angle;
  };

/**
 * @class nav2_controller::PoseProgressChecker::ParameterHandler
 * @brief This class handls parameters and dynamic parameter updates for the nav2_controller.
 */
  class ParameterHandler
  {
  public:
    ParameterHandler(
      rclcpp_lifecycle::LifecycleNode::SharedPtr node,
      std::string & plugin_name, rclcpp::Logger & logger);
    ~ParameterHandler();
    Parameters * getParams() {return &params_;}

  protected:
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

    void
    updateParametersCallback(
      std::vector<rclcpp::Parameter> parameters);

    rcl_interfaces::msg::SetParametersResult
    validateParameterUpdatesCallback(
      std::vector<rclcpp::Parameter> parameters);
    rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_params_handler_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handler_;
    Parameters params_;
    std::string plugin_name_;
    rclcpp::Logger logger_ {rclcpp::get_logger("PoseProgressChecker")};
  };

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
  Parameters * params_;
  std::string plugin_name_;

  rclcpp::Logger logger_ {rclcpp::get_logger("PoseProgressChecker")};
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::unique_ptr<nav2_controller::PoseProgressChecker::ParameterHandler> param_handler_;
};

}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__PLUGINS__POSE_PROGRESS_CHECKER_HPP_
