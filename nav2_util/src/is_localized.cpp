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

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/is_localized.hpp"
#include "nav2_robot/robot.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

namespace nav2_util
{

IsLocalized::IsLocalized()
{
  node_ = rclcpp::Node::make_shared("is_localized_class");
  robot_ = std::make_unique<nav2_robot::Robot>(node_);

  node_->get_parameter_or<double>("is_localized.x_tol", x_tol_, 0.25);
  node_->get_parameter_or<double>("is_localized.y_tol", y_tol_, 0.25);
  node_->get_parameter_or<double>("is_localized.rot_tol", rot_tol_, M_PI / 4);

}

bool
IsLocalized::isLocalized()
{
  auto current_pose = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();

  if (!robot_->getCurrentPose(current_pose)) {
    RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
    return false;
  }

  // Naive way to check if the robot has been localized
  // TODO(mhpanah): come up with a method to properly check particles convergence
  if (current_pose->pose.covariance[cov_x_] < x_tol_ &&
    current_pose->pose.covariance[cov_y_] < y_tol_ &&
    current_pose->pose.covariance[cov_a_] < rot_tol_)
  {
    return true;
  }

  return false;
}

}  // namespace nav2_util
