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

#include "dwb_controller/progress_checker.hpp"
#include <cmath>
#include "dwb_core/exceptions.hpp"

namespace dwb_controller
{
static double pose_distance(const geometry_msgs::msg::Pose2D &, const geometry_msgs::msg::Pose2D &);

ProgressChecker::ProgressChecker(const rclcpp::Node::SharedPtr & node)
: nh_(node)
{
  // Scale is set to 0 by default, so if it was not set otherwise, set to 0
  nh_->get_parameter_or("required_movement_radius", radius_, 0.5);
  double time_allowance_param;
  nh_->get_parameter_or("movement_time_allowance_", time_allowance_param, 10.0);
  time_allowance_ = rclcpp::Duration::from_seconds(time_allowance_param);
}

void ProgressChecker::check(nav_2d_msgs::msg::Pose2DStamped & current_pose)
{
  // relies on short circuit evaluation to not call is_robot_moved_enough if
  // baseline_pose is not set.
  if ((!baseline_pose_set_) || (is_robot_moved_enough(current_pose.pose))) {
    reset_baseline_pose(current_pose.pose);
    return;
  }
  if ((nh_->now() - baseline_time_) > time_allowance_) {
    throw nav_core2::PlannerException("Failed to make progress");
  }
}

void ProgressChecker::reset_baseline_pose(const geometry_msgs::msg::Pose2D & pose)
{
  baseline_pose_ = pose;
  baseline_time_ = nh_->now();
  baseline_pose_set_ = true;
}

bool ProgressChecker::is_robot_moved_enough(const geometry_msgs::msg::Pose2D & pose)
{
  if (pose_distance(pose, baseline_pose_) > radius_) {
    return true;
  } else {
    return false;
  }
}

static double pose_distance(
  const geometry_msgs::msg::Pose2D & pose1,
  const geometry_msgs::msg::Pose2D & pose2)
{
  double dx = pose1.x - pose2.x;
  double dy = pose1.y - pose2.y;

  return std::hypot(dx, dy);
}

}  // namespace dwb_controller
