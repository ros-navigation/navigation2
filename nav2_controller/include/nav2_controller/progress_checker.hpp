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

#ifndef NAV2_CONTROLLER__PROGRESS_CHECKER_HPP_
#define NAV2_CONTROLLER__PROGRESS_CHECKER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

namespace nav2_controller
{

class ProgressChecker
{
public:
  explicit ProgressChecker(const rclcpp::Node::SharedPtr & node);
  void check(geometry_msgs::msg::PoseStamped & current_pose);
  void reset() {baseline_pose_set_ = false;}

protected:
  bool is_robot_moved_enough(const geometry_msgs::msg::Pose2D & pose);
  void reset_baseline_pose(const geometry_msgs::msg::Pose2D & pose);

  rclcpp::Node::SharedPtr nh_;

  double radius_;
  rclcpp::Duration time_allowance_{0, 0};

  geometry_msgs::msg::Pose2D baseline_pose_;
  rclcpp::Time baseline_time_;

  bool baseline_pose_set_{false};
};
}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__PROGRESS_CHECKER_HPP_
