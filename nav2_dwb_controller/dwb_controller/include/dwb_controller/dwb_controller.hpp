// Copyright (c) 2018 Intel Corporation
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

#ifndef nav2_dwb_controller__DWB_CONTROLLER_HPP_
#define nav2_dwb_controller__DWB_CONTROLLER_HPP_

#include <memory>
#include <string>
#include "nav2_tasks/follow_path_task.hpp"
#include "dwb_core/dwb_core.h"
#include "dwb_core/common_types.h"
#include "nav_2d_msgs/msg/pose2_d_stamped.hpp"
#include "nav_2d_utils/odom_subscriber.h"

namespace nav2_dwb_controller
{

class DwbController : public nav2_tasks::FollowPathTaskServer
{
public:
  DwbController();
  ~DwbController();

  nav2_tasks::TaskStatus execute(const nav2_tasks::FollowPathCommand::SharedPtr path) override;

protected:
  bool isGoalReached(const nav_2d_msgs::msg::Pose2DStamped & pose2d);
  void publishVelocity(const nav_2d_msgs::msg::Twist2DStamped & velocity);
  void publishZeroVelocity();
  bool getRobotPose(nav_2d_msgs::msg::Pose2DStamped & pose2d);
  dwb_core::CostmapROSPtr cm_;
  dwb_core::DWBLocalPlanner planner_;
  std::shared_ptr<nav_2d_utils::OdomSubscriber> odom_sub_;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> vel_pub_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
};

}  // namespace nav2_dwb_controller

#endif  // nav2_dwb_controller__DWB_CONTROLLER_HPP_
