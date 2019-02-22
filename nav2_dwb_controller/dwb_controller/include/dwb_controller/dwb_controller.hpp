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

#ifndef DWB_CONTROLLER__DWB_CONTROLLER_HPP_
#define DWB_CONTROLLER__DWB_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <thread>

#include "nav2_tasks/follow_path_task.hpp"
#include "nav2_lifecycle/lifecycle_node.hpp"
#include "dwb_core/dwb_local_planner.hpp"
#include "dwb_core/common_types.hpp"
#include "nav_2d_msgs/msg/pose2_d_stamped.hpp"
#include "nav_2d_utils/odom_subscriber.hpp"

namespace nav2_dwb_controller
{

class DwbController : public nav2_lifecycle::LifecycleNode
{
public:
  DwbController();
  ~DwbController();

protected:
  nav2_lifecycle::CallbackReturn onConfigure(const rclcpp_lifecycle::State & state) override;
  nav2_lifecycle::CallbackReturn onActivate(const rclcpp_lifecycle::State & state) override;
  nav2_lifecycle::CallbackReturn onDeactivate(const rclcpp_lifecycle::State & state) override;
  nav2_lifecycle::CallbackReturn onCleanup(const rclcpp_lifecycle::State & state) override;

  // This module is a task server that implements the FollowPath task
  std::unique_ptr<nav2_tasks::FollowPathTaskServer> task_server_;
  nav2_tasks::TaskStatus followPath(const nav2_tasks::FollowPathCommand::SharedPtr path);

  bool isGoalReached(const nav_2d_msgs::msg::Pose2DStamped & pose2d);
  void publishVelocity(const nav_2d_msgs::msg::Twist2DStamped & velocity);
  void publishZeroVelocity();
  bool getRobotPose(nav_2d_msgs::msg::Pose2DStamped & pose2d);

  // It uses a costmap node
  dwb_core::CostmapROSPtr costmap_ros_;
  std::unique_ptr<std::thread> costmap_thread_;

  // Publishers and subscribers
  std::shared_ptr<nav_2d_utils::OdomSubscriber> odom_sub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

  std::unique_ptr<dwb_core::DWBLocalPlanner> planner_;
};

}  // namespace nav2_dwb_controller

#endif  // DWB_CONTROLLER__DWB_CONTROLLER_HPP_
