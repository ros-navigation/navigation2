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

#include "dwb_core/common_types.hpp"
#include "dwb_core/dwb_local_planner.hpp"
#include "nav_2d_msgs/msg/pose2_d_stamped.hpp"
#include "nav_2d_utils/odom_subscriber.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav2_util/robot_utils.hpp"

namespace dwb_controller
{

class ProgressChecker;

class DwbController : public nav2_util::LifecycleNode
{
public:
  DwbController();
  ~DwbController();

protected:
  // The lifecycle interface
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

  using ActionServer = nav2_util::SimpleActionServer<nav2_msgs::action::FollowPath>;

  // Our action server implements the FollowPath action
  std::unique_ptr<ActionServer> action_server_;

  // The action server callback
  void followPath();

  void setPlannerPath(const nav2_msgs::msg::Path & path);
  void computeAndPublishVelocity();
  void updateGlobalPath();
  void publishVelocity(const nav_2d_msgs::msg::Twist2DStamped & velocity);
  void publishZeroVelocity();
  bool isGoalReached();
  bool getRobotPose(nav_2d_msgs::msg::Pose2DStamped & pose2d);

  // The DWBController contains a costmap node
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<std::thread> costmap_thread_;

  // Publishers and subscribers
  std::shared_ptr<nav_2d_utils::OdomSubscriber> odom_sub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;

  // The local planner
  std::unique_ptr<dwb_core::DWBLocalPlanner> planner_;

  // An executor used to spin the costmap node
  rclcpp::executors::SingleThreadedExecutor costmap_executor_;

  std::unique_ptr<ProgressChecker> progress_checker_;

  double controller_frequency_;
};

}  // namespace dwb_controller

#endif  // DWB_CONTROLLER__DWB_CONTROLLER_HPP_
