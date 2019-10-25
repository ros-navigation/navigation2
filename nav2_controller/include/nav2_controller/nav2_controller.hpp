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

#ifndef NAV2_CONTROLLER__NAV2_CONTROLLER_HPP_
#define NAV2_CONTROLLER__NAV2_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

#include "nav2_core/local_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/transform_listener.h"
#include "nav2_msgs/action/compute_control.hpp"
#include "nav_2d_utils/odom_subscriber.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav2_util/robot_utils.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_controller
{

class ProgressChecker;

class ControllerServer : public nav2_util::LifecycleNode
{
public:
  typedef std::unordered_map<std::string, nav2_core::LocalPlanner::Ptr> ControllerMap;
  ControllerServer();
  ~ControllerServer();

protected:
  // The lifecycle interface
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

  using ActionServer = nav2_util::SimpleActionServer<nav2_msgs::action::ComputeControl>;

  // Our action server implements the ComputeControl action
  std::unique_ptr<ActionServer> action_server_;

  // The action server callback
  void computeControl();

  void setPlannerPath(const nav_msgs::msg::Path & path);
  void computeAndPublishVelocity();
  void updateGlobalPath();
  void publishVelocity(const geometry_msgs::msg::TwistStamped & velocity);
  void publishZeroVelocity();
  bool isGoalReached();
  bool getRobotPose(geometry_msgs::msg::PoseStamped & pose);

  // The local controller needs a costmap node
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<nav2_util::NodeThread> costmap_thread_;

  // Publishers and subscribers
  std::unique_ptr<nav_2d_utils::OdomSubscriber> odom_sub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;

  // Controller Plugins
  pluginlib::ClassLoader<nav2_core::LocalPlanner> lp_loader_;
  ControllerMap controllers_;
  std::vector<std::string> controller_names_, controller_types_;
  std::string controller_names_concat_, current_controller_;

  std::unique_ptr<ProgressChecker> progress_checker_;

  double controller_frequency_;

  // Whether we've published the single controller warning yet
  bool single_controller_warning_given_{false};
};

}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__NAV2_CONTROLLER_HPP_
