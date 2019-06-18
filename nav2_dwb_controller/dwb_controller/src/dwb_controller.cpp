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

#include "dwb_controller/dwb_controller.hpp"

#include <chrono>
#include <memory>
#include <string>

#include "dwb_core/exceptions.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "nav2_util/node_utils.hpp"
#include "dwb_controller/progress_checker.hpp"

using namespace std::chrono_literals;

namespace dwb_controller
{

DwbController::DwbController()
: LifecycleNode("dwb_controller", "", true)
{
  RCLCPP_INFO(get_logger(), "Creating");

  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("local_costmap");
  costmap_thread_ = std::make_unique<std::thread>(
    [](rclcpp_lifecycle::LifecycleNode::SharedPtr node)
    {rclcpp::spin(node->get_node_base_interface());}, costmap_ros_);
}

DwbController::~DwbController()
{
  RCLCPP_INFO(get_logger(), "Destroying");
  costmap_thread_->join();
}

nav2_util::CallbackReturn
DwbController::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  costmap_ros_->on_configure(state);

  auto node = shared_from_this();

  planner_ = std::make_unique<dwb_core::DWBLocalPlanner>(
    node, costmap_ros_->getTfBuffer(), costmap_ros_);
  planner_->on_configure(state);

  odom_sub_ = std::make_shared<nav_2d_utils::OdomSubscriber>(*this);
  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

  // Create the action server that we implement with our followPath method
  action_server_ = std::make_unique<ActionServer>(rclcpp_node_, "FollowPath",
      std::bind(&DwbController::followPath, this, std::placeholders::_1));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DwbController::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Activating");

  planner_->on_activate(state);
  costmap_ros_->on_activate(state);
  vel_pub_->on_activate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DwbController::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  planner_->on_deactivate(state);
  costmap_ros_->on_deactivate(state);
  vel_pub_->on_deactivate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DwbController::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  // Cleanup the helper classes
  planner_->on_cleanup(state);
  costmap_ros_->on_cleanup(state);

  // Release any allocated resources
  planner_.reset();
  odom_sub_.reset();
  vel_pub_.reset();
  action_server_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DwbController::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_FATAL(get_logger(), "Lifecycle node entered error state");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DwbController::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void
DwbController::followPath(const std::shared_ptr<GoalHandle> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Received a goal, begin following path");
  auto result = std::make_shared<nav2_msgs::action::FollowPath::Result>();

  std::shared_ptr<GoalHandle> current_goal_handle = goal_handle;

  rclcpp::Rate loop_rate(100ms);    // period vs. hz

  auto goal = current_goal_handle->get_goal();

  try {
    auto path = nav_2d_utils::pathToPath2D(goal->path);

    RCLCPP_DEBUG(get_logger(), "Providing path to the local planner");
    planner_->setPlan(path);

    ProgressChecker progress_checker(rclcpp_node_);

    rclcpp::Rate loop_rate(10);
    while (rclcpp::ok()) {
      nav_2d_msgs::msg::Pose2DStamped pose2d;
      if (!getRobotPose(pose2d)) {
        RCLCPP_INFO(get_logger(), "No pose. Stopping the robot");
        publishZeroVelocity();
      } else {
        if (isGoalReached(pose2d)) {
          RCLCPP_INFO(get_logger(), "Reached the goal");
          break;
        }
        progress_checker.check(pose2d);
        auto velocity = odom_sub_->getTwist();
        auto cmd_vel_2d = planner_->computeVelocityCommands(pose2d, velocity);

        RCLCPP_DEBUG(get_logger(), "Publishing velocity at time %.2f", now().seconds());
        publishVelocity(cmd_vel_2d);

        if (current_goal_handle->is_canceling()) {
          RCLCPP_INFO(this->get_logger(), "Canceling execution of the local planner");
          current_goal_handle->canceled(result);
          publishZeroVelocity();
          return;
        }

        // Check if there is an update to the path to follow
        if (action_server_->preempt_requested()) {
          RCLCPP_DEBUG(get_logger(), "Received a new goal, pre-empting the old one");
          current_goal_handle = action_server_->get_updated_goal_handle();
          goal = current_goal_handle->get_goal();

          // If so, pass the new path to the local planner
          auto path = nav_2d_utils::pathToPath2D(goal->path);
          planner_->setPlan(path);
        }
      }

      loop_rate.sleep();
    }
  } catch (nav_core2::PlannerException & e) {
    RCLCPP_ERROR(this->get_logger(), e.what());
    publishZeroVelocity();
    current_goal_handle->abort(result);
    return;
  }

  RCLCPP_DEBUG(get_logger(), "DWB succeeded, setting result");
  current_goal_handle->succeed(result);
  publishZeroVelocity();
}

void DwbController::publishVelocity(const nav_2d_msgs::msg::Twist2DStamped & velocity)
{
  auto cmd_vel = nav_2d_utils::twist2Dto3D(velocity.velocity);
  vel_pub_->publish(cmd_vel);
}

void DwbController::publishZeroVelocity()
{
  nav_2d_msgs::msg::Twist2DStamped velocity;
  velocity.velocity.x = 0;
  velocity.velocity.y = 0;
  velocity.velocity.theta = 0;

  publishVelocity(velocity);
}

bool DwbController::isGoalReached(const nav_2d_msgs::msg::Pose2DStamped & pose2d)
{
  nav_2d_msgs::msg::Twist2D velocity = odom_sub_->getTwist();
  return planner_->isGoalReached(pose2d, velocity);
}

bool DwbController::getRobotPose(nav_2d_msgs::msg::Pose2DStamped & pose2d)
{
  geometry_msgs::msg::PoseStamped current_pose;
  if (!costmap_ros_->getRobotPose(current_pose)) {
    RCLCPP_ERROR(this->get_logger(), "Could not get robot pose");
    return false;
  }
  pose2d = nav_2d_utils::poseStampedToPose2D(current_pose);
  return true;
}

}  // namespace dwb_controller
