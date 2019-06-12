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

using namespace std::chrono_literals;

namespace dwb_controller
{

DwbController::DwbController()
: LifecycleNode("dwb_controller", "", true)
{
  RCLCPP_INFO(get_logger(), "Creating");

  // The costmap node is used in the implementation of the DWB controller
  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("local_costmap");

  // Create an executor that will be used to spin the costmap node
  costmap_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();

  // Launch a thread to run the costmap node
  costmap_thread_ = std::make_unique<std::thread>(
    [&](rclcpp_lifecycle::LifecycleNode::SharedPtr node)
    {
      // TODO(mjeronimo): Once Brian pushes his change upstream to rlcpp executors, we'll
      // be able to provide our own executor to spin(), reducing this to a single line
      costmap_executor_->add_node(node->get_node_base_interface());
      costmap_executor_->spin();
      costmap_executor_->remove_node(node->get_node_base_interface());
    }, costmap_ros_);
}

DwbController::~DwbController()
{
  RCLCPP_INFO(get_logger(), "Destroying");
  costmap_executor_->cancel();
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
      std::bind(&DwbController::followPath, this));

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

void DwbController::followPath()
{
  RCLCPP_INFO(get_logger(), "Received a goal, begin following path");

  try {
    setPlannerPath(action_server_->get_current_goal()->path);
    ProgressChecker progress_checker(rclcpp_node_);

    rclcpp::Rate loop_rate(100ms);
    while (rclcpp::ok()) {
      // Check if the action was cancelled by the client
      if (action_server_->is_cancelling_current_goal()) {
        cancelAndStop();
        return;
      }

      // Check if there is an update to the path to follow
      if (action_server_->preempt_requested()) {
        RCLCPP_DEBUG(get_logger(), "Preempting the goal. Passing the new path to the planner.");
        setPlannerPath(action_server_->accept_pending_goal()->path);
      }

      // Now the real work, compute the velocity commands
      if (!computeAndUpdate(progress_checker)) {
        RCLCPP_WARN(get_logger(), "Error during control computation. Stopping the robot");
        publishZeroVelocity();
      } else if (isGoalReached()) {
        RCLCPP_INFO(get_logger(), "Reached the goal!");
        break;
      }

      loop_rate.sleep();
    }
  } catch (nav_core2::PlannerException & e) {
    RCLCPP_ERROR(this->get_logger(), e.what());
    publishZeroVelocity();
    action_server_->abort_all();
    return;
  }

  RCLCPP_DEBUG(get_logger(), "DWB succeeded, setting result");
  action_server_->succeeded_current();
  processPendingPreemption();

  publishZeroVelocity();
}

void DwbController::setPlannerPath(const nav2_msgs::msg::Path & path)
{
  auto path2d = nav_2d_utils::pathToPath2D(path);

  RCLCPP_DEBUG(get_logger(), "Providing path to the local planner");
  planner_->setPlan(path2d);

  auto end_pose = *(path.poses.end() - 1);

  RCLCPP_DEBUG(get_logger(), "Path end point is (%.2f, %.2f)",
    end_pose.position.x, end_pose.position.y);
}

bool DwbController::computeAndUpdate(ProgressChecker & progress_checker)
{
  nav_2d_msgs::msg::Pose2DStamped pose2d;

  if (!getRobotPose(pose2d)) {
    return false;
  }

  progress_checker.check(pose2d);

  auto cmd_vel_2d = planner_->computeVelocityCommands(pose2d, odom_sub_->getTwist());

  RCLCPP_DEBUG(get_logger(), "Publishing velocity at time %.2f", now().seconds());
  publishVelocity(cmd_vel_2d);

  return true;
}

void DwbController::cancelAndStop()
{
  RCLCPP_INFO(this->get_logger(), "Canceling execution of the local planner");

  // Check if a new action is available
  if (action_server_->preempt_requested()) {
    // TODO(orduno): Assuming the cancel request applies for current goal and preempt request
    RCLCPP_WARN(this->get_logger(), "A preempt goal was available, we're cancelling all.");
  }

  action_server_->cancel_all();
  publishZeroVelocity();
}

void DwbController::processPendingPreemption()
{
  // TODO(orduno) What to do with a pending preemption if the controller has
  //              succeeded or failed on the current goal?
  //              Currently, the controller will abort the pending preempt.
  //              However, other options exist e.g. abort if paths have the same end pose
  //               or pursue if different end pose.
  if (action_server_->preempt_requested()) {
    RCLCPP_WARN(get_logger(),
      "A preempt request was made while controller reaching a final state (i.e. succeed, failed)"
      " on the current goal. Controller won't pursue this new goal and instead abort.");
    action_server_->abort_all();
  }
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

bool DwbController::isGoalReached()
{
  nav_2d_msgs::msg::Pose2DStamped pose2d;
  if (!getRobotPose(pose2d)) {
    return false;
  }

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
