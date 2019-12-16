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

#include <chrono>
#include <vector>
#include <memory>
#include <string>

#include "nav2_core/exceptions.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_controller/progress_checker.hpp"
#include "nav2_controller/nav2_controller.hpp"

using namespace std::chrono_literals;

namespace nav2_controller
{

ControllerServer::ControllerServer()
: LifecycleNode("controller_server", "", true),
  lp_loader_("nav2_core", "nav2_core::Controller")
{
  RCLCPP_INFO(get_logger(), "Creating controller server");

  declare_parameter("controller_frequency", 20.0);
  std::vector<std::string> default_id, default_type;
  default_type.push_back("dwb_core::DWBLocalPlanner");
  default_id.push_back("FollowPath");
  declare_parameter("controller_plugin_ids", default_id);
  declare_parameter("controller_plugin_types", default_type);

  // The costmap node is used in the implementation of the controller
  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "local_costmap", std::string{get_namespace()}, "local_costmap");

  // Launch a thread to run the costmap node
  costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);
}

ControllerServer::~ControllerServer()
{
  RCLCPP_INFO(get_logger(), "Destroying");
}

nav2_util::CallbackReturn
ControllerServer::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Configuring controller interface");

  get_parameter("controller_plugin_ids", controller_ids_);
  get_parameter("controller_plugin_types", controller_types_);
  get_parameter("controller_frequency", controller_frequency_);
  RCLCPP_INFO(get_logger(), "Controller frequency set to %.4fHz", controller_frequency_);

  costmap_ros_->on_configure(state);

  auto node = shared_from_this();

  progress_checker_ = std::make_unique<ProgressChecker>(rclcpp_node_);

  if (controller_types_.size() != controller_ids_.size()) {
    RCLCPP_FATAL(get_logger(), "Size of controller names (%i) and "
      "controller types (%i) are not the same!",
      static_cast<int>(controller_types_.size()),
      static_cast<int>(controller_ids_.size()));
    exit(-1);
  }

  for (uint i = 0; i != controller_types_.size(); i++) {
    try {
      nav2_core::Controller::Ptr controller =
        lp_loader_.createUniqueInstance(controller_types_[i]);
      RCLCPP_INFO(get_logger(), "Created controller : %s of type %s",
        controller_ids_[i].c_str(), controller_types_[i].c_str());
      controller->configure(node, controller_ids_[i],
        costmap_ros_->getTfBuffer(), costmap_ros_);
      controllers_.insert({controller_ids_[i], controller});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(get_logger(), "Failed to create controller. Exception: %s", ex.what());
    }
  }

  for (uint i = 0; i != controller_ids_.size(); i++) {
    controller_ids_concat_ += controller_ids_[i] + std::string(" ");
  }

  odom_sub_ = std::make_unique<nav_2d_utils::OdomSubscriber>(node);
  vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

  // Create the action server that we implement with our followPath method
  action_server_ = std::make_unique<ActionServer>(rclcpp_node_, "follow_path",
      std::bind(&ControllerServer::computeControl, this));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ControllerServer::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Activating");

  costmap_ros_->on_activate(state);
  ControllerMap::iterator it;
  for (it = controllers_.begin(); it != controllers_.end(); ++it) {
    it->second->activate();
  }
  vel_publisher_->on_activate();
  action_server_->activate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ControllerServer::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_->deactivate();
  ControllerMap::iterator it;
  for (it = controllers_.begin(); it != controllers_.end(); ++it) {
    it->second->deactivate();
  }
  costmap_ros_->on_deactivate(state);

  publishZeroVelocity();
  vel_publisher_->on_deactivate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ControllerServer::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  // Cleanup the helper classes
  ControllerMap::iterator it;
  for (it = controllers_.begin(); it != controllers_.end(); ++it) {
    it->second->cleanup();
  }
  controllers_.clear();
  costmap_ros_->on_cleanup(state);

  // Release any allocated resources
  action_server_.reset();
  for (it = controllers_.begin(); it != controllers_.end(); ++it) {
    it->second.reset();
  }
  odom_sub_.reset();

  vel_publisher_.reset();
  action_server_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ControllerServer::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_FATAL(get_logger(), "Lifecycle node entered error state");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ControllerServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void ControllerServer::computeControl()
{
  RCLCPP_INFO(get_logger(), "Received a goal, begin computing control effort.");

  try {
    std::string c_name = action_server_->get_current_goal()->controller_id;

    if (controllers_.find(c_name) == controllers_.end()) {
      if (controllers_.size() == 1 && c_name.empty()) {
        if (!single_controller_warning_given_) {
          RCLCPP_WARN(get_logger(), "No controller was specified in action call."
            " Server will use only plugin loaded %s. "
            "This warning will appear once.", controller_ids_concat_.c_str());
          single_controller_warning_given_ = true;
        }
        current_controller_ = controllers_.begin()->first;
      } else {
        RCLCPP_ERROR(get_logger(), "FollowPath called with controller name %s, "
          "which does not exist. Available controllers are %s.",
          c_name.c_str(), controller_ids_concat_.c_str());
        action_server_->terminate_current();
        return;
      }
    } else {
      current_controller_ = c_name;
    }

    setPlannerPath(action_server_->get_current_goal()->path);
    progress_checker_->reset();

    rclcpp::Rate loop_rate(controller_frequency_);
    while (rclcpp::ok()) {
      if (action_server_ == nullptr) {
        RCLCPP_DEBUG(get_logger(), "Action server unavailable. Stopping.");
        return;
      }

      if (!action_server_->is_server_active()) {
        RCLCPP_DEBUG(get_logger(), "Action server is inactive. Stopping.");
        return;
      }

      if (action_server_->is_cancel_requested()) {
        RCLCPP_INFO(get_logger(), "Goal was canceled. Stopping the robot.");
        action_server_->terminate_all();
        publishZeroVelocity();
        return;
      }

      updateGlobalPath();

      computeAndPublishVelocity();

      if (isGoalReached()) {
        RCLCPP_INFO(get_logger(), "Reached the goal!");
        break;
      }

      if (!loop_rate.sleep()) {
        RCLCPP_WARN(get_logger(), "Control loop missed its desired rate of %.4fHz",
          controller_frequency_);
      }
    }
  } catch (nav2_core::PlannerException & e) {
    RCLCPP_ERROR(this->get_logger(), e.what());
    publishZeroVelocity();
    action_server_->terminate_current();
    return;
  }

  RCLCPP_DEBUG(get_logger(), "DWB succeeded, setting result");

  publishZeroVelocity();

  // TODO(orduno) #861 Handle a pending preemption and set controller name
  action_server_->succeeded_current();
}

void ControllerServer::setPlannerPath(const nav_msgs::msg::Path & path)
{
  RCLCPP_DEBUG(get_logger(),
    "Providing path to the controller %s", current_controller_);
  if (path.poses.empty()) {
    throw nav2_core::PlannerException("Invalid path, Path is empty.");
  }
  controllers_[current_controller_]->setPlan(path);

  auto end_pose = *(path.poses.end() - 1);

  RCLCPP_DEBUG(get_logger(), "Path end point is (%.2f, %.2f)",
    end_pose.pose.position.x, end_pose.pose.position.y);
}

void ControllerServer::computeAndPublishVelocity()
{
  geometry_msgs::msg::PoseStamped pose;

  if (!getRobotPose(pose)) {
    throw nav2_core::PlannerException("Failed to obtain robot pose");
  }

  progress_checker_->check(pose);

  auto cmd_vel_2d =
    controllers_[current_controller_]->computeVelocityCommands(pose,
      nav_2d_utils::twist2Dto3D(odom_sub_->getTwist()));

  RCLCPP_DEBUG(get_logger(), "Publishing velocity at time %.2f", now().seconds());
  publishVelocity(cmd_vel_2d);
}

void ControllerServer::updateGlobalPath()
{
  if (action_server_->is_preempt_requested()) {
    RCLCPP_INFO(get_logger(), "Preempting the goal. Passing the new path to the planner.");
    setPlannerPath(action_server_->accept_pending_goal()->path);
  }
}

void ControllerServer::publishVelocity(const geometry_msgs::msg::TwistStamped & velocity)
{
  auto cmd_vel = velocity.twist;
  vel_publisher_->publish(cmd_vel);
}

void ControllerServer::publishZeroVelocity()
{
  geometry_msgs::msg::TwistStamped velocity;
  velocity.twist.angular.x = 0;
  velocity.twist.angular.y = 0;
  velocity.twist.angular.z = 0;
  velocity.twist.linear.x = 0;
  velocity.twist.linear.y = 0;
  velocity.twist.linear.z = 0;
  publishVelocity(velocity);
}

bool ControllerServer::isGoalReached()
{
  geometry_msgs::msg::PoseStamped pose;

  if (!getRobotPose(pose)) {
    return false;
  }

  geometry_msgs::msg::Twist velocity = nav_2d_utils::twist2Dto3D(odom_sub_->getTwist());
  return controllers_[current_controller_]->isGoalReached(pose, velocity);
}

bool ControllerServer::getRobotPose(geometry_msgs::msg::PoseStamped & pose)
{
  geometry_msgs::msg::PoseStamped current_pose;
  if (!costmap_ros_->getRobotPose(current_pose)) {
    RCLCPP_ERROR(this->get_logger(), "Could not get robot pose");
    return false;
  }
  pose = current_pose;
  return true;
}

}  // namespace nav2_controller
