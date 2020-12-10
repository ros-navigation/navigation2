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
#include <utility>

#include "nav2_core/exceptions.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "nav_2d_utils/tf_help.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_controller/nav2_controller.hpp"

using namespace std::chrono_literals;

namespace nav2_controller
{

ControllerServer::ControllerServer()
: LifecycleNode("controller_server", "", true),
  progress_checker_loader_("nav2_core", "nav2_core::ProgressChecker"),
  default_progress_checker_id_{"progress_checker"},
  default_progress_checker_type_{"nav2_controller::SimpleProgressChecker"},
  goal_checker_loader_("nav2_core", "nav2_core::GoalChecker"),
  default_goal_checker_id_{"goal_checker"},
  default_goal_checker_type_{"nav2_controller::SimpleGoalChecker"},
  lp_loader_("nav2_core", "nav2_core::Controller"),
  default_ids_{"FollowPath"},
  default_types_{"dwb_core::DWBLocalPlanner"}
{
  RCLCPP_INFO(get_logger(), "Creating controller server");

  declare_parameter("controller_frequency", 20.0);

  declare_parameter("progress_checker_plugin", default_progress_checker_id_);
  declare_parameter("goal_checker_plugin", default_goal_checker_id_);
  declare_parameter("controller_plugins", default_ids_);
  declare_parameter("min_x_velocity_threshold", rclcpp::ParameterValue(0.0001));
  declare_parameter("min_y_velocity_threshold", rclcpp::ParameterValue(0.0001));
  declare_parameter("min_theta_velocity_threshold", rclcpp::ParameterValue(0.0001));

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
  auto node = shared_from_this();

  RCLCPP_INFO(get_logger(), "Configuring controller interface");

  get_parameter("progress_checker_plugin", progress_checker_id_);
  if (progress_checker_id_ == default_progress_checker_id_) {
    nav2_util::declare_parameter_if_not_declared(
      node, default_progress_checker_id_ + ".plugin",
      rclcpp::ParameterValue(default_progress_checker_type_));
  }
  get_parameter("goal_checker_plugin", goal_checker_id_);
  if (goal_checker_id_ == default_goal_checker_id_) {
    nav2_util::declare_parameter_if_not_declared(
      node, default_goal_checker_id_ + ".plugin",
      rclcpp::ParameterValue(default_goal_checker_type_));
  }

  get_parameter("controller_plugins", controller_ids_);
  if (controller_ids_ == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      nav2_util::declare_parameter_if_not_declared(
        node, default_ids_[i] + ".plugin",
        rclcpp::ParameterValue(default_types_[i]));
    }
  }
  controller_types_.resize(controller_ids_.size());

  get_parameter("controller_frequency", controller_frequency_);
  get_parameter("min_x_velocity_threshold", min_x_velocity_threshold_);
  get_parameter("min_y_velocity_threshold", min_y_velocity_threshold_);
  get_parameter("min_theta_velocity_threshold", min_theta_velocity_threshold_);
  RCLCPP_INFO(get_logger(), "Controller frequency set to %.4fHz", controller_frequency_);

  costmap_ros_->on_configure(state);

  try {
    progress_checker_type_ = nav2_util::get_plugin_type_param(node, progress_checker_id_);
    progress_checker_ = progress_checker_loader_.createUniqueInstance(progress_checker_type_);
    RCLCPP_INFO(
      get_logger(), "Created progress_checker : %s of type %s",
      progress_checker_id_.c_str(), progress_checker_type_.c_str());
    progress_checker_->initialize(node, progress_checker_id_);
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_FATAL(
      get_logger(),
      "Failed to create progress_checker. Exception: %s", ex.what());
    return nav2_util::CallbackReturn::FAILURE;
  }
  try {
    goal_checker_type_ = nav2_util::get_plugin_type_param(node, goal_checker_id_);
    goal_checker_ = goal_checker_loader_.createUniqueInstance(goal_checker_type_);
    RCLCPP_INFO(
      get_logger(), "Created goal_checker : %s of type %s",
      goal_checker_id_.c_str(), goal_checker_type_.c_str());
    goal_checker_->initialize(node, goal_checker_id_);
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_FATAL(
      get_logger(),
      "Failed to create goal_checker. Exception: %s", ex.what());
    return nav2_util::CallbackReturn::FAILURE;
  }

  for (size_t i = 0; i != controller_ids_.size(); i++) {
    try {
      controller_types_[i] = nav2_util::get_plugin_type_param(node, controller_ids_[i]);
      nav2_core::Controller::Ptr controller =
        lp_loader_.createUniqueInstance(controller_types_[i]);
      RCLCPP_INFO(
        get_logger(), "Created controller : %s of type %s",
        controller_ids_[i].c_str(), controller_types_[i].c_str());
      controller->configure(
        node, controller_ids_[i],
        costmap_ros_->getTfBuffer(), costmap_ros_);
      controllers_.insert({controller_ids_[i], controller});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(),
        "Failed to create controller. Exception: %s", ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  for (size_t i = 0; i != controller_ids_.size(); i++) {
    controller_ids_concat_ += controller_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(
    get_logger(),
    "Controller Server has %s controllers available.", controller_ids_concat_.c_str());

  odom_sub_ = std::make_unique<nav_2d_utils::OdomSubscriber>(node);
  vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

  // Create the action server that we implement with our followPath method
  action_server_ = std::make_unique<ActionServer>(
    rclcpp_node_, "follow_path",
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
  odom_sub_.reset();
  vel_publisher_.reset();
  action_server_.reset();
  goal_checker_->reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ControllerServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

bool ControllerServer::findControllerId(
  const std::string & c_name,
  std::string & current_controller)
{
  if (controllers_.find(c_name) == controllers_.end()) {
    if (controllers_.size() == 1 && c_name.empty()) {
      RCLCPP_WARN_ONCE(
        get_logger(), "No controller was specified in action call."
        " Server will use only plugin loaded %s. "
        "This warning will appear once.", controller_ids_concat_.c_str());
      current_controller = controllers_.begin()->first;
    } else {
      RCLCPP_ERROR(
        get_logger(), "FollowPath called with controller name %s, "
        "which does not exist. Available controllers are %s.",
        c_name.c_str(), controller_ids_concat_.c_str());
      return false;
    }
  } else {
    RCLCPP_DEBUG(get_logger(), "Selected controller: %s.", c_name.c_str());
    current_controller = c_name;
  }

  return true;
}

void ControllerServer::computeControl()
{
  RCLCPP_INFO(get_logger(), "Received a goal, begin computing control effort.");

  try {
    std::string c_name = action_server_->get_current_goal()->controller_id;
    std::string current_controller;
    if (findControllerId(c_name, current_controller)) {
      current_controller_ = current_controller;
    } else {
      action_server_->terminate_current();
      return;
    }

    setPlannerPath(action_server_->get_current_goal()->path);
    progress_checker_->reset();

    rclcpp::WallRate loop_rate(controller_frequency_);
    while (rclcpp::ok()) {
      if (action_server_ == nullptr || !action_server_->is_server_active()) {
        RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
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
        RCLCPP_WARN(
          get_logger(), "Control loop missed its desired rate of %.4fHz",
          controller_frequency_);
      }
    }
  } catch (nav2_core::PlannerException & e) {
    RCLCPP_ERROR(this->get_logger(), e.what());
    publishZeroVelocity();
    action_server_->terminate_current();
    return;
  }

  RCLCPP_DEBUG(get_logger(), "Controller succeeded, setting result");

  publishZeroVelocity();

  // TODO(orduno) #861 Handle a pending preemption and set controller name
  action_server_->succeeded_current();
}

void ControllerServer::setPlannerPath(const nav_msgs::msg::Path & path)
{
  RCLCPP_DEBUG(
    get_logger(),
    "Providing path to the controller %s", current_controller_.c_str());
  if (path.poses.empty()) {
    throw nav2_core::PlannerException("Invalid path, Path is empty.");
  }
  controllers_[current_controller_]->setPlan(path);

  auto end_pose = path.poses.back();
  end_pose.header.frame_id = path.header.frame_id;
  rclcpp::Duration tolerance(costmap_ros_->getTransformTolerance() * 1e9);
  nav_2d_utils::transformPose(
    costmap_ros_->getTfBuffer(), costmap_ros_->getGlobalFrameID(),
    end_pose, end_pose, tolerance);
  goal_checker_->reset();

  RCLCPP_DEBUG(
    get_logger(), "Path end point is (%.2f, %.2f)",
    end_pose.pose.position.x, end_pose.pose.position.y);
  end_pose_ = end_pose.pose;
}

void ControllerServer::computeAndPublishVelocity()
{
  geometry_msgs::msg::PoseStamped pose;

  if (!getRobotPose(pose)) {
    throw nav2_core::PlannerException("Failed to obtain robot pose");
  }

  if (!progress_checker_->check(pose)) {
    throw nav2_core::PlannerException("Failed to make progress");
  }

  nav_2d_msgs::msg::Twist2D twist = getThresholdedTwist(odom_sub_->getTwist());

  auto cmd_vel_2d =
    controllers_[current_controller_]->computeVelocityCommands(
    pose,
    nav_2d_utils::twist2Dto3D(twist));

  std::shared_ptr<Action::Feedback> feedback = std::make_shared<Action::Feedback>();
  feedback->speed = std::hypot(cmd_vel_2d.twist.linear.x, cmd_vel_2d.twist.linear.y);
  feedback->distance_to_goal = nav2_util::geometry_utils::euclidean_distance(end_pose_, pose.pose);
  action_server_->publish_feedback(feedback);

  RCLCPP_DEBUG(get_logger(), "Publishing velocity at time %.2f", now().seconds());
  publishVelocity(cmd_vel_2d);
}

void ControllerServer::updateGlobalPath()
{
  if (action_server_->is_preempt_requested()) {
    RCLCPP_INFO(get_logger(), "Passing new path to controller.");
    auto goal = action_server_->accept_pending_goal();
    std::string current_controller;
    if (findControllerId(goal->controller_id, current_controller)) {
      current_controller_ = current_controller;
    } else {
      RCLCPP_INFO(
        get_logger(), "Terminating action, invalid controller %s requested.",
        goal->controller_id.c_str());
      action_server_->terminate_current();
      return;
    }
    setPlannerPath(goal->path);
  }
}

void ControllerServer::publishVelocity(const geometry_msgs::msg::TwistStamped & velocity)
{
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>(velocity.twist);
  if (
    vel_publisher_->is_activated() &&
    this->count_subscribers(vel_publisher_->get_topic_name()) > 0)
  {
    vel_publisher_->publish(std::move(cmd_vel));
  }
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
  velocity.header.frame_id = costmap_ros_->getBaseFrameID();
  velocity.header.stamp = now();
  publishVelocity(velocity);
}

bool ControllerServer::isGoalReached()
{
  geometry_msgs::msg::PoseStamped pose;

  if (!getRobotPose(pose)) {
    return false;
  }

  nav_2d_msgs::msg::Twist2D twist = getThresholdedTwist(odom_sub_->getTwist());
  geometry_msgs::msg::Twist velocity = nav_2d_utils::twist2Dto3D(twist);
  return goal_checker_->isGoalReached(pose.pose, end_pose_, velocity);
}

bool ControllerServer::getRobotPose(geometry_msgs::msg::PoseStamped & pose)
{
  geometry_msgs::msg::PoseStamped current_pose;
  if (!costmap_ros_->getRobotPose(current_pose)) {
    return false;
  }
  pose = current_pose;
  return true;
}

}  // namespace nav2_controller
