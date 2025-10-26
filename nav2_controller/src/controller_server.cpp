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
#include <limits>

#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_ros_common/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/path_utils.hpp"
#include "nav2_controller/controller_server.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;
using nav2_util::geometry_utils::euclidean_distance;

namespace nav2_controller
{

ControllerServer::ControllerServer(const rclcpp::NodeOptions & options)
: nav2::LifecycleNode("controller_server", "", options),
  progress_checker_loader_("nav2_core", "nav2_core::ProgressChecker"),
  goal_checker_loader_("nav2_core", "nav2_core::GoalChecker"),
  lp_loader_("nav2_core", "nav2_core::Controller"),
  path_handler_loader_("nav2_core", "nav2_core::PathHandler"),
  start_index_(0),
  costmap_update_timeout_(300ms)
{
  RCLCPP_INFO(get_logger(), "Creating controller server");

  // The costmap node is used in the implementation of the controller
  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "local_costmap", std::string{get_namespace()},
    get_parameter("use_sim_time").as_bool(), options);
}

ControllerServer::~ControllerServer()
{
  progress_checkers_.clear();
  goal_checkers_.clear();
  controllers_.clear();
  path_handlers_.clear();
  costmap_thread_.reset();
}

nav2::CallbackReturn
ControllerServer::on_configure(const rclcpp_lifecycle::State & state)
{
  auto node = shared_from_this();

  RCLCPP_INFO(get_logger(), "Configuring controller interface");

  costmap_ros_->configure();
  // Launch a thread to run the costmap node
  costmap_thread_ = std::make_unique<nav2::NodeThread>(costmap_ros_);
  transform_tolerance_ = costmap_ros_->getTransformTolerance();
  try {
    param_handler_ = std::make_unique<ParameterHandler>(
      node, get_logger());
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(get_logger(), "%s", ex.what());
    on_cleanup(state);
    return nav2::CallbackReturn::FAILURE;
  }
  params_ = param_handler_->getParams();

  for (size_t i = 0; i != params_->progress_checker_ids.size(); i++) {
    try {
      nav2_core::ProgressChecker::Ptr progress_checker =
        progress_checker_loader_.createUniqueInstance(params_->progress_checker_types[i]);
      RCLCPP_INFO(
        get_logger(), "Created progress_checker : %s of type %s",
        params_->progress_checker_ids[i].c_str(), params_->progress_checker_types[i].c_str());
      progress_checker->initialize(node, params_->progress_checker_ids[i]);
      progress_checkers_.insert({params_->progress_checker_ids[i], progress_checker});
    } catch (const std::exception & ex) {
      RCLCPP_FATAL(
        get_logger(),
        "Failed to create progress_checker. Exception: %s", ex.what());
      on_cleanup(state);
      return nav2::CallbackReturn::FAILURE;
    }
  }

  for (size_t i = 0; i != params_->progress_checker_ids.size(); i++) {
    progress_checker_ids_concat_ += params_->progress_checker_ids[i] + std::string(" ");
  }

  RCLCPP_INFO(
    get_logger(),
    "Controller Server has %s progress checkers available.", progress_checker_ids_concat_.c_str());

  for (size_t i = 0; i != params_->goal_checker_ids.size(); i++) {
    try {
      nav2_core::GoalChecker::Ptr goal_checker =
        goal_checker_loader_.createUniqueInstance(params_->goal_checker_types[i]);
      RCLCPP_INFO(
        get_logger(), "Created goal checker : %s of type %s",
        params_->goal_checker_ids[i].c_str(), params_->goal_checker_types[i].c_str());
      goal_checker->initialize(node, params_->goal_checker_ids[i], costmap_ros_);
      goal_checkers_.insert({params_->goal_checker_ids[i], goal_checker});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(),
        "Failed to create goal checker. Exception: %s", ex.what());
      on_cleanup(state);
      return nav2::CallbackReturn::FAILURE;
    }
  }

  for (size_t i = 0; i != params_->goal_checker_ids.size(); i++) {
    goal_checker_ids_concat_ += params_->goal_checker_ids[i] + std::string(" ");
  }

  RCLCPP_INFO(
    get_logger(),
    "Controller Server has %s goal checkers available.", goal_checker_ids_concat_.c_str());

  for (size_t i = 0; i != params_->path_handler_ids.size(); i++) {
    try {
      nav2_core::PathHandler::Ptr path_handler =
        path_handler_loader_.createUniqueInstance(params_->path_handler_types[i]);
      RCLCPP_INFO(
        get_logger(), "Created path handler : %s of type %s",
        params_->path_handler_ids[i].c_str(), params_->path_handler_types[i].c_str());
      path_handler->initialize(node, get_logger(), params_->path_handler_ids[i], costmap_ros_,
          costmap_ros_->getTfBuffer());
      path_handlers_.insert({params_->path_handler_ids[i], path_handler});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(),
        "Failed to create path handler Exception: %s", ex.what());
      on_cleanup(state);
      return nav2::CallbackReturn::FAILURE;
    }
  }

  for (size_t i = 0; i != params_->path_handler_ids.size(); i++) {
    path_handler_ids_concat_ += params_->path_handler_ids[i] + std::string(" ");
  }

  RCLCPP_INFO(
    get_logger(),
    "Controller Server has %s path handlers available.", path_handler_ids_concat_.c_str());

  for (size_t i = 0; i != params_->controller_ids.size(); i++) {
    try {
      nav2_core::Controller::Ptr controller =
        lp_loader_.createUniqueInstance(params_->controller_types[i]);
      RCLCPP_INFO(
        get_logger(), "Created controller : %s of type %s",
        params_->controller_ids[i].c_str(), params_->controller_types[i].c_str());
      controller->configure(
        node, params_->controller_ids[i],
        costmap_ros_->getTfBuffer(), costmap_ros_);
      controllers_.insert({params_->controller_ids[i], controller});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(),
        "Failed to create controller. Exception: %s", ex.what());
      on_cleanup(state);
      return nav2::CallbackReturn::FAILURE;
    }
  }

  for (size_t i = 0; i != params_->controller_ids.size(); i++) {
    controller_ids_concat_ += params_->controller_ids[i] + std::string(" ");
  }

  RCLCPP_INFO(
    get_logger(),
    "Controller Server has %s controllers available.", controller_ids_concat_.c_str());

  odom_sub_ = std::make_unique<nav2_util::OdomSmoother>(node, params_->odom_duration,
      params_->odom_topic);
  vel_publisher_ = std::make_unique<nav2_util::TwistPublisher>(node, "cmd_vel");
  transformed_plan_pub_ = create_publisher<nav_msgs::msg::Path>("transformed_global_plan");
  tracking_feedback_pub_ = create_publisher<nav2_msgs::msg::TrackingFeedback>("tracking_feedback");

  costmap_update_timeout_ = rclcpp::Duration::from_seconds(params_->costmap_update_timeout);

  // Create the action server that we implement with our followPath method
  // This may throw due to real-time prioritization if user doesn't have real-time permissions
  try {
    action_server_ = create_action_server<Action>(
      "follow_path",
      std::bind(&ControllerServer::computeControl, this),
      nullptr,
      std::chrono::milliseconds(500),
      true /*spin thread*/, params_->use_realtime_priority /*soft realtime*/);
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR(get_logger(), "Error creating action server! %s", e.what());
    on_cleanup(state);
    return nav2::CallbackReturn::FAILURE;
  }

  // Set subscription to the speed limiting topic
  speed_limit_sub_ = create_subscription<nav2_msgs::msg::SpeedLimit>(
    params_->speed_limit_topic,
    std::bind(&ControllerServer::speedLimitCallback, this, std::placeholders::_1));

  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn
ControllerServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  const auto costmap_ros_state = costmap_ros_->activate();
  if (costmap_ros_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return nav2::CallbackReturn::FAILURE;
  }
  ControllerMap::iterator it;
  for (it = controllers_.begin(); it != controllers_.end(); ++it) {
    it->second->activate();
  }
  vel_publisher_->on_activate();
  transformed_plan_pub_->on_activate();
  tracking_feedback_pub_->on_activate();
  action_server_->activate();
  param_handler_->activate();
  auto node = shared_from_this();

  // create bond connection
  createBond();

  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn
ControllerServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_->deactivate();
  ControllerMap::iterator it;
  for (it = controllers_.begin(); it != controllers_.end(); ++it) {
    it->second->deactivate();
  }

  /*
   * The costmap is also a lifecycle node, so it may have already fired on_deactivate
   * via rcl preshutdown cb. Despite the rclcpp docs saying on_shutdown callbacks fire
   * in the order added, the preshutdown callbacks clearly don't per se, due to using an
   * unordered_set iteration. Once this issue is resolved, we can maybe make a stronger
   * ordering assumption: https://github.com/ros2/rclcpp/issues/2096
   */
  costmap_ros_->deactivate();

  publishZeroVelocity();
  vel_publisher_->on_deactivate();
  transformed_plan_pub_->on_deactivate();
  tracking_feedback_pub_->on_deactivate();
  param_handler_->deactivate();

  // destroy bond connection
  destroyBond();

  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn
ControllerServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  // Cleanup the helper classes
  ControllerMap::iterator it;
  for (it = controllers_.begin(); it != controllers_.end(); ++it) {
    it->second->cleanup();
  }
  controllers_.clear();

  goal_checkers_.clear();
  progress_checkers_.clear();
  path_handlers_.clear();

  costmap_ros_->cleanup();


  // Release any allocated resources
  action_server_.reset();
  odom_sub_.reset();
  costmap_thread_.reset();
  vel_publisher_.reset();
  transformed_plan_pub_.reset();
  tracking_feedback_pub_.reset();
  speed_limit_sub_.reset();

  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn
ControllerServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2::CallbackReturn::SUCCESS;
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
        "which does not exist. Available controllers are: %s.",
        c_name.c_str(), controller_ids_concat_.c_str());
      return false;
    }
  } else {
    RCLCPP_DEBUG(get_logger(), "Selected controller: %s.", c_name.c_str());
    current_controller = c_name;
  }

  return true;
}

bool ControllerServer::findGoalCheckerId(
  const std::string & c_name,
  std::string & current_goal_checker)
{
  if (goal_checkers_.find(c_name) == goal_checkers_.end()) {
    if (goal_checkers_.size() == 1 && c_name.empty()) {
      RCLCPP_WARN_ONCE(
        get_logger(), "No goal checker was specified in parameter 'current_goal_checker'."
        " Server will use only plugin loaded %s. "
        "This warning will appear once.", goal_checker_ids_concat_.c_str());
      current_goal_checker = goal_checkers_.begin()->first;
    } else {
      RCLCPP_ERROR(
        get_logger(), "FollowPath called with goal_checker name %s in parameter"
        " 'current_goal_checker', which does not exist. Available goal checkers are: %s.",
        c_name.c_str(), goal_checker_ids_concat_.c_str());
      return false;
    }
  } else {
    RCLCPP_DEBUG(get_logger(), "Selected goal checker: %s.", c_name.c_str());
    current_goal_checker = c_name;
  }

  return true;
}

bool ControllerServer::findProgressCheckerId(
  const std::string & c_name,
  std::string & current_progress_checker)
{
  if (progress_checkers_.find(c_name) == progress_checkers_.end()) {
    if (progress_checkers_.size() == 1 && c_name.empty()) {
      RCLCPP_WARN_ONCE(
        get_logger(), "No progress checker was specified in parameter 'current_progress_checker'."
        " Server will use only plugin loaded %s. "
        "This warning will appear once.", progress_checker_ids_concat_.c_str());
      current_progress_checker = progress_checkers_.begin()->first;
    } else {
      RCLCPP_ERROR(
        get_logger(), "FollowPath called with progress_checker name %s in parameter"
        " 'current_progress_checker', which does not exist. Available progress checkers are: %s.",
        c_name.c_str(), progress_checker_ids_concat_.c_str());
      return false;
    }
  } else {
    RCLCPP_DEBUG(get_logger(), "Selected progress checker: %s.", c_name.c_str());
    current_progress_checker = c_name;
  }

  return true;
}

bool ControllerServer::findPathHandlerId(
  const std::string & c_name,
  std::string & current_path_handler)
{
  if (path_handlers_.find(c_name) == path_handlers_.end()) {
    if (path_handlers_.size() == 1 && c_name.empty()) {
      RCLCPP_WARN_ONCE(
        get_logger(), "No path handler was specified in parameter 'current_path_handler'."
        " Server will use only plugin loaded %s. "
        "This warning will appear once.", path_handler_ids_concat_.c_str());
      current_path_handler = path_handlers_.begin()->first;
    } else {
      RCLCPP_ERROR(
        get_logger(), "FollowPath called with path_handler name %s in parameter"
        " 'current_path_handler', which does not exist. Available path handlers are: %s.",
        c_name.c_str(), path_handler_ids_concat_.c_str());
      return false;
    }
  } else {
    RCLCPP_DEBUG(get_logger(), "Selected path handler: %s.", c_name.c_str());
    current_path_handler = c_name;
  }

  return true;
}

void ControllerServer::computeControl()
{
  std::lock_guard<std::mutex> lock_reinit(param_handler_->getMutex());

  RCLCPP_INFO(get_logger(), "Received a goal, begin computing control effort.");

  try {
    auto goal = action_server_->get_current_goal();
    if (!goal) {
      return;  //  goal would be nullptr if action_server_ is deactivate.
    }

    std::string c_name = goal->controller_id;
    std::string current_controller;
    if (findControllerId(c_name, current_controller)) {
      current_controller_ = current_controller;
    } else {
      throw nav2_core::InvalidController("Failed to find controller name: " + c_name);
    }

    std::string gc_name = goal->goal_checker_id;
    std::string current_goal_checker;
    if (findGoalCheckerId(gc_name, current_goal_checker)) {
      current_goal_checker_ = current_goal_checker;
    } else {
      throw nav2_core::ControllerException("Failed to find goal checker name: " + gc_name);
    }

    std::string pc_name = goal->progress_checker_id;
    std::string current_progress_checker;
    if (findProgressCheckerId(pc_name, current_progress_checker)) {
      current_progress_checker_ = current_progress_checker;
    } else {
      throw nav2_core::ControllerException("Failed to find progress checker name: " + pc_name);
    }

    std::string ph_name = goal->path_handler_id;
    std::string current_path_handler;
    if(findPathHandlerId(ph_name, current_path_handler)) {
      current_path_handler_ = current_path_handler;
    } else {
      throw nav2_core::ControllerException("Failed to find path handler name: " + ph_name);
    }

    setPlannerPath(goal->path);
    progress_checkers_[current_progress_checker_]->reset();

    last_valid_cmd_time_ = now();
    rclcpp::WallRate loop_rate(params_->controller_frequency);
    while (rclcpp::ok()) {
      auto start_time = this->now();

      if (action_server_ == nullptr || !action_server_->is_server_active()) {
        RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
        return;
      }

      if (action_server_->is_cancel_requested()) {
        if (controllers_[current_controller_]->cancel()) {
          RCLCPP_INFO(get_logger(), "Cancellation was successful. Stopping the robot.");
          action_server_->terminate_all();
          onGoalExit(true);
          return;
        } else {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000, "Waiting for the controller to finish cancellation");
        }
      }

      // Don't compute a trajectory until costmap is valid (after clear costmap)
      rclcpp::Rate r(100);
      auto waiting_start = now();
      while (!costmap_ros_->isCurrent()) {
        if (now() - waiting_start > costmap_update_timeout_) {
          throw nav2_core::ControllerTimedOut("Costmap timed out waiting for update");
        }
        r.sleep();
      }

      updateGlobalPath();

      computeAndPublishVelocity();

      if (isGoalReached()) {
        RCLCPP_INFO(get_logger(), "Reached the goal!");
        break;
      }

      auto cycle_duration = this->now() - start_time;
      if (!loop_rate.sleep()) {
        RCLCPP_WARN(
          get_logger(),
          "Control loop missed its desired rate of %.4f Hz. Current loop rate is %.4f Hz.",
          params_->controller_frequency, 1 / cycle_duration.seconds());
      }
    }
  } catch (nav2_core::InvalidController & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    onGoalExit(true);
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    result->error_code = Action::Result::INVALID_CONTROLLER;
    result->error_msg = e.what();
    action_server_->terminate_current(result);
    return;
  } catch (nav2_core::ControllerTFError & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    onGoalExit(true);
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    result->error_code = Action::Result::TF_ERROR;
    result->error_msg = e.what();
    action_server_->terminate_current(result);
    return;
  } catch (nav2_core::NoValidControl & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    onGoalExit(true);
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    result->error_code = Action::Result::NO_VALID_CONTROL;
    result->error_msg = e.what();
    action_server_->terminate_current(result);
    return;
  } catch (nav2_core::FailedToMakeProgress & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    onGoalExit(true);
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    result->error_code = Action::Result::FAILED_TO_MAKE_PROGRESS;
    result->error_msg = e.what();
    action_server_->terminate_current(result);
    return;
  } catch (nav2_core::PatienceExceeded & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    onGoalExit(true);
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    result->error_code = Action::Result::PATIENCE_EXCEEDED;
    result->error_msg = e.what();
    action_server_->terminate_current(result);
    return;
  } catch (nav2_core::InvalidPath & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    onGoalExit(true);
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    result->error_code = Action::Result::INVALID_PATH;
    result->error_msg = e.what();
    action_server_->terminate_current(result);
    return;
  } catch (nav2_core::ControllerTimedOut & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    onGoalExit(true);
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    result->error_code = Action::Result::CONTROLLER_TIMED_OUT;
    result->error_msg = e.what();
    action_server_->terminate_current(result);
    return;
  } catch (nav2_core::ControllerException & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    onGoalExit(true);
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    result->error_code = Action::Result::UNKNOWN;
    result->error_msg = e.what();
    action_server_->terminate_current(result);
    return;
  } catch (std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    onGoalExit(true);
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    result->error_code = Action::Result::UNKNOWN;
    result->error_msg = e.what();
    action_server_->terminate_current(result);
    return;
  }

  RCLCPP_DEBUG(get_logger(), "Controller succeeded, setting result");

  onGoalExit(false);

  // TODO(orduno) #861 Handle a pending preemption and set controller name
  action_server_->succeeded_current();
}

void ControllerServer::setPlannerPath(const nav_msgs::msg::Path & path)
{
  RCLCPP_DEBUG(
    get_logger(),
    "Providing path to the controller %s", current_controller_.c_str());
  if (path.poses.empty()) {
    throw nav2_core::InvalidPath("Path is empty.");
  }
  controllers_[current_controller_]->newPathReceived(path);
  path_handlers_[current_path_handler_]->setPlan(path);

  end_pose_ = path.poses.back();
  end_pose_.header.frame_id = path.header.frame_id;
  goal_checkers_[current_goal_checker_]->reset();

  RCLCPP_DEBUG(
    get_logger(), "Path end point is (%.2f, %.2f)",
    end_pose_.pose.position.x, end_pose_.pose.position.y);

  start_index_ = 0;
  current_path_ = path;
}

void ControllerServer::computeAndPublishVelocity()
{
  geometry_msgs::msg::PoseStamped pose;

  if (!getRobotPose(pose)) {
    throw nav2_core::ControllerTFError("Failed to obtain robot pose");
  }

  if (!progress_checkers_[current_progress_checker_]->check(pose)) {
    throw nav2_core::FailedToMakeProgress("Failed to make progress");
  }

  geometry_msgs::msg::Twist twist = getThresholdedTwist(odom_sub_->getRawTwist());

  geometry_msgs::msg::PoseStamped goal =
    path_handlers_[current_path_handler_]->getTransformedGoal(pose.header.stamp);
  transformed_global_plan_ = path_handlers_[current_path_handler_]->transformGlobalPlan(pose);
  auto path = std::make_unique<nav_msgs::msg::Path>(transformed_global_plan_);
  transformed_plan_pub_->publish(std::move(path));

  geometry_msgs::msg::TwistStamped cmd_vel_2d;

  try {
    cmd_vel_2d =
      controllers_[current_controller_]->computeVelocityCommands(
      pose,
      twist,
      goal_checkers_[current_goal_checker_].get(),
      transformed_global_plan_,
      goal
      );
    last_valid_cmd_time_ = now();
    cmd_vel_2d.header.frame_id = costmap_ros_->getBaseFrameID();
    cmd_vel_2d.header.stamp = last_valid_cmd_time_;
    // Only no valid control exception types are valid to attempt to have control patience, as
    // other types will not be resolved with more attempts
  } catch (nav2_core::NoValidControl & e) {
    if (params_->failure_tolerance > 0 || params_->failure_tolerance == -1.0) {
      RCLCPP_WARN(this->get_logger(), "%s", e.what());
      cmd_vel_2d.twist.angular.x = 0;
      cmd_vel_2d.twist.angular.y = 0;
      cmd_vel_2d.twist.angular.z = 0;
      cmd_vel_2d.twist.linear.x = 0;
      cmd_vel_2d.twist.linear.y = 0;
      cmd_vel_2d.twist.linear.z = 0;
      cmd_vel_2d.header.frame_id = costmap_ros_->getBaseFrameID();
      cmd_vel_2d.header.stamp = now();
      if ((now() - last_valid_cmd_time_).seconds() > params_->failure_tolerance &&
        params_->failure_tolerance != -1.0)
      {
        throw nav2_core::PatienceExceeded("Controller patience exceeded");
      }
    } else {
      throw nav2_core::NoValidControl(e.what());
    }
  }

  RCLCPP_DEBUG(get_logger(), "Publishing velocity at time %.2f", now().seconds());
  publishVelocity(cmd_vel_2d);

  nav2_msgs::msg::TrackingFeedback current_tracking_feedback;

  // Use the current robot pose's timestamp for the transformation
  end_pose_.header.stamp = pose.header.stamp;

  if (!nav2_util::transformPoseInTargetFrame(
        end_pose_, transformed_end_pose_, *costmap_ros_->getTfBuffer(),
        costmap_ros_->getGlobalFrameID(), transform_tolerance_))
  {
    throw nav2_core::ControllerTFError("Failed to transform end pose to global frame");
  }

  if (current_path_.poses.size() >= 2) {
    double current_distance_to_goal = nav2_util::geometry_utils::euclidean_distance(
      pose, transformed_end_pose_);

    // Transform robot pose to path frame for path tracking calculations
    geometry_msgs::msg::PoseStamped robot_pose_in_path_frame;
    if (!nav2_util::transformPoseInTargetFrame(
      pose, robot_pose_in_path_frame, *costmap_ros_->getTfBuffer(),
            current_path_.header.frame_id, transform_tolerance_))
    {
      throw nav2_core::ControllerTFError("Failed to transform robot pose to path frame");
    }

    const auto path_search_result = nav2_util::distance_from_path(
      current_path_, robot_pose_in_path_frame.pose, start_index_, params_->search_window);

    // Create tracking error message
    auto tracking_feedback_msg = std::make_unique<nav2_msgs::msg::TrackingFeedback>();
    tracking_feedback_msg->header = pose.header;
    tracking_feedback_msg->tracking_error = path_search_result.distance;
    tracking_feedback_msg->current_path_index = path_search_result.closest_segment_index;
    tracking_feedback_msg->robot_pose = pose;
    tracking_feedback_msg->distance_to_goal = current_distance_to_goal;
    tracking_feedback_msg->speed = std::hypot(twist.linear.x, twist.linear.y);
    tracking_feedback_msg->remaining_path_length =
      nav2_util::geometry_utils::calculate_path_length(current_path_, start_index_);
    start_index_ = path_search_result.closest_segment_index;

    // Update current tracking error and publish
    current_tracking_feedback = *tracking_feedback_msg;
    if (tracking_feedback_pub_->get_subscription_count() > 0) {
      tracking_feedback_pub_->publish(std::move(tracking_feedback_msg));
    }
  }

  // Publish action feedback
  std::shared_ptr<Action::Feedback> feedback = std::make_shared<Action::Feedback>();
  feedback->tracking_feedback = current_tracking_feedback;
  action_server_->publish_feedback(feedback);
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
      std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
      result->error_code = Action::Result::INVALID_CONTROLLER;
      result->error_msg = "Terminating action, invalid controller " +
        goal->controller_id + " requested.";
      action_server_->terminate_current(result);
      return;
    }
    std::string current_goal_checker;
    if (findGoalCheckerId(goal->goal_checker_id, current_goal_checker)) {
      current_goal_checker_ = current_goal_checker;
    } else {
      std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
      result->error_code = Action::Result::INVALID_CONTROLLER;
      result->error_msg = "Terminating action, invalid goal checker " +
        goal->goal_checker_id + " requested.";
      action_server_->terminate_current(result);
      return;
    }
    std::string current_progress_checker;
    if (findProgressCheckerId(goal->progress_checker_id, current_progress_checker)) {
      if (current_progress_checker_ != current_progress_checker) {
        RCLCPP_INFO(
          get_logger(), "Change of progress checker %s requested, resetting it",
          goal->progress_checker_id.c_str());
        current_progress_checker_ = current_progress_checker;
        progress_checkers_[current_progress_checker_]->reset();
      }
    } else {
      std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
      result->error_code = Action::Result::INVALID_CONTROLLER;
      result->error_msg = "Terminating action, invalid progress checker " +
        goal->progress_checker_id + " requested.";
      action_server_->terminate_current(result);
      return;
    }
    std::string current_path_handler;
    if (findPathHandlerId(goal->path_handler_id, current_path_handler)) {
      if(current_path_handler_ != current_path_handler) {
        RCLCPP_INFO(
          get_logger(), "Change of path handler %s requested, resetting it",
          goal->path_handler_id.c_str());
        current_path_handler_ = current_path_handler;
      }
    } else {
      std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
      result->error_code = Action::Result::INVALID_CONTROLLER;
      result->error_msg = "Terminating action, invalid path handler" +
        goal->path_handler_id + " requested.";
      action_server_->terminate_current(result);
      return;
    }
    setPlannerPath(goal->path);
  }
}

void ControllerServer::publishVelocity(const geometry_msgs::msg::TwistStamped & velocity)
{
  auto cmd_vel = std::make_unique<geometry_msgs::msg::TwistStamped>(velocity);
  if (!nav2_util::validateTwist(cmd_vel->twist)) {
    RCLCPP_ERROR(get_logger(), "Velocity message contains NaNs or Infs! Ignoring as invalid!");
    return;
  }
  if (vel_publisher_->is_activated() && vel_publisher_->get_subscription_count() > 0) {
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

void ControllerServer::onGoalExit(bool force_stop)
{
  if (params_->publish_zero_velocity || force_stop) {
    publishZeroVelocity();
  }

  // Reset controller state
  for (auto & controller : controllers_) {
    controller.second->reset();
  }
}

bool ControllerServer::isGoalReached()
{
  geometry_msgs::msg::PoseStamped pose;

  if (!getRobotPose(pose)) {
    return false;
  }

  geometry_msgs::msg::Twist velocity = getThresholdedTwist(odom_sub_->getRawTwist());

  return goal_checkers_[current_goal_checker_]->isGoalReached(
    pose.pose, transformed_end_pose_.pose,
    velocity, transformed_global_plan_);
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

void ControllerServer::speedLimitCallback(const nav2_msgs::msg::SpeedLimit::SharedPtr msg)
{
  ControllerMap::iterator it;
  for (it = controllers_.begin(); it != controllers_.end(); ++it) {
    it->second->setSpeedLimit(msg->speed_limit, msg->percentage);
  }
}

}  // namespace nav2_controller

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_controller::ControllerServer)
