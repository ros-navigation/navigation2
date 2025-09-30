// Copyright (c) 2024 Open Navigation LLC
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

#include "angles/angles.h"
#include "opennav_docking/docking_server.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace opennav_docking
{

DockingServer::DockingServer(const rclcpp::NodeOptions & options)
: nav2::LifecycleNode("docking_server", "", options)
{
  RCLCPP_INFO(get_logger(), "Creating %s", get_name());

  declare_parameter("controller_frequency", 50.0);
  declare_parameter("initial_perception_timeout", 5.0);
  declare_parameter("wait_charge_timeout", 5.0);
  declare_parameter("dock_approach_timeout", 30.0);
  declare_parameter("rotate_to_dock_timeout", 10.0);
  declare_parameter("undock_linear_tolerance", 0.05);
  declare_parameter("undock_angular_tolerance", 0.05);
  declare_parameter("max_retries", 3);
  declare_parameter("base_frame", "base_link");
  declare_parameter("fixed_frame", "odom");
  declare_parameter("dock_backwards", rclcpp::PARAMETER_BOOL);
  declare_parameter("dock_prestaging_tolerance", 0.5);
  declare_parameter("odom_topic", "odom");
  declare_parameter("odom_duration", 0.3);
  declare_parameter("rotation_angular_tolerance", 0.05);
}

nav2::CallbackReturn
DockingServer::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Configuring %s", get_name());
  auto node = shared_from_this();

  get_parameter("controller_frequency", controller_frequency_);
  get_parameter("initial_perception_timeout", initial_perception_timeout_);
  get_parameter("wait_charge_timeout", wait_charge_timeout_);
  get_parameter("dock_approach_timeout", dock_approach_timeout_);
  get_parameter("rotate_to_dock_timeout", rotate_to_dock_timeout_);
  get_parameter("undock_linear_tolerance", undock_linear_tolerance_);
  get_parameter("undock_angular_tolerance", undock_angular_tolerance_);
  get_parameter("max_retries", max_retries_);
  get_parameter("base_frame", base_frame_);
  get_parameter("fixed_frame", fixed_frame_);
  get_parameter("dock_prestaging_tolerance", dock_prestaging_tolerance_);
  get_parameter("rotation_angular_tolerance", rotation_angular_tolerance_);

  RCLCPP_INFO(get_logger(), "Controller frequency set to %.4fHz", controller_frequency_);

  // Check the dock_backwards deprecated parameter
  bool dock_backwards = false;
  try {
    if (get_parameter("dock_backwards", dock_backwards)) {
      dock_backwards_ = dock_backwards;
      RCLCPP_WARN(get_logger(), "Parameter dock_backwards is deprecated. "
      "Please use the dock_direction parameter in your dock plugin instead.");
    }
  } catch (rclcpp::exceptions::ParameterUninitializedException & ex) {
  }

  vel_publisher_ = std::make_unique<nav2_util::TwistPublisher>(node, "cmd_vel");
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  // Create odom subscriber for backward blind docking
  std::string odom_topic;
  get_parameter("odom_topic", odom_topic);
  double odom_duration;
  get_parameter("odom_duration", odom_duration);
  odom_sub_ = std::make_unique<nav2_util::OdomSmoother>(node, odom_duration, odom_topic);

  // Create the action servers for dock / undock
  docking_action_server_ = node->create_action_server<DockRobot>(
    "dock_robot",
    std::bind(&DockingServer::dockRobot, this),
    nullptr, std::chrono::milliseconds(500),
    true);

  undocking_action_server_ = node->create_action_server<UndockRobot>(
    "undock_robot",
    std::bind(&DockingServer::undockRobot, this),
    nullptr, std::chrono::milliseconds(500),
    true);

  // Create composed utilities
  mutex_ = std::make_shared<std::mutex>();
  controller_ = std::make_unique<Controller>(node, tf2_buffer_, fixed_frame_, base_frame_);
  navigator_ = std::make_unique<Navigator>(node);
  dock_db_ = std::make_unique<DockDatabase>(mutex_);
  if (!dock_db_->initialize(node, tf2_buffer_)) {
    on_cleanup(state);
    return nav2::CallbackReturn::FAILURE;
  }

  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn
DockingServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating %s", get_name());

  auto node = shared_from_this();

  tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_, this, true);
  dock_db_->activate();
  navigator_->activate();
  vel_publisher_->on_activate();
  docking_action_server_->activate();
  undocking_action_server_->activate();
  curr_dock_type_.clear();

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&DockingServer::dynamicParametersCallback, this, _1));

  // Create bond connection
  createBond();

  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn
DockingServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating %s", get_name());

  docking_action_server_->deactivate();
  undocking_action_server_->deactivate();
  dock_db_->deactivate();
  navigator_->deactivate();
  vel_publisher_->on_deactivate();

  remove_on_set_parameters_callback(dyn_params_handler_.get());
  dyn_params_handler_.reset();
  tf2_listener_.reset();

  // Destroy bond connection
  destroyBond();

  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn
DockingServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up %s", get_name());
  tf2_buffer_.reset();
  docking_action_server_.reset();
  undocking_action_server_.reset();
  dock_db_.reset();
  navigator_.reset();
  curr_dock_type_.clear();
  controller_.reset();
  vel_publisher_.reset();
  dock_backwards_.reset();
  odom_sub_.reset();
  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn
DockingServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down %s", get_name());
  return nav2::CallbackReturn::SUCCESS;
}

template<typename ActionT>
void DockingServer::getPreemptedGoalIfRequested(
  typename std::shared_ptr<const typename ActionT::Goal> goal,
  const typename nav2::SimpleActionServer<ActionT>::SharedPtr & action_server)
{
  if (action_server->is_preempt_requested()) {
    goal = action_server->accept_pending_goal();
  }
}

template<typename ActionT>
bool DockingServer::checkAndWarnIfCancelled(
  typename nav2::SimpleActionServer<ActionT>::SharedPtr & action_server,
  const std::string & name)
{
  if (action_server->is_cancel_requested()) {
    RCLCPP_WARN(get_logger(), "Goal was cancelled. Cancelling %s action", name.c_str());
    return true;
  }
  return false;
}

template<typename ActionT>
bool DockingServer::checkAndWarnIfPreempted(
  typename nav2::SimpleActionServer<ActionT>::SharedPtr & action_server,
  const std::string & name)
{
  if (action_server->is_preempt_requested()) {
    RCLCPP_WARN(get_logger(), "Goal was preempted. Cancelling %s action", name.c_str());
    return true;
  }
  return false;
}

void DockingServer::dockRobot()
{
  std::lock_guard<std::mutex> lock(*mutex_);
  action_start_time_ = this->now();
  rclcpp::Rate loop_rate(controller_frequency_);

  auto goal = docking_action_server_->get_current_goal();
  auto result = std::make_shared<DockRobot::Result>();
  result->success = false;

  if (!docking_action_server_ || !docking_action_server_->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
    return;
  }

  if (checkAndWarnIfCancelled<DockRobot>(docking_action_server_, "dock_robot")) {
    docking_action_server_->terminate_all();
    return;
  }

  getPreemptedGoalIfRequested<DockRobot>(goal, docking_action_server_);
  Dock * dock{nullptr};
  num_retries_ = 0;

  try {
    // Get dock (instance and plugin information) from request
    if (goal->use_dock_id) {
      RCLCPP_INFO(
        get_logger(),
        "Attempting to dock robot at %s.", goal->dock_id.c_str());
      dock = dock_db_->findDock(goal->dock_id);
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Attempting to dock robot at position (%0.2f, %0.2f).",
        goal->dock_pose.pose.position.x, goal->dock_pose.pose.position.y);
      dock = generateGoalDock(goal);
    }

    // Check if robot is docked or charging before proceeding, only applicable to charging docks
    if (dock->plugin->isCharger() && (dock->plugin->isDocked() || dock->plugin->isCharging())) {
      RCLCPP_INFO(
        get_logger(), "Robot is already docked and/or charging (if applicable), no need to dock");
      result->success = true;
      docking_action_server_->succeeded_current(result);
      return;
    }

    // Send robot to its staging pose
    publishDockingFeedback(DockRobot::Feedback::NAV_TO_STAGING_POSE);
    const auto initial_staging_pose = dock->getStagingPose();
    const auto robot_pose = getRobotPoseInFrame(initial_staging_pose.header.frame_id);
    if (!goal->navigate_to_staging_pose ||
      utils::l2Norm(robot_pose.pose, initial_staging_pose.pose) < dock_prestaging_tolerance_)
    {
      RCLCPP_INFO(get_logger(), "Robot already within pre-staging pose tolerance for dock");
    } else {
      std::function<bool()> isPreempted = [this]() {
          return checkAndWarnIfCancelled<DockRobot>(docking_action_server_, "dock_robot") ||
                 checkAndWarnIfPreempted<DockRobot>(docking_action_server_, "dock_robot");
        };

      navigator_->goToPose(
        initial_staging_pose,
        rclcpp::Duration::from_seconds(goal->max_staging_time),
        isPreempted);
      RCLCPP_INFO(get_logger(), "Successful navigation to staging pose");
    }

    // Construct initial estimate of where the dock is located in fixed_frame
    auto dock_pose = utils::getDockPoseStamped(dock, rclcpp::Time(0));
    tf2_buffer_->transform(dock_pose, dock_pose, fixed_frame_);

    // Get initial detection of dock before proceeding to move
    doInitialPerception(dock, dock_pose);
    RCLCPP_INFO(get_logger(), "Successful initial dock detection");

    // Get the direction of the movement
    bool dock_backward = dock_backwards_.has_value() ?
      dock_backwards_.value() :
      (dock->plugin->getDockDirection() == opennav_docking_core::DockDirection::BACKWARD);

    // If we performed a rotation before docking backward, we must rotate the staging pose
    // to match the robot orientation
    auto staging_pose = dock->getStagingPose();
    if (dock->plugin->shouldRotateToDock()) {
      staging_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
        tf2::getYaw(staging_pose.pose.orientation) + M_PI);
    }

    // Docking control loop: while not docked, run controller
    rclcpp::Time dock_contact_time;
    while (rclcpp::ok()) {
      try {
        // Perform a 180º to face away from the dock if needed
        if (dock->plugin->shouldRotateToDock()) {
          rotateToDock(dock_pose);
        }
        // Approach the dock using control law
        if (approachDock(dock, dock_pose, dock_backward)) {
          // We are docked, wait for charging to begin
          RCLCPP_INFO(
            get_logger(), "Made contact with dock, waiting for charge to start (if applicable).");
          publishZeroVelocity();
          if (waitForCharge(dock)) {
            if (dock->plugin->isCharger()) {
              RCLCPP_INFO(get_logger(), "Robot is charging!");
            } else {
              RCLCPP_INFO(get_logger(), "Docking was successful!");
            }
            result->success = true;
            result->num_retries = num_retries_;
            stashDockData(goal->use_dock_id, dock, true);
            publishZeroVelocity();
            dock->plugin->stopDetectionProcess();
            docking_action_server_->succeeded_current(result);
            return;
          }
        }

        // Cancelled, preempted, or shutting down (recoverable errors throw DockingException)
        stashDockData(goal->use_dock_id, dock, false);
        publishZeroVelocity();
        dock->plugin->stopDetectionProcess();
        docking_action_server_->terminate_all(result);
        return;
      } catch (opennav_docking_core::DockingException & e) {
        if (++num_retries_ > max_retries_) {
          RCLCPP_ERROR(get_logger(), "Failed to dock, all retries have been used");
          throw;
        }
        RCLCPP_WARN(get_logger(), "Docking failed, will retry: %s", e.what());
      }

      // Reset to staging pose to try again
      if (!resetApproach(staging_pose, dock_backward)) {
        // Cancelled, preempted, or shutting down
        stashDockData(goal->use_dock_id, dock, false);
        publishZeroVelocity();
        dock->plugin->stopDetectionProcess();
        docking_action_server_->terminate_all(result);
        return;
      }
      RCLCPP_INFO(get_logger(), "Returned to staging pose, attempting docking again");
    }
  } catch (const tf2::TransformException & e) {
    result->error_msg = std::string("Transform error: ") + e.what();
    RCLCPP_ERROR(get_logger(), result->error_msg.c_str());
    result->error_code = DockRobot::Result::UNKNOWN;
  } catch (opennav_docking_core::DockNotInDB & e) {
    result->error_msg = e.what();
    RCLCPP_ERROR(get_logger(), result->error_msg.c_str());
    result->error_code = DockRobot::Result::DOCK_NOT_IN_DB;
  } catch (opennav_docking_core::DockNotValid & e) {
    result->error_msg = e.what();
    RCLCPP_ERROR(get_logger(), result->error_msg.c_str());
    result->error_code = DockRobot::Result::DOCK_NOT_VALID;
  } catch (opennav_docking_core::FailedToStage & e) {
    result->error_msg = e.what();
    RCLCPP_ERROR(get_logger(), result->error_msg.c_str());
    result->error_code = DockRobot::Result::FAILED_TO_STAGE;
  } catch (opennav_docking_core::FailedToDetectDock & e) {
    result->error_msg = e.what();
    RCLCPP_ERROR(get_logger(), result->error_msg.c_str());
    result->error_code = DockRobot::Result::FAILED_TO_DETECT_DOCK;
  } catch (opennav_docking_core::FailedToControl & e) {
    result->error_msg = e.what();
    RCLCPP_ERROR(get_logger(), result->error_msg.c_str());
    result->error_code = DockRobot::Result::FAILED_TO_CONTROL;
  } catch (opennav_docking_core::FailedToCharge & e) {
    result->error_msg = e.what();
    RCLCPP_ERROR(get_logger(), result->error_msg.c_str());
    result->error_code = DockRobot::Result::FAILED_TO_CHARGE;
  } catch (opennav_docking_core::DockingException & e) {
    result->error_msg = e.what();
    RCLCPP_ERROR(get_logger(), result->error_msg.c_str());
    result->error_code = DockRobot::Result::UNKNOWN;
  } catch (std::exception & e) {
    result->error_code = DockRobot::Result::UNKNOWN;
    result->error_msg = e.what();
    RCLCPP_ERROR(get_logger(), result->error_msg.c_str());
  }

  // Store dock state for later undocking and delete temp dock, if applicable
  stashDockData(goal->use_dock_id, dock, false);
  result->num_retries = num_retries_;
  publishZeroVelocity();
  dock->plugin->stopDetectionProcess();
  docking_action_server_->terminate_current(result);
}

void DockingServer::stashDockData(bool use_dock_id, Dock * dock, bool successful)
{
  if (dock && successful) {
    curr_dock_type_ = dock->type;
  }

  if (!use_dock_id && dock) {
    delete dock;
    dock = nullptr;
  }
}

Dock * DockingServer::generateGoalDock(std::shared_ptr<const DockRobot::Goal> goal)
{
  auto dock = new Dock();
  dock->frame = goal->dock_pose.header.frame_id;
  dock->pose = goal->dock_pose.pose;
  dock->type = goal->dock_type;
  dock->plugin = dock_db_->findDockPlugin(dock->type);
  return dock;
}

void DockingServer::doInitialPerception(Dock * dock, geometry_msgs::msg::PoseStamped & dock_pose)
{
  publishDockingFeedback(DockRobot::Feedback::INITIAL_PERCEPTION);

  if (!dock->plugin->startDetectionProcess()) {
    throw opennav_docking_core::FailedToDetectDock("Failed to start the detection process.");
  }

  rclcpp::Rate loop_rate(controller_frequency_);
  auto start = this->now();
  auto timeout = rclcpp::Duration::from_seconds(initial_perception_timeout_);
  while (!dock->plugin->getRefinedPose(dock_pose, dock->id)) {
    if (this->now() - start > timeout) {
      throw opennav_docking_core::FailedToDetectDock(
        "Failed initial dock detection: Timeout exceeded");
    }

    if (checkAndWarnIfCancelled<DockRobot>(docking_action_server_, "dock_robot") ||
      checkAndWarnIfPreempted<DockRobot>(docking_action_server_, "dock_robot"))
    {
      return;
    }

    loop_rate.sleep();
  }
}

void DockingServer::rotateToDock(const geometry_msgs::msg::PoseStamped & dock_pose)
{
  const double dt = 1.0 / controller_frequency_;
  auto target_pose = dock_pose;
  target_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
    tf2::getYaw(target_pose.pose.orientation) + M_PI);

  rclcpp::Rate loop_rate(controller_frequency_);
  auto start = this->now();
  auto timeout = rclcpp::Duration::from_seconds(rotate_to_dock_timeout_);

  while (rclcpp::ok()) {
    auto robot_pose = getRobotPoseInFrame(dock_pose.header.frame_id);
    auto angular_distance_to_heading = angles::shortest_angular_distance(
      tf2::getYaw(robot_pose.pose.orientation), tf2::getYaw(target_pose.pose.orientation));
    if (fabs(angular_distance_to_heading) < rotation_angular_tolerance_) {
      break;
    }

    auto current_vel = std::make_unique<geometry_msgs::msg::TwistStamped>();
    current_vel->twist.angular.z = odom_sub_->getRawTwist().angular.z;

    auto command = std::make_unique<geometry_msgs::msg::TwistStamped>();
    command->header = robot_pose.header;
    command->twist = controller_->computeRotateToHeadingCommand(
      angular_distance_to_heading, current_vel->twist, dt);

    vel_publisher_->publish(std::move(command));

    if (this->now() - start > timeout) {
      throw opennav_docking_core::FailedToControl("Timed out rotating to dock");
    }

    loop_rate.sleep();
  }
}

bool DockingServer::approachDock(
  Dock * dock, geometry_msgs::msg::PoseStamped & dock_pose, bool backward)
{
  rclcpp::Rate loop_rate(controller_frequency_);
  auto start = this->now();
  auto timeout = rclcpp::Duration::from_seconds(dock_approach_timeout_);

  while (rclcpp::ok()) {
    publishDockingFeedback(DockRobot::Feedback::CONTROLLING);

    // Stop and report success if connected to dock
    if (dock->plugin->isDocked() || (dock->plugin->isCharger() && dock->plugin->isCharging())) {
      return true;
    }

    // Stop if cancelled/preempted
    if (checkAndWarnIfCancelled<DockRobot>(docking_action_server_, "dock_robot") ||
      checkAndWarnIfPreempted<DockRobot>(docking_action_server_, "dock_robot"))
    {
      return false;
    }

    // Update perception
    if (!dock->plugin->getRefinedPose(dock_pose, dock->id) && !dock->plugin->shouldRotateToDock()) {
      throw opennav_docking_core::FailedToDetectDock("Failed dock detection");
    }

    // Transform target_pose into base_link frame
    geometry_msgs::msg::PoseStamped target_pose = dock_pose;
    target_pose.header.stamp = rclcpp::Time(0);

    // The control law can get jittery when close to the end when atan2's can explode.
    // Thus, we backward project the controller's target pose a little bit after the
    // dock so that the robot never gets to the end of the spiral before its in contact
    // with the dock to stop the docking procedure.
    const double backward_projection = 0.25;
    const double yaw = tf2::getYaw(target_pose.pose.orientation);
    target_pose.pose.position.x += cos(yaw) * backward_projection;
    target_pose.pose.position.y += sin(yaw) * backward_projection;
    tf2_buffer_->transform(target_pose, target_pose, base_frame_);

    // Make sure that the target pose is pointing at the robot when moving backwards
    // This is to ensure that the robot doesn't try to dock from the wrong side
    if (backward) {
      target_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
        tf2::getYaw(target_pose.pose.orientation) + M_PI);
    }

    // Compute and publish controls
    auto command = std::make_unique<geometry_msgs::msg::TwistStamped>();
    command->header.stamp = now();
    if (!controller_->computeVelocityCommand(target_pose.pose, command->twist, true, backward)) {
      throw opennav_docking_core::FailedToControl("Failed to get control");
    }
    vel_publisher_->publish(std::move(command));

    if (this->now() - start > timeout) {
      throw opennav_docking_core::FailedToControl(
              "Timed out approaching dock; dock nor charging (if applicable) detected");
    }

    loop_rate.sleep();
  }
  return false;
}

bool DockingServer::waitForCharge(Dock * dock)
{
  // This is a non-charger docking request
  if (!dock->plugin->isCharger()) {
    return true;
  }

  rclcpp::Rate loop_rate(controller_frequency_);
  auto start = this->now();
  auto timeout = rclcpp::Duration::from_seconds(wait_charge_timeout_);
  while (rclcpp::ok()) {
    publishDockingFeedback(DockRobot::Feedback::WAIT_FOR_CHARGE);

    if (dock->plugin->isCharging()) {
      return true;
    }

    if (checkAndWarnIfCancelled<DockRobot>(docking_action_server_, "dock_robot") ||
      checkAndWarnIfPreempted<DockRobot>(docking_action_server_, "dock_robot"))
    {
      return false;
    }

    if (this->now() - start > timeout) {
      throw opennav_docking_core::FailedToCharge("Timed out waiting for charge to start");
    }

    loop_rate.sleep();
  }
  return false;
}

bool DockingServer::resetApproach(
  const geometry_msgs::msg::PoseStamped & staging_pose, bool backward)
{
  rclcpp::Rate loop_rate(controller_frequency_);
  auto start = this->now();
  auto timeout = rclcpp::Duration::from_seconds(dock_approach_timeout_);
  while (rclcpp::ok()) {
    publishDockingFeedback(DockRobot::Feedback::INITIAL_PERCEPTION);

    // Stop if cancelled/preempted
    if (checkAndWarnIfCancelled<DockRobot>(docking_action_server_, "dock_robot") ||
      checkAndWarnIfPreempted<DockRobot>(docking_action_server_, "dock_robot"))
    {
      return false;
    }

    // Compute and publish command
    auto command = std::make_unique<geometry_msgs::msg::TwistStamped>();
    command->header.stamp = now();
    if (getCommandToPose(
        command->twist, staging_pose, undock_linear_tolerance_, undock_angular_tolerance_, false,
        !backward))
    {
      return true;
    }
    vel_publisher_->publish(std::move(command));

    if (this->now() - start > timeout) {
      throw opennav_docking_core::FailedToControl("Timed out resetting dock approach");
    }

    loop_rate.sleep();
  }
  return false;
}

bool DockingServer::getCommandToPose(
  geometry_msgs::msg::Twist & cmd, const geometry_msgs::msg::PoseStamped & pose,
  double linear_tolerance, double angular_tolerance, bool is_docking, bool backward)
{
  // Reset command to zero velocity
  cmd.linear.x = 0;
  cmd.angular.z = 0;

  // Determine if we have reached pose yet & stop
  geometry_msgs::msg::PoseStamped robot_pose = getRobotPoseInFrame(pose.header.frame_id);
  const double dist = std::hypot(
    robot_pose.pose.position.x - pose.pose.position.x,
    robot_pose.pose.position.y - pose.pose.position.y);
  const double yaw = angles::shortest_angular_distance(
    tf2::getYaw(robot_pose.pose.orientation), tf2::getYaw(pose.pose.orientation));
  if (dist < linear_tolerance && abs(yaw) < angular_tolerance) {
    return true;
  }

  // Transform target_pose into base_link frame
  geometry_msgs::msg::PoseStamped target_pose = pose;
  target_pose.header.stamp = rclcpp::Time(0);
  tf2_buffer_->transform(target_pose, target_pose, base_frame_);

  // Compute velocity command
  if (!controller_->computeVelocityCommand(target_pose.pose, cmd, is_docking, backward)) {
    throw opennav_docking_core::FailedToControl("Failed to get control");
  }

  // Command is valid, but target is not reached
  return false;
}

void DockingServer::undockRobot()
{
  std::lock_guard<std::mutex> lock(*mutex_);
  action_start_time_ = this->now();
  rclcpp::Rate loop_rate(controller_frequency_);

  auto goal = undocking_action_server_->get_current_goal();
  auto result = std::make_shared<UndockRobot::Result>();
  result->success = false;

  if (!undocking_action_server_ || !undocking_action_server_->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
    return;
  }

  if (checkAndWarnIfCancelled<UndockRobot>(undocking_action_server_, "undock_robot")) {
    undocking_action_server_->terminate_all(result);
    return;
  }

  getPreemptedGoalIfRequested<UndockRobot>(goal, undocking_action_server_);
  auto max_duration = rclcpp::Duration::from_seconds(goal->max_undocking_time);

  try {
    // Get dock plugin information from request or docked state, reset state.
    std::string dock_type = curr_dock_type_;
    if (!goal->dock_type.empty()) {
      dock_type = goal->dock_type;
    }

    ChargingDock::Ptr dock = dock_db_->findDockPlugin(dock_type);
    if (!dock) {
      throw opennav_docking_core::DockNotValid("No dock information to undock from!");
    }
    RCLCPP_INFO(
      get_logger(),
      "Attempting to undock robot of dock type %s.", dock->getName().c_str());

    // Check if the robot is docked before proceeding
    if (dock->isCharger() && (!dock->isDocked() && !dock->isCharging())) {
      RCLCPP_INFO(get_logger(), "Robot is not in the dock, no need to undock");
      return;
    }

    bool dock_backward = dock_backwards_.has_value() ?
      dock_backwards_.value() :
      (dock->getDockDirection() == opennav_docking_core::DockDirection::BACKWARD);

    // Get "dock pose" by finding the robot pose
    geometry_msgs::msg::PoseStamped dock_pose = getRobotPoseInFrame(fixed_frame_);

    // Make sure that the staging pose is pointing in the same direction when moving backwards
    if (dock_backward) {
      dock_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
        tf2::getYaw(dock_pose.pose.orientation) + M_PI);
    }

    // Get staging pose (in fixed frame)
    geometry_msgs::msg::PoseStamped staging_pose =
      dock->getStagingPose(dock_pose.pose, dock_pose.header.frame_id);

    // If we performed a rotation before docking backward, we must rotate the staging pose
    // to match the robot orientation
    if (dock->shouldRotateToDock()) {
      staging_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
        tf2::getYaw(staging_pose.pose.orientation) + M_PI);
    }

    // Control robot to staging pose
    rclcpp::Time loop_start = this->now();
    while (rclcpp::ok()) {
      // Stop if we exceed max duration
      auto timeout = rclcpp::Duration::from_seconds(goal->max_undocking_time);
      if (this->now() - loop_start > timeout) {
        throw opennav_docking_core::FailedToControl("Undocking timed out");
      }

      // Stop if cancelled/preempted
      if (checkAndWarnIfCancelled<UndockRobot>(undocking_action_server_, "undock_robot") ||
        checkAndWarnIfPreempted<UndockRobot>(undocking_action_server_, "undock_robot"))
      {
        publishZeroVelocity();
        undocking_action_server_->terminate_all(result);
        return;
      }

      // Don't control the robot until charging is disabled
      if (dock->isCharger() && !dock->disableCharging()) {
        loop_rate.sleep();
        continue;
      }

      // Get command to approach staging pose
      auto command = std::make_unique<geometry_msgs::msg::TwistStamped>();
      command->header.stamp = now();

      if (getCommandToPose(
          command->twist, staging_pose, undock_linear_tolerance_, undock_angular_tolerance_, false,
          !dock_backward))
      {
        // Perform a 180º to the original staging pose
        if (dock->shouldRotateToDock()) {
          rotateToDock(staging_pose);
        }

        // Have reached staging_pose
        RCLCPP_INFO(get_logger(), "Robot has reached staging pose");
        vel_publisher_->publish(std::move(command));
        if (!dock->isCharger() || dock->hasStoppedCharging()) {
          RCLCPP_INFO(get_logger(), "Robot has undocked!");
          result->success = true;
          curr_dock_type_.clear();
          publishZeroVelocity();
          undocking_action_server_->succeeded_current(result);
          return;
        }
        // Haven't stopped charging?
        throw opennav_docking_core::FailedToControl("Failed to control off dock");
      }

      // Publish command and sleep
      vel_publisher_->publish(std::move(command));
      loop_rate.sleep();
    }
  } catch (const tf2::TransformException & e) {
    result->error_msg = std::string("Transform error: ") + e.what();
    RCLCPP_ERROR(get_logger(), result->error_msg.c_str());
    result->error_code = DockRobot::Result::UNKNOWN;
  } catch (opennav_docking_core::DockNotValid & e) {
    result->error_msg = e.what();
    RCLCPP_ERROR(get_logger(), result->error_msg.c_str());
    result->error_code = DockRobot::Result::DOCK_NOT_VALID;
  } catch (opennav_docking_core::FailedToControl & e) {
    result->error_msg = e.what();
    RCLCPP_ERROR(get_logger(), result->error_msg.c_str());
    result->error_code = DockRobot::Result::FAILED_TO_CONTROL;
  } catch (opennav_docking_core::DockingException & e) {
    result->error_msg = e.what();
    RCLCPP_ERROR(get_logger(), result->error_msg.c_str());
    result->error_code = DockRobot::Result::UNKNOWN;
  } catch (std::exception & e) {
    result->error_msg = std::string("Internal error: ") + e.what();
    RCLCPP_ERROR(get_logger(), result->error_msg.c_str());
    result->error_code = DockRobot::Result::UNKNOWN;
  }

  publishZeroVelocity();
  undocking_action_server_->terminate_current(result);
}

geometry_msgs::msg::PoseStamped DockingServer::getRobotPoseInFrame(const std::string & frame)
{
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = base_frame_;
  robot_pose.header.stamp = rclcpp::Time(0);
  tf2_buffer_->transform(robot_pose, robot_pose, frame);
  return robot_pose;
}

void DockingServer::publishZeroVelocity()
{
  auto cmd_vel = std::make_unique<geometry_msgs::msg::TwistStamped>();
  cmd_vel->header.stamp = now();
  vel_publisher_->publish(std::move(cmd_vel));
}

void DockingServer::publishDockingFeedback(uint16_t state)
{
  auto feedback = std::make_shared<DockRobot::Feedback>();
  feedback->state = state;
  feedback->docking_time = this->now() - action_start_time_;
  feedback->num_retries = num_retries_;
  docking_action_server_->publish_feedback(feedback);
}

rcl_interfaces::msg::SetParametersResult
DockingServer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::mutex> lock(*mutex_);

  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find('.') != std::string::npos) {
      continue;
    }

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == "controller_frequency") {
        controller_frequency_ = parameter.as_double();
      } else if (param_name == "initial_perception_timeout") {
        initial_perception_timeout_ = parameter.as_double();
      } else if (param_name == "wait_charge_timeout") {
        wait_charge_timeout_ = parameter.as_double();
      } else if (param_name == "undock_linear_tolerance") {
        undock_linear_tolerance_ = parameter.as_double();
      } else if (param_name == "undock_angular_tolerance") {
        undock_angular_tolerance_ = parameter.as_double();
      } else if (param_name == "rotation_angular_tolerance") {
        rotation_angular_tolerance_ = parameter.as_double();
      }
    } else if (param_type == ParameterType::PARAMETER_STRING) {
      if (param_name == "base_frame") {
        base_frame_ = parameter.as_string();
      } else if (param_name == "fixed_frame") {
        fixed_frame_ = parameter.as_string();
      }
    } else if (param_type == ParameterType::PARAMETER_INTEGER) {
      if (param_name == "max_retries") {
        max_retries_ = parameter.as_int();
      }
    }
  }

  result.successful = true;
  return result;
}

}  // namespace opennav_docking

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(opennav_docking::DockingServer)
