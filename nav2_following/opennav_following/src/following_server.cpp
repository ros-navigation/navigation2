// Copyright (c) 2024 Open Navigation LLC
// Copyright (c) 2024 Alberto J. Tudela Roldán
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
#include "opennav_docking_core/docking_exceptions.hpp"
#include "opennav_following/following_server.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace opennav_following
{

FollowingServer::FollowingServer(const rclcpp::NodeOptions & options)
: nav2::LifecycleNode("following_server", "", options)
{
  RCLCPP_INFO(get_logger(), "Creating %s", get_name());

  declare_parameter("controller_frequency", 50.0);
  declare_parameter("detection_timeout", 2.0);
  declare_parameter("rotate_to_object_timeout", 10.0);
  declare_parameter("static_object_timeout", -1.0);
  declare_parameter("linear_tolerance", 0.15);
  declare_parameter("angular_tolerance", 0.15);
  declare_parameter("max_retries", 3);
  declare_parameter("base_frame", "base_link");
  declare_parameter("fixed_frame", "odom");
  declare_parameter("filter_coef", 0.1);
  declare_parameter("desired_distance", 1.0);
  declare_parameter("skip_orientation", true);
  declare_parameter("search_by_rotating", false);
  declare_parameter("search_angle", M_PI_2);
  declare_parameter("odom_topic", "odom");
  declare_parameter("odom_duration", 0.3);
  declare_parameter("transform_tolerance", 0.1);
}

nav2::CallbackReturn
FollowingServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring %s", get_name());
  auto node = shared_from_this();

  get_parameter("controller_frequency", controller_frequency_);
  get_parameter("detection_timeout", detection_timeout_);
  get_parameter("rotate_to_object_timeout", rotate_to_object_timeout_);
  get_parameter("static_object_timeout", static_object_timeout_);
  get_parameter("linear_tolerance", linear_tolerance_);
  get_parameter("angular_tolerance", angular_tolerance_);
  get_parameter("max_retries", max_retries_);
  get_parameter("base_frame", base_frame_);
  get_parameter("fixed_frame", fixed_frame_);
  get_parameter("desired_distance", desired_distance_);
  get_parameter("skip_orientation", skip_orientation_);
  get_parameter("search_by_rotating", search_by_rotating_);
  get_parameter("search_angle", search_angle_);
  get_parameter("transform_tolerance", transform_tolerance_);
  RCLCPP_INFO(get_logger(), "Controller frequency set to %.4fHz", controller_frequency_);

  vel_publisher_ = std::make_unique<nav2_util::TwistPublisher>(node, "cmd_vel");
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  // Create odom subscriber for backward blind docking
  std::string odom_topic;
  get_parameter("odom_topic", odom_topic);
  double odom_duration;
  get_parameter("odom_duration", odom_duration);
  odom_sub_ = std::make_unique<nav2_util::OdomSmoother>(node, odom_duration, odom_topic);

  // Create the action server for dynamic following
  following_action_server_ = node->create_action_server<FollowObject>(
    "follow_object",
    std::bind(&FollowingServer::followObject, this),
    nullptr, std::chrono::milliseconds(500),
    true);

  // Create the controller
  // Note: Collision detection is not supported in following server so we force it off
  // and warn if the user has it enabled (from launch file or parameter file)
  declare_parameter("controller.use_collision_detection", false);
  controller_ =
    std::make_unique<opennav_docking::Controller>(node, tf2_buffer_, fixed_frame_, base_frame_);

  auto get_use_collision_detection = false;
  get_parameter("controller.use_collision_detection", get_use_collision_detection);
  if (get_use_collision_detection) {
    RCLCPP_ERROR(get_logger(),
      "Collision detection is not supported in the following server. Please disable "
      "the controller.use_collision_detection parameter.");
    return nav2::CallbackReturn::FAILURE;
  }

  // Setup filter
  double filter_coef;
  get_parameter("filter_coef", filter_coef);
  filter_ = std::make_unique<opennav_docking::PoseFilter>(filter_coef, detection_timeout_);

  // And publish the filtered pose
  filtered_dynamic_pose_pub_ =
    create_publisher<geometry_msgs::msg::PoseStamped>("filtered_dynamic_pose");

  // Initialize static object detection variables
  static_timer_initialized_ = false;
  static_object_start_time_ = rclcpp::Time(0);

  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn
FollowingServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating %s", get_name());

  auto node = shared_from_this();

  tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_, this, true);
  vel_publisher_->on_activate();
  filtered_dynamic_pose_pub_->on_activate();
  following_action_server_->activate();

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&FollowingServer::dynamicParametersCallback, this, _1));

  // Create bond connection
  createBond();

  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn
FollowingServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating %s", get_name());

  following_action_server_->deactivate();
  vel_publisher_->on_deactivate();
  filtered_dynamic_pose_pub_->on_deactivate();

  remove_on_set_parameters_callback(dyn_params_handler_.get());
  dyn_params_handler_.reset();
  tf2_listener_.reset();

  // Destroy bond connection
  destroyBond();

  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn
FollowingServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up %s", get_name());
  tf2_buffer_.reset();
  following_action_server_.reset();
  controller_.reset();
  vel_publisher_.reset();
  filtered_dynamic_pose_pub_.reset();
  odom_sub_.reset();
  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn
FollowingServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down %s", get_name());
  return nav2::CallbackReturn::SUCCESS;
}

template<typename ActionT>
void FollowingServer::getPreemptedGoalIfRequested(
  typename std::shared_ptr<const typename ActionT::Goal> goal,
  const typename nav2::SimpleActionServer<ActionT>::SharedPtr & action_server)
{
  if (action_server->is_preempt_requested()) {
    goal = action_server->accept_pending_goal();
  }
}

template<typename ActionT>
bool FollowingServer::checkAndWarnIfCancelled(
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
bool FollowingServer::checkAndWarnIfPreempted(
  typename nav2::SimpleActionServer<ActionT>::SharedPtr & action_server,
  const std::string & name)
{
  if (action_server->is_preempt_requested()) {
    RCLCPP_WARN(get_logger(), "Goal was preempted. Cancelling %s action", name.c_str());
    return true;
  }
  return false;
}

void FollowingServer::followObject()
{
  std::unique_lock<std::mutex> lock(dynamic_params_lock_);
  action_start_time_ = this->now();
  rclcpp::Rate loop_rate(controller_frequency_);

  auto goal = following_action_server_->get_current_goal();
  auto result = std::make_shared<FollowObject::Result>();

  if (!following_action_server_ || !following_action_server_->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
    return;
  }

  if (checkAndWarnIfCancelled<FollowObject>(following_action_server_, "follow_object")) {
    following_action_server_->terminate_all();
    return;
  }

  getPreemptedGoalIfRequested<FollowObject>(goal, following_action_server_);
  num_retries_ = 0;
  static_timer_initialized_ = false;

  // Reset the last detected dynamic pose timestamp so we start fresh for this action
  detected_dynamic_pose_.header.stamp = rclcpp::Time(0);

  try {
    auto pose_topic = goal->pose_topic;
    auto target_frame = goal->tracked_frame;
    if (target_frame.empty()) {
      if (pose_topic.empty()) {
        RCLCPP_ERROR(get_logger(),
          "Both pose topic and target frame are empty. Cannot follow object.");
        result->error_code = FollowObject::Result::FAILED_TO_DETECT_OBJECT;
        result->error_msg = "No pose topic or target frame provided.";
        following_action_server_->terminate_all(result);
        return;
      } else {
        lock.unlock();
        RCLCPP_INFO(get_logger(), "Subscribing to pose topic: %s", pose_topic.c_str());
        dynamic_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
          pose_topic,
          [this](const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
            detected_dynamic_pose_ = *pose;
          },
          nav2::qos::StandardTopicQoS(1));  // Only want the most recent pose
        lock.lock();
      }
    } else {
      RCLCPP_INFO(get_logger(), "Following frame: %s instead of pose", target_frame.c_str());
    }

    // Following control loop: while not timeout, run controller
    geometry_msgs::msg::PoseStamped object_pose;
    rclcpp::Duration max_duration = goal->max_duration;
    while (rclcpp::ok()) {
      try {
        // Check if we have run out of time
        if (this->now() - action_start_time_ > max_duration && max_duration.seconds() > 0.0) {
          RCLCPP_INFO(get_logger(), "Exceeded max duration. Stopping.");
          result->total_elapsed_time = this->now() - action_start_time_;
          result->num_retries = num_retries_;
          publishZeroVelocity();
          following_action_server_->succeeded_current(result);
          dynamic_pose_sub_.reset();
          return;
        }

        // Approach the object using control law
        if (approachObject(object_pose, target_frame)) {
          // Initialize static timer on first entry
          if (!static_timer_initialized_) {
            static_object_start_time_ = this->now();
            static_timer_initialized_ = true;
          }

          // We have reached the object, maintain position
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Reached object. Stopping until goal is moved again.");
          publishFollowingFeedback(FollowObject::Feedback::STOPPING);
          publishZeroVelocity();

          // Stop if the object has been static for some time
          if (static_object_timeout_ > 0.0) {
            auto static_duration = this->now() - static_object_start_time_;
            if (static_duration.seconds() > static_object_timeout_) {
              RCLCPP_INFO(get_logger(),
                "Object has been static for %.2f seconds (timeout: %.2f), stopping.",
                static_duration.seconds(), static_object_timeout_);
              result->total_elapsed_time = this->now() - action_start_time_;
              result->num_retries = num_retries_;
              publishZeroVelocity();
              following_action_server_->succeeded_current(result);
              return;
            }
          }
        } else {
          // Cancelled, preempted, or shutting down (recoverable errors throw DockingException)
          static_timer_initialized_ = false;
          result->total_elapsed_time = this->now() - action_start_time_;
          publishZeroVelocity();
          following_action_server_->terminate_all(result);
          dynamic_pose_sub_.reset();
          return;
        }
      } catch (opennav_docking_core::DockingException & e) {
        if (++num_retries_ > max_retries_) {
          RCLCPP_ERROR(get_logger(), "Failed to follow, all retries have been used");
          throw;
        }
        RCLCPP_WARN(get_logger(), "Following failed, will retry: %s", e.what());

        // Perform an in-place rotation to find the object again
        if (search_by_rotating_) {
          RCLCPP_INFO(get_logger(), "Rotating to find object again");
          if (!rotateToObject(object_pose, target_frame)) {
            // Cancelled, preempted, or shutting down
            publishZeroVelocity();
            following_action_server_->terminate_all(result);
            return;
          }
        } else {
          RCLCPP_INFO(get_logger(), "Using last known heading to find object again");
        }
      }
      loop_rate.sleep();
    }
  } catch (const tf2::TransformException & e) {
    result->error_msg = std::string("Transform error: ") + e.what();
    RCLCPP_ERROR(get_logger(), result->error_msg.c_str());
    result->error_code = FollowObject::Result::TF_ERROR;
  } catch (opennav_docking_core::FailedToDetectDock & e) {
    result->error_msg = e.what();
    RCLCPP_ERROR(get_logger(), result->error_msg.c_str());
    result->error_code = FollowObject::Result::FAILED_TO_DETECT_OBJECT;
  } catch (opennav_docking_core::FailedToControl & e) {
    result->error_msg = e.what();
    RCLCPP_ERROR(get_logger(), result->error_msg.c_str());
    result->error_code = FollowObject::Result::FAILED_TO_CONTROL;
  } catch (opennav_docking_core::DockingException & e) {
    result->error_msg = e.what();
    RCLCPP_ERROR(get_logger(), result->error_msg.c_str());
    result->error_code = FollowObject::Result::UNKNOWN;
  } catch (std::exception & e) {
    result->error_msg = e.what();
    RCLCPP_ERROR(get_logger(), result->error_msg.c_str());
    result->error_code = FollowObject::Result::UNKNOWN;
  }

  // Stop the robot and report
  result->total_elapsed_time = this->now() - action_start_time_;
  result->num_retries = num_retries_;
  publishZeroVelocity();
  following_action_server_->terminate_current(result);
  dynamic_pose_sub_.reset();
}

bool FollowingServer::approachObject(
  geometry_msgs::msg::PoseStamped & object_pose, const std::string & target_frame)
{
  rclcpp::Rate loop_rate(controller_frequency_);
  while (rclcpp::ok()) {
    // Update the iteration start time, used for get robot position, transformation and control
    iteration_start_time_ = this->now();

    publishFollowingFeedback(FollowObject::Feedback::CONTROLLING);

    // Stop if cancelled/preempted
    if (checkAndWarnIfCancelled<FollowObject>(following_action_server_, "follow_object") ||
      checkAndWarnIfPreempted<FollowObject>(following_action_server_, "follow_object"))
    {
      return false;
    }

    // Get the tracking pose from topic or frame
    getTrackingPose(object_pose, target_frame);

    // Get the pose at the distance we want to maintain from the object
    // Stop and report success if goal is reached
    auto target_pose = getPoseAtDistance(object_pose, desired_distance_);
    if (isGoalReached(target_pose)) {
      return true;
    }

    // The control law can get jittery when close to the end when atan2's can explode.
    // Thus, we reduce the desired distance by a small amount so that the robot never
    // gets to the end of the spiral before its at the desired distance to stop the
    // following procedure.
    const double backward_projection = 0.25;
    const double effective_distance = desired_distance_ - backward_projection;
    target_pose = getPoseAtDistance(object_pose, effective_distance);

    // ... and transform the target_pose into base_frame
    try {
      tf2_buffer_->transform(
        target_pose, target_pose, base_frame_, tf2::durationFromSec(transform_tolerance_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "Failed to transform target pose: %s", ex.what());
      return false;
    }

    // If the object is behind the robot, we reverse the control
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!nav2_util::getCurrentPose(
        robot_pose, *tf2_buffer_, target_pose.header.frame_id, base_frame_, transform_tolerance_,
        iteration_start_time_))
    {
      RCLCPP_WARN(get_logger(), "Failed to get current robot pose");
      return false;
    }

    // Compute and publish controls
    auto command = std::make_unique<geometry_msgs::msg::TwistStamped>();
    command->header.stamp = now();
    if (!controller_->computeVelocityCommand(target_pose.pose, command->twist, true, false)) {
      throw opennav_docking_core::FailedToControl("Failed to get control");
    }
    vel_publisher_->publish(std::move(command));

    loop_rate.sleep();
  }
  return false;
}

bool FollowingServer::rotateToObject(
  geometry_msgs::msg::PoseStamped & object_pose, const std::string & target_frame)
{
  const double dt = 1.0 / controller_frequency_;

  // Compute initial robot heading
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!nav2_util::getCurrentPose(
      robot_pose, *tf2_buffer_, object_pose.header.frame_id, base_frame_, transform_tolerance_,
      iteration_start_time_))
  {
    RCLCPP_WARN(get_logger(), "Failed to get current robot pose");
    return false;
  }
  double initial_yaw = tf2::getYaw(robot_pose.pose.orientation);

  // Search angles: left offset, then right offset from initial heading
  std::vector<double> angles = {initial_yaw + search_angle_, initial_yaw - search_angle_};

  rclcpp::Rate loop_rate(controller_frequency_);
  auto start = this->now();
  auto timeout = rclcpp::Duration::from_seconds(rotate_to_object_timeout_);

  // Iterate over target angles
  for (const double & target_angle : angles) {
    // Create a target pose oriented at target_angle
    auto target_pose = object_pose;
    target_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(target_angle);

    // Rotate towards target_angle while checking for detection
    while (rclcpp::ok()) {
      // Update the iteration start time, used for get robot position, transformation and control
      iteration_start_time_ = this->now();

      publishFollowingFeedback(FollowObject::Feedback::RETRY);

      // Stop if cancelled/preempted
      if (checkAndWarnIfCancelled<FollowObject>(following_action_server_, "follow_object") ||
        checkAndWarnIfPreempted<FollowObject>(following_action_server_, "follow_object"))
      {
        return false;
      }

      // Get current robot pose
      if (!nav2_util::getCurrentPose(
          robot_pose, *tf2_buffer_, object_pose.header.frame_id, base_frame_, transform_tolerance_,
          iteration_start_time_))
      {
        RCLCPP_WARN(get_logger(), "Failed to get current robot pose");
        return false;
      }

      double angular_distance_to_heading = angles::shortest_angular_distance(
        tf2::getYaw(robot_pose.pose.orientation), target_angle);

      // If we are close enough to the target orientation, break and try next angle
      if (fabs(angular_distance_to_heading) < angular_tolerance_) {
        break;
      }

      // While rotating, check if we can get the tracking pose (object detected)
      try {
        if (getTrackingPose(object_pose, target_frame)) {
          return true;
        }
      } catch (opennav_docking_core::FailedToDetectDock & e) {
        // No detection yet, continue rotating
      }

      geometry_msgs::msg::Twist current_vel;
      current_vel.angular.z = odom_sub_->getRawTwist().angular.z;

      auto command = std::make_unique<geometry_msgs::msg::TwistStamped>();
      command->header = robot_pose.header;
      command->twist = controller_->computeRotateToHeadingCommand(
        angular_distance_to_heading, current_vel, dt);

      vel_publisher_->publish(std::move(command));

      if (this->now() - start > timeout) {
        throw opennav_docking_core::FailedToControl("Timed out rotating to object");
      }

      loop_rate.sleep();
    }
  }

  // If we exhausted all search angles and did not detect the object, fail
  throw opennav_docking_core::FailedToControl("Failed to rotate to object");
}

void FollowingServer::publishZeroVelocity()
{
  auto cmd_vel = std::make_unique<geometry_msgs::msg::TwistStamped>();
  cmd_vel->header.frame_id = base_frame_;
  cmd_vel->header.stamp = now();
  vel_publisher_->publish(std::move(cmd_vel));
}

void FollowingServer::publishFollowingFeedback(uint16_t state)
{
  auto feedback = std::make_shared<FollowObject::Feedback>();
  feedback->state = state;
  feedback->following_time = iteration_start_time_ - action_start_time_;
  feedback->num_retries = num_retries_;
  following_action_server_->publish_feedback(feedback);
}

bool FollowingServer::getRefinedPose(geometry_msgs::msg::PoseStamped & pose)
{
  // Get current detections and transform to frame
  geometry_msgs::msg::PoseStamped detected = detected_dynamic_pose_;

  // If we haven't received any detection yet, wait up to detection_timeout_ for one to arrive.
  if (detected.header.stamp == rclcpp::Time(0)) {
    auto start = this->now();
    auto timeout = rclcpp::Duration::from_seconds(detection_timeout_);
    rclcpp::Rate wait_rate(controller_frequency_);
    while (this->now() - start < timeout) {
      // Check if a new detection arrived
      if (detected_dynamic_pose_.header.stamp != rclcpp::Time(0)) {
        detected = detected_dynamic_pose_;
        break;
      }
      wait_rate.sleep();
    }
    if (detected.header.stamp == rclcpp::Time(0)) {
      RCLCPP_WARN(this->get_logger(), "No detection received within timeout period");
      return false;
    }
  }

  // Validate that external pose is new enough
  auto timeout = rclcpp::Duration::from_seconds(detection_timeout_);
  if (this->now() - detected.header.stamp > timeout) {
    RCLCPP_WARN(this->get_logger(), "Lost detection or did not detect: timeout exceeded");
    return false;
  }

  // Transform detected pose into fixed frame
  if (detected.header.frame_id != fixed_frame_) {
    try {
      tf2_buffer_->transform(
        detected, detected, fixed_frame_, tf2::durationFromSec(transform_tolerance_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Failed to transform detected object pose");
      return false;
    }
  }

  // The control law can oscillate if the orientation in the perception
  // is not set correctly or has a lot of noise.
  // Then, we skip the target orientation by pointing it
  // in the same orientation than the vector from the robot to the object.
  if (skip_orientation_) {
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!nav2_util::getCurrentPose(
        robot_pose, *tf2_buffer_, detected.header.frame_id, base_frame_, transform_tolerance_,
        iteration_start_time_))
    {
      RCLCPP_WARN(get_logger(), "Failed to get current robot pose");
      return false;
    }
    double dx = detected.pose.position.x - robot_pose.pose.position.x;
    double dy = detected.pose.position.y - robot_pose.pose.position.y;
    double angle_to_target = std::atan2(dy, dx);
    detected.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(angle_to_target);
  }

  // Filter the detected pose
  auto pose_filtered = filter_->update(detected);
  filtered_dynamic_pose_pub_->publish(pose_filtered);

  pose = pose_filtered;
  return true;
}

bool FollowingServer::getFramePose(
  geometry_msgs::msg::PoseStamped & pose, const std::string & frame_id)
{
  try {
    // Get the transform from the target frame to the fixed frame
    auto transform = tf2_buffer_->lookupTransform(
      fixed_frame_, frame_id, iteration_start_time_, tf2::durationFromSec(transform_tolerance_));

    // Convert transform to pose
    pose.header.frame_id = fixed_frame_;
    pose.header.stamp = transform.header.stamp;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(),
      "Failed to get transform for frame %s: %s", frame_id.c_str(), ex.what());
    return false;
  }

  // Filter the detected pose
  auto filtered_pose = filter_->update(pose);
  filtered_dynamic_pose_pub_->publish(filtered_pose);

  pose = filtered_pose;
  return true;
}

bool FollowingServer::getTrackingPose(
  geometry_msgs::msg::PoseStamped & pose, const std::string & frame_id)
{
  // Use frame tracking if we have a target frame, otherwise use topic tracking
  if (!frame_id.empty()) {
    if (!getFramePose(pose, frame_id)) {
      throw opennav_docking_core::FailedToDetectDock(
        "Failed to get pose in target frame: " + frame_id);
    }
  } else {
    // Use the traditional pose detection from topic
    if (!getRefinedPose(pose)) {
      throw opennav_docking_core::FailedToDetectDock("Failed object detection");
    }
  }
  return true;
}

geometry_msgs::msg::PoseStamped FollowingServer::getPoseAtDistance(
  const geometry_msgs::msg::PoseStamped & pose, double distance)
{
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!nav2_util::getCurrentPose(
      robot_pose, *tf2_buffer_, pose.header.frame_id, base_frame_, transform_tolerance_,
      iteration_start_time_))
  {
    RCLCPP_WARN(get_logger(), "Failed to get current robot pose");
    // Return original pose as fallback
    return pose;
  }
  double dx = pose.pose.position.x - robot_pose.pose.position.x;
  double dy = pose.pose.position.y - robot_pose.pose.position.y;
  const double dist = std::hypot(dx, dy);
  geometry_msgs::msg::PoseStamped forward_pose = pose;
  forward_pose.pose.position.x -= distance * (dx / dist);
  forward_pose.pose.position.y -= distance * (dy / dist);
  return forward_pose;
}

bool FollowingServer::isGoalReached(const geometry_msgs::msg::PoseStamped & goal_pose)
{
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!nav2_util::getCurrentPose(
      robot_pose, *tf2_buffer_, goal_pose.header.frame_id, base_frame_, transform_tolerance_,
      iteration_start_time_))
  {
    RCLCPP_WARN(get_logger(), "Failed to get current robot pose");
    return false;
  }
  const double dist = std::hypot(
    robot_pose.pose.position.x - goal_pose.pose.position.x,
    robot_pose.pose.position.y - goal_pose.pose.position.y);
  const double yaw = angles::shortest_angular_distance(
    tf2::getYaw(robot_pose.pose.orientation), tf2::getYaw(goal_pose.pose.orientation));
  return dist < linear_tolerance_ && abs(yaw) < angular_tolerance_;
}

rcl_interfaces::msg::SetParametersResult
FollowingServer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);

  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == "controller_frequency") {
        controller_frequency_ = parameter.as_double();
      } else if (name == "detection_timeout") {
        detection_timeout_ = parameter.as_double();
      } else if (name == "rotate_to_object_timeout") {
        rotate_to_object_timeout_ = parameter.as_double();
      } else if (name == "static_object_timeout") {
        static_object_timeout_ = parameter.as_double();
      } else if (name == "linear_tolerance") {
        linear_tolerance_ = parameter.as_double();
      } else if (name == "angular_tolerance") {
        angular_tolerance_ = parameter.as_double();
      } else if (name == "desired_distance") {
        desired_distance_ = parameter.as_double();
      } else if (name == "transform_tolerance") {
        transform_tolerance_ = parameter.as_double();
      } else if (name == "search_angle") {
        search_angle_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_STRING) {
      if (name == "base_frame") {
        base_frame_ = parameter.as_string();
      } else if (name == "fixed_frame") {
        fixed_frame_ = parameter.as_string();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == "skip_orientation") {
        skip_orientation_ = parameter.as_bool();
      } else if (name == "search_by_rotating") {
        search_by_rotating_ = parameter.as_bool();
      }
    }
  }

  result.successful = true;
  return result;
}

}  // namespace opennav_following

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(opennav_following::FollowingServer)
