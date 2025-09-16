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

#include <cmath>
#include <chrono>

#include "nav2_ros_common/node_utils.hpp"
#include "opennav_docking/simple_charging_dock.hpp"
#include "opennav_docking/utils.hpp"

using namespace std::chrono_literals;

namespace opennav_docking
{

void SimpleChargingDock::configure(
  const nav2::LifecycleNode::WeakPtr & parent,
  const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf)
{
  name_ = name;
  tf2_buffer_ = tf;
  is_charging_ = false;
  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Optionally use battery info to check when charging, else say charging if docked
  nav2::declare_parameter_if_not_declared(
    node_, name + ".use_battery_status", rclcpp::ParameterValue(true));

  // Parameters for optional detector control
  nav2::declare_parameter_if_not_declared(
    node_, name + ".detector_service_name", rclcpp::ParameterValue(""));
  nav2::declare_parameter_if_not_declared(
    node_, name + ".detector_service_timeout", rclcpp::ParameterValue(5.0));
  nav2::declare_parameter_if_not_declared(
    node_, name + ".subscribe_toggle", rclcpp::ParameterValue(false));

  // Parameters for optional external detection of dock pose
  nav2::declare_parameter_if_not_declared(
    node_, name + ".use_external_detection_pose", rclcpp::ParameterValue(false));
  nav2::declare_parameter_if_not_declared(
    node_, name + ".external_detection_timeout", rclcpp::ParameterValue(1.0));
  nav2::declare_parameter_if_not_declared(
    node_, name + ".external_detection_translation_x", rclcpp::ParameterValue(-0.20));
  nav2::declare_parameter_if_not_declared(
    node_, name + ".external_detection_translation_y", rclcpp::ParameterValue(0.0));
  nav2::declare_parameter_if_not_declared(
    node_, name + ".external_detection_rotation_yaw", rclcpp::ParameterValue(0.0));
  nav2::declare_parameter_if_not_declared(
    node_, name + ".external_detection_rotation_pitch", rclcpp::ParameterValue(1.57));
  nav2::declare_parameter_if_not_declared(
    node_, name + ".external_detection_rotation_roll", rclcpp::ParameterValue(-1.57));
  nav2::declare_parameter_if_not_declared(
    node_, name + ".filter_coef", rclcpp::ParameterValue(0.1));

  // Charging threshold from BatteryState message
  nav2::declare_parameter_if_not_declared(
    node_, name + ".charging_threshold", rclcpp::ParameterValue(0.5));

  // Optionally determine if docked via stall detection using joint_states
  nav2::declare_parameter_if_not_declared(
    node_, name + ".use_stall_detection", rclcpp::ParameterValue(false));
  nav2::declare_parameter_if_not_declared(
    node_, name + ".stall_joint_names", rclcpp::PARAMETER_STRING_ARRAY);
  nav2::declare_parameter_if_not_declared(
    node_, name + ".stall_velocity_threshold", rclcpp::ParameterValue(1.0));
  nav2::declare_parameter_if_not_declared(
    node_, name + ".stall_effort_threshold", rclcpp::ParameterValue(1.0));

  // If not using stall detection, this is how close robot should get to pose
  nav2::declare_parameter_if_not_declared(
    node_, name + ".docking_threshold", rclcpp::ParameterValue(0.05));

  // Staging pose configuration
  nav2::declare_parameter_if_not_declared(
    node_, name + ".staging_x_offset", rclcpp::ParameterValue(-0.7));
  nav2::declare_parameter_if_not_declared(
    node_, name + ".staging_yaw_offset", rclcpp::ParameterValue(0.0));

  // Direction of docking and if we should rotate to dock
  nav2::declare_parameter_if_not_declared(
    node_, name + ".dock_direction", rclcpp::ParameterValue(std::string("forward")));
  nav2::declare_parameter_if_not_declared(
    node_, name + ".rotate_to_dock", rclcpp::ParameterValue(false));

  node_->get_parameter(name + ".use_battery_status", use_battery_status_);
  node_->get_parameter(name + ".use_external_detection_pose", use_external_detection_pose_);
  node_->get_parameter(name + ".external_detection_timeout", external_detection_timeout_);
  node_->get_parameter(
    name + ".external_detection_translation_x", external_detection_translation_x_);
  node_->get_parameter(
    name + ".external_detection_translation_y", external_detection_translation_y_);
  double yaw, pitch, roll;
  node_->get_parameter(name + ".external_detection_rotation_yaw", yaw);
  node_->get_parameter(name + ".external_detection_rotation_pitch", pitch);
  node_->get_parameter(name + ".external_detection_rotation_roll", roll);
  external_detection_rotation_.setEuler(pitch, roll, yaw);
  node_->get_parameter(name + ".charging_threshold", charging_threshold_);
  node_->get_parameter(name + ".stall_velocity_threshold", stall_velocity_threshold_);
  node_->get_parameter(name + ".stall_effort_threshold", stall_effort_threshold_);
  node_->get_parameter(name + ".docking_threshold", docking_threshold_);
  node_->get_parameter("base_frame", base_frame_id_);  // Get server base frame ID
  node_->get_parameter(name + ".staging_x_offset", staging_x_offset_);
  node_->get_parameter(name + ".staging_yaw_offset", staging_yaw_offset_);

  node_->get_parameter(name + ".detector_service_name", detector_service_name_);
  node_->get_parameter(name + ".detector_service_timeout", detector_service_timeout_);
  node_->get_parameter(name + ".subscribe_toggle", subscribe_toggle_);

  // Initialize detection state
  detection_started_ = false;
  initial_pose_received_ = false;

  // Create persistent subscription if toggling is disabled.
  if (use_external_detection_pose_ && !subscribe_toggle_) {
    dock_pose_.header.stamp = rclcpp::Time(0);
    dock_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "detected_dock_pose",
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
        detected_dock_pose_ = *pose;
        initial_pose_received_ = true;
      },
      nav2::qos::StandardTopicQoS());
  }

  std::string dock_direction;
  node_->get_parameter(name + ".dock_direction", dock_direction);
  dock_direction_ = utils::getDockDirectionFromString(dock_direction);
  if (dock_direction_ == opennav_docking_core::DockDirection::UNKNOWN) {
    throw std::runtime_error{"Dock direction is not valid. Valid options are: forward or backward"};
  }

  node_->get_parameter(name + ".rotate_to_dock", rotate_to_dock_);
  if (rotate_to_dock_ && dock_direction_ != opennav_docking_core::DockDirection::BACKWARD) {
    throw std::runtime_error{"Parameter rotate_to_dock is enabled but dock direction is not "
            "backward. Please set dock direction to backward."};
  }

  // Setup filter
  double filter_coef;
  node_->get_parameter(name + ".filter_coef", filter_coef);
  filter_ = std::make_unique<PoseFilter>(filter_coef, external_detection_timeout_);

  if (!detector_service_name_.empty()) {
    detector_client_ = node_->create_client<std_srvs::srv::Trigger>(
      detector_service_name_, false);
  }

  if (use_battery_status_) {
    battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
      "battery_state",
      [this](const sensor_msgs::msg::BatteryState::SharedPtr state) {
        is_charging_ = state->current > charging_threshold_;
      });
  }

  bool use_stall_detection;
  node_->get_parameter(name + ".use_stall_detection", use_stall_detection);
  if (use_stall_detection) {
    is_stalled_ = false;
    node_->get_parameter(name + ".stall_joint_names", stall_joint_names_);
    if (stall_joint_names_.size() < 1) {
      RCLCPP_ERROR(node_->get_logger(), "stall_joint_names cannot be empty!");
    }
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states",
      std::bind(&SimpleChargingDock::jointStateCallback, this, std::placeholders::_1),
      nav2::qos::StandardTopicQoS());
  }

  dock_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("dock_pose");
  filtered_dock_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    "filtered_dock_pose");
  staging_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("staging_pose");
}

geometry_msgs::msg::PoseStamped SimpleChargingDock::getStagingPose(
  const geometry_msgs::msg::Pose & pose, const std::string & frame)
{
  // If not using detection, set the dock pose as the given dock pose estimate
  if (!use_external_detection_pose_) {
    // This gets called at the start of docking
    // Reset our internally tracked dock pose
    dock_pose_.header.frame_id = frame;
    dock_pose_.pose = pose;
  }

  // Compute the staging pose with given offsets
  const double yaw = tf2::getYaw(pose.orientation);
  geometry_msgs::msg::PoseStamped staging_pose;
  staging_pose.header.frame_id = frame;
  staging_pose.header.stamp = node_->now();
  staging_pose.pose = pose;
  staging_pose.pose.position.x += cos(yaw) * staging_x_offset_;
  staging_pose.pose.position.y += sin(yaw) * staging_x_offset_;
  tf2::Quaternion orientation;
  orientation.setEuler(0.0, 0.0, yaw + staging_yaw_offset_);
  staging_pose.pose.orientation = tf2::toMsg(orientation);

  // Publish staging pose for debugging purposes
  staging_pose_pub_->publish(staging_pose);
  return staging_pose;
}

bool SimpleChargingDock::getRefinedPose(geometry_msgs::msg::PoseStamped & pose, std::string /*id*/)
{
  // If using not detection, set the dock pose to the static fixed-frame version
  if (!use_external_detection_pose_) {
    dock_pose_pub_->publish(pose);
    dock_pose_ = pose;
    return true;
  }

  // Guard against using pose data before the first detection has arrived.
  if (!initial_pose_received_) {
    RCLCPP_WARN(node_->get_logger(), "Waiting for first detected_dock_pose; none received yet");
    return false;
  }

  // If using detections, get current detections, transform to frame, and apply offsets
  geometry_msgs::msg::PoseStamped detected = detected_dock_pose_;

  // Validate that external pose is new enough
  auto timeout = rclcpp::Duration::from_seconds(external_detection_timeout_);
  if (node_->now() - detected.header.stamp > timeout) {
    RCLCPP_WARN(node_->get_logger(), "Lost detection or did not detect: timeout exceeded");
    return false;
  }

  // Transform detected pose into fixed frame. Note that the argument pose
  // is the output of detection, but also acts as the initial estimate
  // and contains the frame_id of docking
  if (detected.header.frame_id != pose.header.frame_id) {
    try {
      if (!tf2_buffer_->canTransform(
          pose.header.frame_id, detected.header.frame_id,
          detected.header.stamp, rclcpp::Duration::from_seconds(0.2)))
      {
        RCLCPP_WARN(node_->get_logger(), "Failed to transform detected dock pose");
        return false;
      }
      tf2_buffer_->transform(detected, detected, pose.header.frame_id);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(node_->get_logger(), "Failed to transform detected dock pose");
      return false;
    }
  }

  // Filter the detected pose
  detected = filter_->update(detected);
  filtered_dock_pose_pub_->publish(detected);

  // Rotate the just the orientation, then remove roll/pitch
  geometry_msgs::msg::PoseStamped just_orientation;
  just_orientation.pose.orientation = tf2::toMsg(external_detection_rotation_);
  geometry_msgs::msg::TransformStamped transform;
  transform.transform.rotation = detected.pose.orientation;
  tf2::doTransform(just_orientation, just_orientation, transform);

  tf2::Quaternion orientation;
  orientation.setEuler(0.0, 0.0, tf2::getYaw(just_orientation.pose.orientation));
  dock_pose_.pose.orientation = tf2::toMsg(orientation);

  // Construct dock_pose_ by applying translation/rotation
  dock_pose_.header = detected.header;
  dock_pose_.pose.position = detected.pose.position;
  const double yaw = tf2::getYaw(dock_pose_.pose.orientation);
  dock_pose_.pose.position.x += cos(yaw) * external_detection_translation_x_ -
    sin(yaw) * external_detection_translation_y_;
  dock_pose_.pose.position.y += sin(yaw) * external_detection_translation_x_ +
    cos(yaw) * external_detection_translation_y_;
  dock_pose_.pose.position.z = 0.0;

  // Publish & return dock pose for debugging purposes
  dock_pose_pub_->publish(dock_pose_);
  pose = dock_pose_;
  return true;
}

bool SimpleChargingDock::isDocked()
{
  if (joint_state_sub_) {
    // Using stall detection
    return is_stalled_;
  }

  if (dock_pose_.header.frame_id.empty()) {
    // Dock pose is not yet valid
    return false;
  }

  // Find base pose in target frame
  geometry_msgs::msg::PoseStamped base_pose;
  base_pose.header.stamp = rclcpp::Time(0);
  base_pose.header.frame_id = base_frame_id_;
  base_pose.pose.orientation.w = 1.0;
  try {
    tf2_buffer_->transform(base_pose, base_pose, dock_pose_.header.frame_id);
  } catch (const tf2::TransformException & ex) {
    return false;
  }

  // If we are close enough, pretend we are charging
  double d = std::hypot(
    base_pose.pose.position.x - dock_pose_.pose.position.x,
    base_pose.pose.position.y - dock_pose_.pose.position.y);
  return d < docking_threshold_;
}

bool SimpleChargingDock::isCharging()
{
  return use_battery_status_ ? is_charging_ : isDocked();
}

bool SimpleChargingDock::disableCharging()
{
  return true;
}

bool SimpleChargingDock::hasStoppedCharging()
{
  return !isCharging();
}

void SimpleChargingDock::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr state)
{
  double velocity = 0.0;
  double effort = 0.0;
  for (size_t i = 0; i < state->name.size(); ++i) {
    for (auto & name : stall_joint_names_) {
      if (state->name[i] == name) {
        // Tracking this joint
        velocity += abs(state->velocity[i]);
        effort += abs(state->effort[i]);
      }
    }
  }

  // Take average
  effort /= stall_joint_names_.size();
  velocity /= stall_joint_names_.size();

  is_stalled_ = (velocity < stall_velocity_threshold_) && (effort > stall_effort_threshold_);
}

bool SimpleChargingDock::startDetectionProcess()
{
  // Skip if already starting or ON
  if (detection_started_) {
    return true;
  }

  // 1. Service START request
  if (detector_client_) {
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    try {
      auto future = detector_client_->invoke(
        req,
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(detector_service_timeout_)));

      if (!future || !future->success) {
        RCLCPP_ERROR(
          node_->get_logger(), "Detector service '%s' failed to start.",
          detector_service_name_.c_str());
        return false;
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        node_->get_logger(), "Calling detector service '%s' failed: %s",
        detector_service_name_.c_str(), e.what());
      return false;
    }
  }

  // 2. Subscription toggle
  //    Only subscribe once; will set state to ON on first message
  if (subscribe_toggle_ && !dock_pose_sub_) {
    dock_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "detected_dock_pose",
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
        detected_dock_pose_ = *pose;
        initial_pose_received_ = true;
      },
      nav2::qos::StandardTopicQoS());
  }

  detection_started_ = true;
  RCLCPP_INFO(node_->get_logger(), "External detector activation requested.");
  return true;
}

bool SimpleChargingDock::stopDetectionProcess()
{
  // Skip if already OFF
  if (!detection_started_) {
    return true;
  }

  // 1. Service STOP request
  if (detector_client_) {
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    try {
      auto future = detector_client_->invoke(
        req,
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(detector_service_timeout_)));

      if (!future || !future->success) {
        RCLCPP_ERROR(
          node_->get_logger(), "Detector service '%s' failed to stop.",
          detector_service_name_.c_str());
        return false;
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        node_->get_logger(), "Calling detector service '%s' failed: %s",
        detector_service_name_.c_str(), e.what());
      return false;
    }
  }

  // 2. Unsubscribe to release resources
  //    reset() will tear down the topic subscription immediately
  if (subscribe_toggle_ && dock_pose_sub_) {
    dock_pose_sub_.reset();
  }

  detection_started_ = false;
  initial_pose_received_ = false;
  RCLCPP_INFO(node_->get_logger(), "External detector deactivation requested.");
  return true;
}

void SimpleChargingDock::activate()
{
  dock_pose_pub_->on_activate();
  filtered_dock_pose_pub_->on_activate();
  staging_pose_pub_->on_activate();
}

void SimpleChargingDock::deactivate()
{
  stopDetectionProcess();
  dock_pose_pub_->on_deactivate();
  filtered_dock_pose_pub_->on_deactivate();
  staging_pose_pub_->on_deactivate();
  RCLCPP_DEBUG(node_->get_logger(), "SimpleChargingDock deactivated");
}

void SimpleChargingDock::cleanup()
{
  detector_client_.reset();
  dock_pose_sub_.reset();
  RCLCPP_DEBUG(node_->get_logger(), "SimpleChargingDock cleaned up");
}

}  // namespace opennav_docking

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(opennav_docking::SimpleChargingDock, opennav_docking_core::ChargingDock)
