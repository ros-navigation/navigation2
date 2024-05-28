// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2019 Steven Macenski
// Copyright (c) 2019 Samsung Research America
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

#include <string>
#include <cmath>
#include <memory>

#include "tf2/convert.h"
#include "nav2_util/robot_utils.hpp"
#include "rclcpp/logger.hpp"

namespace nav2_util
{

bool getCurrentPose(
  geometry_msgs::msg::PoseStamped & global_pose,
  tf2_ros::Buffer & tf_buffer, const std::string global_frame,
  const std::string robot_frame, const double transform_timeout,
  const rclcpp::Time stamp)
{
  tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
  global_pose.header.frame_id = robot_frame;
  global_pose.header.stamp = stamp;

  return transformPoseInTargetFrame(
    global_pose, global_pose, tf_buffer, global_frame, transform_timeout);
}

bool transformPoseInTargetFrame(
  const geometry_msgs::msg::PoseStamped & input_pose,
  geometry_msgs::msg::PoseStamped & transformed_pose,
  tf2_ros::Buffer & tf_buffer, const std::string target_frame,
  const double transform_timeout)
{
  static rclcpp::Logger logger = rclcpp::get_logger("transformPoseInTargetFrame");

  try {
    transformed_pose = tf_buffer.transform(
      input_pose, target_frame,
      tf2::durationFromSec(transform_timeout));
    return true;
  } catch (tf2::LookupException & ex) {
    RCLCPP_ERROR(
      logger,
      "No Transform available Error looking up target frame: %s\n", ex.what());
  } catch (tf2::ConnectivityException & ex) {
    RCLCPP_ERROR(
      logger,
      "Connectivity Error looking up target frame: %s\n", ex.what());
  } catch (tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(
      logger,
      "Extrapolation Error looking up target frame: %s\n", ex.what());
  } catch (tf2::TimeoutException & ex) {
    RCLCPP_ERROR(
      logger,
      "Transform timeout with tolerance: %.4f", transform_timeout);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(
      logger, "Failed to transform from %s to %s",
      input_pose.header.frame_id.c_str(), target_frame.c_str());
  }

  return false;
}

bool getTransform(
  const std::string & source_frame_id,
  const std::string & target_frame_id,
  const tf2::Duration & transform_tolerance,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  geometry_msgs::msg::TransformStamped & transform_msg)
{
  if (source_frame_id == target_frame_id) {
    // We are already in required frame
    return true;
  }

  try {
    // Obtaining the transform to get data from source to target frame
    transform_msg = tf_buffer->lookupTransform(
      target_frame_id, source_frame_id,
      tf2::TimePointZero, transform_tolerance);
  } catch (tf2::TransformException & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("getTransform"),
      "Failed to get \"%s\"->\"%s\" frame transform: %s",
      source_frame_id.c_str(), target_frame_id.c_str(), e.what());
    return false;
  }
  return true;
}

bool getTransform(
  const std::string & source_frame_id,
  const std::string & target_frame_id,
  const tf2::Duration & transform_tolerance,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  tf2::Transform & tf2_transform)
{
  tf2_transform.setIdentity();  // initialize by identical transform
  geometry_msgs::msg::TransformStamped transform;
  if (getTransform(source_frame_id, target_frame_id, transform_tolerance, tf_buffer, transform)) {
    // Convert TransformStamped to TF2 transform
    tf2::fromMsg(transform.transform, tf2_transform);
    return true;
  }
  return false;
}

bool getTransform(
  const std::string & source_frame_id,
  const rclcpp::Time & source_time,
  const std::string & target_frame_id,
  const rclcpp::Time & target_time,
  const std::string & fixed_frame_id,
  const tf2::Duration & transform_tolerance,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  geometry_msgs::msg::TransformStamped & transform_msg)
{
  try {
    // Obtaining the transform to get data from source to target frame.
    // This also considers the time shift between source and target.
    transform_msg = tf_buffer->lookupTransform(
      target_frame_id, target_time,
      source_frame_id, source_time,
      fixed_frame_id, transform_tolerance);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(
      rclcpp::get_logger("getTransform"),
      "Failed to get \"%s\"->\"%s\" frame transform: %s",
      source_frame_id.c_str(), target_frame_id.c_str(), ex.what());
    return false;
  }

  return true;
}

bool getTransform(
  const std::string & source_frame_id,
  const rclcpp::Time & source_time,
  const std::string & target_frame_id,
  const rclcpp::Time & target_time,
  const std::string & fixed_frame_id,
  const tf2::Duration & transform_tolerance,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  tf2::Transform & tf2_transform)
{
  geometry_msgs::msg::TransformStamped transform;
  tf2_transform.setIdentity();  // initialize by identical transform
  if (getTransform(
      source_frame_id, source_time, target_frame_id, target_time, fixed_frame_id,
      transform_tolerance, tf_buffer, transform))
  {
    // Convert TransformStamped to TF2 transform
    tf2::fromMsg(transform.transform, tf2_transform);
    return true;
  }

  return false;
}

bool validateTwist(const geometry_msgs::msg::Twist & msg)
{
  if (std::isinf(msg.linear.x) || std::isnan(msg.linear.x)) {
    return false;
  }

  if (std::isinf(msg.linear.y) || std::isnan(msg.linear.y)) {
    return false;
  }

  if (std::isinf(msg.linear.z) || std::isnan(msg.linear.z)) {
    return false;
  }

  if (std::isinf(msg.angular.x) || std::isnan(msg.angular.x)) {
    return false;
  }

  if (std::isinf(msg.angular.y) || std::isnan(msg.angular.y)) {
    return false;
  }

  if (std::isinf(msg.angular.z) || std::isnan(msg.angular.z)) {
    return false;
  }

  return true;
}

bool validateTwist(const geometry_msgs::msg::TwistStamped & msg)
{
  return validateTwist(msg.twist);
}

}  // end namespace nav2_util
