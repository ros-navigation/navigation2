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

#ifndef NAV2_UTIL__ROBOT_UTILS_HPP_
#define NAV2_UTIL__ROBOT_UTILS_HPP_

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_util
{
/**
* @brief get the current pose of the robot
* @param global_pose Pose to transform
* @param tf_buffer TF buffer to use for the transformation
* @param global_frame Frame to transform into
* @param robot_frame Frame to transform from
* @param transform_timeout TF Timeout to use for transformation
* @return bool Whether it could be transformed successfully
*/
bool getCurrentPose(
  geometry_msgs::msg::PoseStamped & global_pose,
  tf2_ros::Buffer & tf_buffer, const std::string global_frame = "map",
  const std::string robot_frame = "base_link", const double transform_timeout = 0.1,
  const rclcpp::Time stamp = rclcpp::Time());

/**
* @brief get an arbitrary pose in a target frame
* @param input_pose Pose to transform
* @param transformed_pose Output transformation
* @param tf_buffer TF buffer to use for the transformation
* @param target_frame Frame to transform into
* @param transform_timeout TF Timeout to use for transformation
* @return bool Whether it could be transformed successfully
*/
bool transformPoseInTargetFrame(
  const geometry_msgs::msg::PoseStamped & input_pose,
  geometry_msgs::msg::PoseStamped & transformed_pose,
  tf2_ros::Buffer & tf_buffer, const std::string target_frame,
  const double transform_timeout = 0.1);

/**
 * @brief Obtains a transform from source_frame_id at source_time ->
 * to target_frame_id at target_time time
 * @param source_frame_id Source frame ID to convert from
 * @param source_time Source timestamp to convert from
 * @param target_frame_id Target frame ID to convert to
 * @param target_time Target time to interpolate to
 * @param transform_tolerance Transform tolerance
 * @param tf_transform Output source->target transform
 * @return True if got correct transform, otherwise false
 */

/**
 * @brief Obtains a transform from source_frame_id -> to target_frame_id
 * @param source_frame_id Source frame ID to convert from
 * @param target_frame_id Target frame ID to convert to
 * @param transform_tolerance Transform tolerance
 * @param tf_buffer TF buffer to use for the transformation
 * @param tf_transform Output source->target transform
 * @return True if got correct transform, otherwise false
 */
bool getTransform(
  const std::string & source_frame_id,
  const std::string & target_frame_id,
  const tf2::Duration & transform_tolerance,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  tf2::Transform & tf2_transform);

/**
 * @brief Obtains a transform from source_frame_id at source_time ->
 * to target_frame_id at target_time time
 * @param source_frame_id Source frame ID to convert from
 * @param source_time Source timestamp to convert from
 * @param target_frame_id Target frame ID to convert to
 * @param target_time Current node time to interpolate to
 * @param fixed_frame_id The frame in which to assume the transform is constant in time
 * @param transform_tolerance Transform tolerance
 * @param tf_buffer TF buffer to use for the transformation
 * @param tf_transform Output source->target transform
 * @return True if got correct transform, otherwise false
 */
bool getTransform(
  const std::string & source_frame_id,
  const rclcpp::Time & source_time,
  const std::string & target_frame_id,
  const rclcpp::Time & target_time,
  const std::string & fixed_frame_id,
  const tf2::Duration & transform_tolerance,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  tf2::Transform & tf2_transform);

/**
 * @brief Validates a twist message contains no nans or infs
 * @param msg Twist message to validate
 * @return True if valid, false if contains unactionable values
 */
bool validateTwist(const geometry_msgs::msg::Twist & msg);

}  // end namespace nav2_util

#endif  // NAV2_UTIL__ROBOT_UTILS_HPP_
