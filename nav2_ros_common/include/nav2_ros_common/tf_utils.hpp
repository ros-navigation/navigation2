// Copyright (c) 2026 Open Navigation LLC
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

#ifndef NAV2_ROS_COMMON__TF_UTILS_HPP_
#define NAV2_ROS_COMMON__TF_UTILS_HPP_

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_ros_common/tf2_factories.hpp"
#include "rclcpp/time.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2/exceptions.hpp"
#include "tf2/time.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2
{

/**
 * @brief Convert a stamped transform to the equivalent stamped pose.
 * @param transform Transform to convert.
 * @return Pose containing the transform header, translation, and rotation.
 *   The child frame ID is not represented in PoseStamped.
 */
[[nodiscard]] inline geometry_msgs::msg::PoseStamped transformToPoseStamped(
  const geometry_msgs::msg::TransformStamped & transform)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = transform.header;
  pose.pose.position.x = transform.transform.translation.x;
  pose.pose.position.y = transform.transform.translation.y;
  pose.pose.position.z = transform.transform.translation.z;
  pose.pose.orientation = transform.transform.rotation;
  return pose;
}

/**
 * @brief Convert a stamped pose to a transform with the specified child frame.
 * @param pose Pose of the child frame expressed in the pose header's frame.
 * @param child_frame Child frame ID for the transform.
 * @return Transform containing the pose header, position, and orientation.
 */
[[nodiscard]] inline geometry_msgs::msg::TransformStamped poseToTransformStamped(
  const geometry_msgs::msg::PoseStamped & pose,
  const std::string & child_frame)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header = pose.header;
  transform.child_frame_id = child_frame;
  transform.transform.translation.x = pose.pose.position.x;
  transform.transform.translation.y = pose.pose.position.y;
  transform.transform.translation.z = pose.pose.position.z;
  transform.transform.rotation = pose.pose.orientation;
  return transform;
}

/**
 * @brief Get the latest transform from source_frame into target_frame.
 *
 * Uses tf2::TimePointZero to select the latest common time along the TF chain.
 * Does not wait for unavailable TF. Zero-stamped results (including static-only
 * chains) are exempt from the age check. Future-dated results pass the age check.
 * Equal frames return an identity transform stamped with current_time;
 * otherwise the TF timestamp is preserved. Does not log lookup failures.
 *
 * @param tf_buffer TF buffer used for the lookup.
 * @param target_frame Frame to transform into.
 * @param source_frame Frame to transform from.
 * @param current_time Reference time for the age check and identity timestamp;
 *   must use the same time domain as TF.
 * @param[out] transform Result; unchanged on failure.
 * @param staleness_threshold Maximum age in seconds; non-positive disables the
 *   check. A result exactly at the threshold is accepted.
 * @return true on success; false on TF failure or stale TF.
 */
[[nodiscard]] inline bool getLatestTransform(
  nav2::TransformBuffer & tf_buffer,
  const std::string & target_frame,
  const std::string & source_frame,
  const rclcpp::Time & current_time,
  geometry_msgs::msg::TransformStamped & transform,
  double staleness_threshold = 0.0)
{
  geometry_msgs::msg::TransformStamped result;
  if (target_frame == source_frame) {
    result.header.frame_id = target_frame;
    result.header.stamp = current_time;
    result.child_frame_id = source_frame;
    result.transform.rotation.w = 1.0;
  } else {
    try {
      result = tf_buffer.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      return false;
    }

    const bool has_timestamp = result.header.stamp.sec != 0 || result.header.stamp.nanosec != 0;
    if (staleness_threshold > 0.0 && has_timestamp) {
      const rclcpp::Time transform_time(result.header.stamp, current_time.get_clock_type());
      if ((current_time - transform_time).seconds() > staleness_threshold) {
        return false;
      }
    }
  }
  transform = result;
  return true;
}

/**
 * @brief Get the latest pose of source_frame expressed in target_frame.
 *
 * Wraps getLatestTransform and converts its translation and rotation into the
 * pose of the source frame's origin and axes. Preserves the transform header.
 * Does not transform an arbitrary input pose or wait for unavailable TF.
 *
 * @param tf_buffer TF buffer used for the lookup.
 * @param target_frame Frame in which to express the pose.
 * @param source_frame Frame whose pose is requested.
 * @param current_time Reference time for the age check and identity timestamp;
 *   must use the same time domain as TF.
 * @param[out] pose Result; unchanged on failure.
 * @param staleness_threshold Maximum age in seconds; non-positive disables the
 *   check. Zero-stamped results are exempt from the age check.
 * @return true if getLatestTransform succeeds; false otherwise.
 */
[[nodiscard]] inline bool getLatestPose(
  nav2::TransformBuffer & tf_buffer,
  const std::string & target_frame,
  const std::string & source_frame,
  const rclcpp::Time & current_time,
  geometry_msgs::msg::PoseStamped & pose,
  double staleness_threshold = 0.0)
{
  geometry_msgs::msg::TransformStamped transform;
  if (!getLatestTransform(
      tf_buffer, target_frame, source_frame, current_time, transform, staleness_threshold))
  {
    return false;
  }
  pose = transformToPoseStamped(transform);
  return true;
}

/**
 * @brief Get a transform from source_frame into target_frame at a specified time.
 *
 * Interpolates within available TF history when necessary. Does not apply an
 * age check against the current clock or log lookup failures. Equal frames
 * return an identity transform stamped with stamp.
 *
 * @param tf_buffer TF buffer used for the lookup.
 * @param target_frame Frame to transform into.
 * @param source_frame Frame to transform from.
 * @param stamp Lookup timestamp in the TF time domain. Zero requests the latest
 *   common time along the TF chain, without a staleness check.
 * @param[out] transform Result; unchanged on failure.
 * @param transform_tolerance Maximum lookup wait duration; must be nonnegative.
 *   Zero performs the lookup without waiting. This does not permit timestamp
 *   error or extrapolation.
 * @return true on success; false on TF failure or invalid arguments.
 */
[[nodiscard]] inline bool getStampedTransform(
  nav2::TransformBuffer & tf_buffer,
  const std::string & target_frame,
  const std::string & source_frame,
  const rclcpp::Time & stamp,
  geometry_msgs::msg::TransformStamped & transform,
  const tf2::Duration & transform_tolerance = tf2::Duration::zero())
{
  geometry_msgs::msg::TransformStamped result;
  if (target_frame == source_frame) {
    result.header.frame_id = target_frame;
    result.header.stamp = stamp;
    result.child_frame_id = source_frame;
  } else {
    try {
      result = tf_buffer.lookupTransform(
        target_frame, source_frame, stamp, tf2_ros::toRclcpp(transform_tolerance));
    } catch (const tf2::TransformException &) {
      return false;
    }
  }
  transform = result;
  return true;
}

/**
 * @brief Get a transform between frames evaluated at different timestamps.
 *
 * Transforms from source_frame at source_time into target_frame at target_time,
 * using fixed_frame as the reference across time. No current-clock age check
 * is applied and lookup failures are not logged. Equal frame names do not
 * imply identity when the times differ.
 *
 * @param tf_buffer TF buffer used for the lookup.
 * @param target_frame Frame to transform into.
 * @param target_time Target timestamp in the TF time domain; zero requests the
 *   latest available transform between target_frame and fixed_frame.
 * @param source_frame Frame to transform from.
 * @param source_time Source timestamp in the TF time domain; zero requests the
 *   latest available transform between source_frame and fixed_frame.
 * @param fixed_frame Frame assumed constant across the two timestamps.
 * @param[out] transform Result; unchanged on failure.
 * @param transform_tolerance Maximum lookup wait duration; must be nonnegative.
 *   Zero performs the lookup without waiting.
 * @return true on success; false on TF failure or invalid arguments.
 */
[[nodiscard]] inline bool getStampedTransform(
  nav2::TransformBuffer & tf_buffer,
  const std::string & target_frame,
  const rclcpp::Time & target_time,
  const std::string & source_frame,
  const rclcpp::Time & source_time,
  const std::string & fixed_frame,
  geometry_msgs::msg::TransformStamped & transform,
  const tf2::Duration & transform_tolerance = tf2::Duration::zero())
{
  geometry_msgs::msg::TransformStamped result;
  try {
    result = tf_buffer.lookupTransform(
      target_frame, target_time, source_frame, source_time, fixed_frame,
      tf2_ros::toRclcpp(transform_tolerance));
  } catch (const tf2::TransformException &) {
    return false;
  }
  transform = result;
  return true;
}

}  // namespace nav2

#endif  // NAV2_ROS_COMMON__TF_UTILS_HPP_
