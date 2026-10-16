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

#ifndef NAV2_ROS_COMMON__TF_BUFFER_HPP_
#define NAV2_ROS_COMMON__TF_BUFFER_HPP_

#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/version.h"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2/exceptions.hpp"
#include "tf2/time.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace nav2
{

/** @brief Control diagnostic logging for an individual TF lookup. */
enum class Log {Enabled, Disabled};

/**
 * @brief TF interface for Nav2 lookups, pose transforms, and conversions.
 *
 * Lookup failures and invalid arguments return std::nullopt and are logged by default.
 */
class TransformBuffer
{
public:
  using SharedPtr = std::shared_ptr<TransformBuffer>;

  /** @brief Create a buffer using the given clock and diagnostic logger. */
  explicit TransformBuffer(
    const rclcpp::Clock::SharedPtr & clock,
    const rclcpp::Logger & logger = rclcpp::get_logger("nav2_transform_buffer"))
  : clock_(clock), logger_(logger)
  {
    if (!clock_) {
      throw std::invalid_argument("TransformBuffer requires a non-null clock");
    }
    buffer_ = std::make_shared<tf2_ros::Buffer>(clock_);
  }

  TransformBuffer(const TransformBuffer &) = delete;
  TransformBuffer & operator=(const TransformBuffer &) = delete;

  /**
   * @brief Create and own a listener using the supplied node.
   * @param node Pointer-like node (shared_ptr or raw pointer).
   * @param spin_thread Whether to spin a dedicated listener thread.
   *   When false, the caller must spin the node to receive transforms.
   * @throws std::logic_error if a listener already exists. Reset it before recreating it.
   * Listener creation and reset must be serialized by the caller.
   */
  template<typename NodeT>
  void createListener(const NodeT & node, bool spin_thread = true)
  {
    if (listener_) {
      throw std::logic_error("TransformBuffer already has a listener");
    }
#if RCLCPP_VERSION_GTE(30, 0, 0)
    listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_, *node, spin_thread);
#else
    listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_, node, spin_thread);
#endif
  }

  /**
   * @brief Create and own a listener with its own hidden node and dedicated thread.
   * @throws std::logic_error if a listener already exists. Reset it before recreating it.
   */
  void createListener()
  {
    if (listener_) {
      throw std::logic_error("TransformBuffer already has a listener");
    }
    listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_);
  }

  /** @brief Stop and destroy the owned listener, retaining buffered transforms. */
  void resetListener()
  {
    if (listener_) {
      listener_.reset();
      buffer_->setUsingDedicatedThread(false);
    }
  }

  /** @brief Whether this buffer owns a listener. */
  [[nodiscard]] bool hasListener() const
  {
    return static_cast<bool>(listener_);
  }

  /**
   * @brief Access the native TF buffer for integrations unsupported by this interface.
   * @return Native buffer reference, valid while this wrapper owns the buffer.
   * Use only for native TF integrations, buffer configuration, and test setup.
   * Ordinary lookups and pose transforms should use this wrapper's methods.
   */
  [[nodiscard]] tf2_ros::Buffer & getRawBuffer()
  {
    return *buffer_;
  }

  [[nodiscard]] const tf2_ros::Buffer & getRawBuffer() const
  {
    return *buffer_;
  }

  /**
   * @brief Insert a transform into the buffer.
   * @param transform Stamped transform to insert.
   * @param authority Source of the transform, used in TF diagnostics.
   * @param is_static Whether the transform is valid at all timestamps.
   * @return True if TF accepted the transform; false if it was invalid.
   */
  bool setTransform(
    const geometry_msgs::msg::TransformStamped & transform,
    const std::string & authority, bool is_static = false)
  {
    return buffer_->setTransform(transform, authority, is_static);
  }

  /**
   * @brief Configure TF timeout checks for a separately serviced buffer.
   * @param using_dedicated_thread Whether another thread services TF subscriptions.
   * This informs TF about the threading arrangement; it does not start a thread.
   */
  void setUsingDedicatedThread(bool using_dedicated_thread)
  {
    buffer_->setUsingDedicatedThread(using_dedicated_thread);
  }

  /** @brief Clear transform data using the native TF buffer's reset behavior. */
  void clear()
  {
    buffer_->clear();
  }

  /**
   * @brief Wrap a configured TF buffer, retaining shared ownership.
   * @param buffer Buffer configured with the application's clock and timer interface.
   * @param clock The same clock used to construct the native buffer.
   * @param logger Logger used to report failures.
   * @throws std::invalid_argument if buffer or clock is null.
   */
  explicit TransformBuffer(
    const tf2_ros::Buffer::SharedPtr & buffer,
    const rclcpp::Clock::SharedPtr & clock,
    const rclcpp::Logger & logger = rclcpp::get_logger("nav2_transform_buffer"))
  : clock_(clock), buffer_(buffer), logger_(logger)
  {
    if (!buffer_ || !clock_) {
      RCLCPP_ERROR(logger_, "TransformBuffer requires a non-null TF buffer and clock");
      throw std::invalid_argument("TransformBuffer requires a non-null TF buffer and clock");
    }
  }

  /**
   * @brief Convert a stamped transform to the equivalent stamped pose.
   * @param transform Transform to convert.
   * @return Pose containing the transform header, translation, and rotation.
   *   The child frame ID is not represented in PoseStamped.
   */
  [[nodiscard]] static geometry_msgs::msg::PoseStamped transformToPoseStamped(
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
  [[nodiscard]] static geometry_msgs::msg::TransformStamped poseToTransformStamped(
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
   * @brief Invert a transform, exchanging parent and child frames and preserving its timestamp.
   * @param transform Transform to invert.
   * @return Inverse transform.
   */
  [[nodiscard]] static geometry_msgs::msg::TransformStamped invertTransform(
    const geometry_msgs::msg::TransformStamped & transform)
  {
    tf2::Transform tf_transform;
    tf2::fromMsg(transform.transform, tf_transform);
    geometry_msgs::msg::TransformStamped inverse;
    inverse.header.stamp = transform.header.stamp;
    inverse.header.frame_id = transform.child_frame_id;
    inverse.child_frame_id = transform.header.frame_id;
    inverse.transform = tf2::toMsg(tf_transform.inverse());
    return inverse;
  }

  /**
   * @brief Get the latest transform from source_frame into target_frame.
   *
   * Uses tf2::TimePointZero to select the latest common time along the TF chain.
   * Does not wait for unavailable TF. Static-only chains are exempt from the age check.
   * Staleness is measured against the stored clock at lookup time. Equal frames
   * return an identity transform stamped with the current clock time; otherwise
   * the TF timestamp is preserved.
   * Logs lookup failures and invalid arguments unless disabled.
   *
   * @param target_frame Frame to transform into.
   * @param source_frame Frame to transform from.
   * @param staleness_threshold Maximum age in seconds; non-positive disables the
   *   check. A result exactly at the threshold is accepted.
   * @param log Enable or disable failure diagnostics for this call and nested lookups.
   * @return Transform on success; std::nullopt on TF failure, stale TF, or a nonfinite threshold.
   */
  [[nodiscard]] std::optional<geometry_msgs::msg::TransformStamped> getLatestTransform(
    const std::string & target_frame,
    const std::string & source_frame,
    double staleness_threshold = 0.0,
    Log log = Log::Enabled) const
  {
    if (!std::isfinite(staleness_threshold)) {
      RCLCPP_ERROR_EXPRESSION(logger_, log == Log::Enabled,
          "Transform staleness threshold must be finite (target frame '%s')",
        target_frame.c_str());
      return std::nullopt;
    }
    geometry_msgs::msg::TransformStamped result;
    if (target_frame == source_frame) {
      result.header.frame_id = target_frame;
      result.header.stamp = clock_->now();
      result.child_frame_id = source_frame;
    } else {
      try {
        result = buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR_EXPRESSION(logger_, log == Log::Enabled,
            "Failed to get transform from '%s' to '%s': %s",
          source_frame.c_str(), target_frame.c_str(), ex.what());
        return std::nullopt;
      }

      const bool has_timestamp = rclcpp::Time(result.header.stamp).nanoseconds() != 0;
      if (staleness_threshold > 0.0 && has_timestamp) {
        const auto current_time = clock_->now();
        const rclcpp::Time transform_time(result.header.stamp, clock_->get_clock_type());
        const double age = (current_time - transform_time).seconds();
        if (age > staleness_threshold) {
          RCLCPP_ERROR_EXPRESSION(logger_, log == Log::Enabled,
            "Stale transform from '%s' to '%s': age %.6f s exceeds threshold %.6f s",
            source_frame.c_str(), target_frame.c_str(), age, staleness_threshold);
          return std::nullopt;
        }
      }
    }
    return result;
  }

  /**
   * @brief Get the latest pose of source_frame expressed in target_frame.
   *
   * Wraps getLatestTransform and converts its translation and rotation into the
   * pose of the source frame's origin and axes. Preserves the transform header.
   * Does not transform an arbitrary input pose or wait for unavailable TF.
   *
   * @param target_frame Frame in which to express the pose.
   * @param source_frame Frame whose pose is requested.
   * @param staleness_threshold Maximum age in seconds; non-positive disables the
   *   check. Zero-stamped results are exempt from the age check.
   * @param log Enable or disable failure diagnostics for this call and nested lookups.
   * @return Pose if getLatestTransform succeeds; std::nullopt otherwise.
   */
  [[nodiscard]] std::optional<geometry_msgs::msg::PoseStamped> getLatestPose(
    const std::string & target_frame,
    const std::string & source_frame,
    double staleness_threshold = 0.0,
    Log log = Log::Enabled) const
  {
    const auto transform = getLatestTransform(
      target_frame, source_frame, staleness_threshold, log);
    if (!transform) {
      return std::nullopt;
    }
    return transformToPoseStamped(*transform);
  }

  /**
   * @brief Get a transform from source_frame into target_frame at a specified time.
   *
   * Interpolates within available TF history when necessary. Does not apply an
   * age check against the current clock. Logs lookup failures unless disabled. Equal frames
   * return an identity transform stamped with stamp.
   *
   * @param target_frame Frame to transform into.
   * @param source_frame Frame to transform from.
   * @param stamp Lookup timestamp in the TF time domain. Zero requests the latest
   *   common time along the TF chain, without a staleness check.
   * @param transform_tolerance Maximum lookup wait duration; must be nonnegative.
   *   Zero performs the lookup without waiting. This does not permit timestamp
   *   error or extrapolation.
   * @param log Enable or disable failure diagnostics for this call and nested lookups.
   * @return Transform on success; std::nullopt on TF failure or invalid arguments.
   */
  [[nodiscard]] std::optional<geometry_msgs::msg::TransformStamped> getStampedTransform(
    const std::string & target_frame,
    const std::string & source_frame,
    const rclcpp::Time & stamp,
    const tf2::Duration & transform_tolerance,
    Log log = Log::Enabled) const
  {
    if (transform_tolerance < tf2::Duration::zero()) {
      RCLCPP_ERROR_EXPRESSION(logger_, log == Log::Enabled,
          "Transform tolerance must be nonnegative (target frame '%s')",
        target_frame.c_str());
      return std::nullopt;
    }
    geometry_msgs::msg::TransformStamped result;
    if (target_frame == source_frame) {
      result.header.frame_id = target_frame;
      result.header.stamp = stamp;
      result.child_frame_id = source_frame;
    } else {
      try {
        result = buffer_->lookupTransform(
          target_frame, source_frame, stamp, tf2_ros::toRclcpp(transform_tolerance));
      } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR_EXPRESSION(logger_, log == Log::Enabled,
            "Failed to get transform from '%s' to '%s': %s",
          source_frame.c_str(), target_frame.c_str(), ex.what());
        return std::nullopt;
      }
    }
    return result;
  }

  /**
   * @brief Get a transform between frames evaluated at different timestamps.
   *
   * Transforms from source_frame at source_time into target_frame at target_time,
   * using fixed_frame as the reference across time. No current-clock age check
   * is applied. Lookup failures are logged unless disabled. Equal frame names do not
   * imply identity when the times differ.
   *
   * Use this variant to express an earlier sensor observation in the robot
   * frame at a later time, accounting for robot motion through a fixed frame
   * such as odom.
   *
   * @param target_frame Frame to transform into.
   * @param target_time Target timestamp in the TF time domain; zero requests the
   *   latest available transform between target_frame and fixed_frame.
   * @param source_frame Frame to transform from.
   * @param source_time Source timestamp in the TF time domain; zero requests the
   *   latest available transform between source_frame and fixed_frame.
   * @param fixed_frame Frame assumed constant across the two timestamps.
   * @param transform_tolerance Maximum lookup wait duration; must be nonnegative.
   *   Zero performs the lookup without waiting.
   * @param log Enable or disable failure diagnostics for this call and nested lookups.
   * @return Transform on success; std::nullopt on TF failure or invalid arguments.
   */
  [[nodiscard]] std::optional<geometry_msgs::msg::TransformStamped> getStampedTransform(
    const std::string & target_frame,
    const rclcpp::Time & target_time,
    const std::string & source_frame,
    const rclcpp::Time & source_time,
    const std::string & fixed_frame,
    const tf2::Duration & transform_tolerance,
    Log log = Log::Enabled) const
  {
    if (transform_tolerance < tf2::Duration::zero()) {
      RCLCPP_ERROR_EXPRESSION(logger_, log == Log::Enabled,
          "Transform tolerance must be nonnegative (target frame '%s')",
        target_frame.c_str());
      return std::nullopt;
    }
    geometry_msgs::msg::TransformStamped result;
    try {
      result = buffer_->lookupTransform(
        target_frame, target_time, source_frame, source_time, fixed_frame,
        tf2_ros::toRclcpp(transform_tolerance));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR_EXPRESSION(logger_, log == Log::Enabled,
        "Failed to get transform from '%s' at %.9f to '%s' at %.9f via '%s': %s",
        source_frame.c_str(), source_time.seconds(), target_frame.c_str(), target_time.seconds(),
        fixed_frame.c_str(), ex.what());
      return std::nullopt;
    }
    return result;
  }

  /**
   * @brief Transform an arbitrary pose using TF at the input timestamp.
   *
   * Zero timestamps request the latest common TF time without an age check.
   * Equal frames return the input unchanged; otherwise the output header uses
   * the selected transform's frame and timestamp. Logs lookup failures unless disabled.
   *
   * @param target_frame Frame in which to express the input pose.
   * @param input Pose to transform; unchanged on success or failure.
   * @param transform_tolerance Maximum lookup wait duration; must be nonnegative.
   *   Zero performs the lookup without waiting. This does not permit timestamp
   *   error or extrapolation.
   * @param log Enable or disable failure diagnostics for this call and nested lookups.
   * @return Transformed pose on success; std::nullopt on TF lookup failure.
   */
  [[nodiscard]] std::optional<geometry_msgs::msg::PoseStamped> transformStampedPose(
    const std::string & target_frame,
    const geometry_msgs::msg::PoseStamped & input,
    const tf2::Duration & transform_tolerance,
    Log log = Log::Enabled) const
  {
    if (transform_tolerance < tf2::Duration::zero()) {
      RCLCPP_ERROR_EXPRESSION(logger_, log == Log::Enabled,
          "Transform tolerance must be nonnegative (target frame '%s')",
        target_frame.c_str());
      return std::nullopt;
    }
    if (input.header.frame_id == target_frame) {
      return input;
    }

    const auto transform = getStampedTransform(
      target_frame, input.header.frame_id,
      rclcpp::Time(input.header.stamp), transform_tolerance, log);
    if (!transform) {
      return std::nullopt;
    }

    geometry_msgs::msg::PoseStamped result;
    tf2::doTransform(input, result, *transform);
    return result;
  }

private:
  rclcpp::Clock::SharedPtr clock_;
  tf2_ros::Buffer::SharedPtr buffer_;
  rclcpp::Logger logger_;
  // Destroy the listener and join its thread before destroying the native buffer.
  std::unique_ptr<tf2_ros::TransformListener> listener_;
};

}  // namespace nav2

#endif  // NAV2_ROS_COMMON__TF_BUFFER_HPP_
