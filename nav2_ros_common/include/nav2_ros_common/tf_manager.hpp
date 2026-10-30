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

#ifndef NAV2_ROS_COMMON__TF_MANAGER_HPP_
#define NAV2_ROS_COMMON__TF_MANAGER_HPP_

#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/callback_group.hpp"
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
#include "tf2_ros/create_timer_ros.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace nav2
{

/** @brief Control diagnostic logging for an individual TF lookup. */
enum class Log {Enabled, Disabled};

/**
 * @brief Own TF reception, broadcasting, lookups, and pose conversions for a node.
 *
 * Lookup failures and invalid arguments return std::nullopt and are logged by default.
 */
class TransformManager
{
public:
  using SharedPtr = std::shared_ptr<TransformManager>;

  /**
   * @brief Create a TF manager using the node's clock, logger, and interfaces.
   * @param node Pointer-like node (shared_ptr or raw pointer).
   * @param activate_listener Start receiving transforms immediately.
   * @param spin_thread Spin a dedicated listener thread. When false, the caller
   *   must spin the node to receive transforms, including after startListener().
   * @param callback_group Optional callback group for the timer interface.
   * @throws std::invalid_argument if node is null.
   */
  template<typename NodeT>
  explicit TransformManager(
    const NodeT & node, bool activate_listener = true, bool spin_thread = true,
    rclcpp::CallbackGroup::SharedPtr callback_group = nullptr)
  : clock_(node ? node->get_clock() : nullptr),
    logger_(node ? node->get_logger() : rclcpp::get_logger("nav2_transform_manager")),
    spin_thread_(spin_thread)
  {
    if (!node) {
      throw std::invalid_argument("TransformManager requires a non-null node");
    }
    buffer_ = std::make_shared<tf2_ros::Buffer>(clock_);
    auto base = node->get_node_base_interface();
    auto logging = node->get_node_logging_interface();
    auto parameters = node->get_node_parameters_interface();
    auto topics = node->get_node_topics_interface();
#if RCLCPP_VERSION_GTE(30, 0, 0)
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(*node, callback_group);
#else
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      base, node->get_node_timers_interface(), callback_group);
#endif
    buffer_->setCreateTimerInterface(timer_interface);
    // Retain interfaces rather than the node, which may itself own this manager.
    create_listener_ = [this, base, logging, parameters, topics]() {
#if RCLCPP_VERSION_GTE(30, 0, 0)
        return std::make_unique<tf2_ros::TransformListener>(
          *buffer_, tf2_ros::TransformListener::RequiredInterfaces{
          base, logging, parameters, topics}, spin_thread_);
#else
        return std::make_unique<tf2_ros::TransformListener>(
          *buffer_, base, logging, parameters, topics, spin_thread_);
#endif
      };
    create_broadcaster_ = [parameters, topics]() {
#if RCLCPP_VERSION_GTE(30, 0, 0)
        return std::make_unique<tf2_ros::TransformBroadcaster>(
          tf2_ros::TransformBroadcaster::RequiredInterfaces{parameters, topics});
#else
        return std::make_unique<tf2_ros::TransformBroadcaster>(parameters, topics);
#endif
      };
    create_static_broadcaster_ = [parameters, topics]() {
#if RCLCPP_VERSION_GTE(30, 0, 0)
        return std::make_unique<tf2_ros::StaticTransformBroadcaster>(
          tf2_ros::StaticTransformBroadcaster::RequiredInterfaces{parameters, topics});
#else
        return std::make_unique<tf2_ros::StaticTransformBroadcaster>(parameters, topics);
#endif
      };
    if (activate_listener) {
      startListener();
    }
  }

  ~TransformManager()
  {
    stopListener();
  }

  TransformManager(const TransformManager &) = delete;
  TransformManager & operator=(const TransformManager &) = delete;

  /**
   * @brief Start receiving transforms, retaining any buffered transforms.
   * Has no effect if the listener is already running. Uses the threading mode
   * selected at construction. Serialize listener start and stop calls.
   */
  void startListener()
  {
    if (!listener_) {
      listener_ = create_listener_();
    }
  }

  /**
   * @brief Stop receiving transforms without clearing the buffer.
   * Use for lifecycle deactivation or before destroying dependent TF consumers.
   * Destruction also stops the listener. Has no effect if it is already stopped.
   */
  void stopListener()
  {
    if (listener_) {
      listener_.reset();
      buffer_->setUsingDedicatedThread(false);
    }
  }

  /** @brief Broadcast a dynamic transform on /tf, advertising on first use. */
  void sendTransform(const geometry_msgs::msg::TransformStamped & transform)
  {
    getBroadcaster().sendTransform(transform);
  }

  /** @brief Broadcast dynamic transforms together on /tf, advertising on first use. */
  void sendTransform(const std::vector<geometry_msgs::msg::TransformStamped> & transforms)
  {
    getBroadcaster().sendTransform(transforms);
  }

  /** @brief Broadcast a static transform on /tf_static, advertising on first use. */
  void sendStaticTransform(const geometry_msgs::msg::TransformStamped & transform)
  {
    std::lock_guard<std::mutex> lock(static_broadcaster_mutex_);
    getStaticBroadcaster().sendTransform(transform);
  }

  /** @brief Broadcast static transforms together on /tf_static, advertising on first use. */
  void sendStaticTransform(const std::vector<geometry_msgs::msg::TransformStamped> & transforms)
  {
    std::lock_guard<std::mutex> lock(static_broadcaster_mutex_);
    getStaticBroadcaster().sendTransform(transforms);
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
   * @param transform_tolerance Maximum lookup wait duration in seconds; must be nonnegative.
   *   Zero performs the lookup without waiting. This does not permit timestamp
   *   error or extrapolation.
   * @param log Enable or disable failure diagnostics for this call and nested lookups.
   * @return Transform on success; std::nullopt on TF failure or invalid arguments.
   */
  [[nodiscard]] std::optional<geometry_msgs::msg::TransformStamped> getStampedTransform(
    const std::string & target_frame,
    const std::string & source_frame,
    const rclcpp::Time & stamp,
    double transform_tolerance,
    Log log = Log::Enabled) const
  {
    if (transform_tolerance < 0.0) {
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
          target_frame, source_frame, stamp, rclcpp::Duration::from_seconds(transform_tolerance));
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
   * @param transform_tolerance Maximum lookup wait duration in seconds; must be nonnegative.
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
    double transform_tolerance,
    Log log = Log::Enabled) const
  {
    if (transform_tolerance < 0.0) {
      RCLCPP_ERROR_EXPRESSION(logger_, log == Log::Enabled,
          "Transform tolerance must be nonnegative (target frame '%s')",
        target_frame.c_str());
      return std::nullopt;
    }
    geometry_msgs::msg::TransformStamped result;
    try {
      result = buffer_->lookupTransform(
        target_frame, target_time, source_frame, source_time, fixed_frame,
        rclcpp::Duration::from_seconds(transform_tolerance));
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
   * @param transform_tolerance Maximum lookup wait duration in seconds; must be nonnegative.
   *   Zero performs the lookup without waiting. This does not permit timestamp
   *   error or extrapolation.
   * @param log Enable or disable failure diagnostics for this call and nested lookups.
   * @return Transformed pose on success; std::nullopt on TF lookup failure.
   */
  [[nodiscard]] std::optional<geometry_msgs::msg::PoseStamped> transformStampedPose(
    const std::string & target_frame,
    const geometry_msgs::msg::PoseStamped & input,
    double transform_tolerance,
    Log log = Log::Enabled) const
  {
    if (transform_tolerance < 0.0) {
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
  tf2_ros::TransformBroadcaster & getBroadcaster()
  {
    std::call_once(broadcaster_once_, [this]() {broadcaster_ = create_broadcaster_();});
    return *broadcaster_;
  }

  tf2_ros::StaticTransformBroadcaster & getStaticBroadcaster()
  {
    std::call_once(static_broadcaster_once_, [this]() {
        static_broadcaster_ = create_static_broadcaster_();
      });
    return *static_broadcaster_;
  }

  rclcpp::Clock::SharedPtr clock_;
  tf2_ros::Buffer::SharedPtr buffer_;
  rclcpp::Logger logger_;
  bool spin_thread_{true};
  std::function<std::unique_ptr<tf2_ros::TransformListener>()> create_listener_;
  std::function<std::unique_ptr<tf2_ros::TransformBroadcaster>()> create_broadcaster_;
  std::function<std::unique_ptr<tf2_ros::StaticTransformBroadcaster>()> create_static_broadcaster_;
  std::once_flag broadcaster_once_;
  std::once_flag static_broadcaster_once_;
  // Static broadcasts update a retained message containing all published transforms.
  std::mutex static_broadcaster_mutex_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
};

/**
 * @brief Create a transform manager with timer interface configured.
 * @param node Pointer-like node (shared_ptr or raw pointer).
 * @param activate_listener Start receiving transforms immediately.
 * @param spin_thread Whether to spin a dedicated listener thread.
 * @param callback_group Optional callback group for the timer interface.
 * @return Shared pointer to the configured TransformManager.
 */
template<typename NodeT>
inline nav2::TransformManager::SharedPtr create_transform_manager(
  const NodeT & node, bool activate_listener = true, bool spin_thread = true,
  rclcpp::CallbackGroup::SharedPtr callback_group = nullptr)
{
  return std::make_shared<nav2::TransformManager>(
    node, activate_listener, spin_thread, callback_group);
}

}  // namespace nav2

#endif  // NAV2_ROS_COMMON__TF_MANAGER_HPP_
