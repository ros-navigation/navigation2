// Copyright (c) 2025 Pranav Kolekar
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

#ifndef NAV2_TOOLKIT__POSE_SAVER_NODE_HPP_
#define NAV2_TOOLKIT__POSE_SAVER_NODE_HPP_

#include <yaml-cpp/yaml.h>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/srv/set_initial_pose.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "nav2_ros_common/service_client.hpp"

namespace nav2_toolkit
{

/**
 * @class PoseSaverNode
 * @brief A utility node to periodically save and restore the robot's pose from a YAML file
 *
 * This node subscribes to the robot's pose topic (typically `amcl_pose`), stores the latest
 * pose to a YAML file periodically, and provides services to control pose saving/restoration.
 * The node can automatically restore the last known pose on startup and provides manual
 * restoration through service calls.
 *
 * Key Features:
 * - Periodic pose saving to YAML file with atomic write operations
 * - Automatic pose restoration on node startup (configurable)
 * - Service-based control for starting/stopping pose saving
 * - Manual pose restoration via service call
 * - Crash-safe file operations to prevent data corruption
 *
 * Services Provided:
 * - `start_pose_saver`: Start periodic pose saving
 * - `stop_pose_saver`: Stop periodic pose saving
 * - `localise_at_last_known_position`: Restore the saved pose via service call
 *
 * Topics Subscribed:
 * - `amcl_pose` (geometry_msgs::msg::PoseWithCovarianceStamped): Robot pose updates
 *
 * Service Clients:
 * - `/initialpose` (nav2_msgs::srv::SetInitialPose): To restore saved poses
 */
class PoseSaverNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for PoseSaverNode
   * @param options Node options used for configuring the ROS node
   */
  explicit PoseSaverNode(const rclcpp::NodeOptions & options);

protected:
  // === Callbacks ===

  /**
   * @brief Callback to store incoming robot pose messages
   * @param msg Shared pointer to the received pose message with covariance
   */
  void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  /**
   * @brief Timer callback to periodically write pose to file
   * 
   * Called at regular intervals (configurable via parameter) to save the most
   * recent pose to the configured YAML file path
   */
  void timer_callback();

  /**
   * @brief Handle service call to start saving poses
   * @param req Service request (unused, trigger type)
   * @param res Service response containing success status and message
   */
  void start_service_cb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  /**
   * @brief Handle service call to stop saving poses
   * @param req Service request (unused, trigger type)
   * @param res Service response containing success status and message
   */
  void stop_service_cb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  /**
   * @brief Handle service call to restore pose from saved file
   * @param req Service request (unused, trigger type)
   * @param res Service response containing success status and message
   */
  void restore_service_cb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  /**
   * @brief Monitor pose publisher availability and attempt restoration
   * 
   * Periodically checks if the pose publisher service is available and
   * performs automatic pose restoration if configured and not yet done
   */
  void pose_publisher_monitor_callback();

  /**
   * @brief Perform initial client setup after construction
   * 
   * Deferred initialization to ensure all ROS components are properly
   * set up before attempting service connections and restoration
   */
  void post_init_setup();

  // === File I/O Helpers ===

  /**
   * @brief Serialize current pose to YAML file with atomic operations
   * @param filepath Full path to the target YAML file
   * 
   * Uses atomic file operations (write to temp file, then rename) to
   * prevent data corruption in case of crashes during write operations
   */
  void write_pose_to_file(const std::string & filepath);

  /**
   * @brief Load pose from YAML file
   * @param filepath Full path to the source YAML file
   * @return Loaded pose with covariance, or default-constructed pose if file read fails
   * 
   * Safely loads pose data from YAML file with error handling for
   * malformed files or I/O errors
   */
  geometry_msgs::msg::PoseWithCovarianceStamped read_pose_from_file(const std::string & filepath);

  /**
   * @brief Restore pose from file and publish it using service call
   * @return True if restoration was successful, false otherwise
   * 
   * Combines file reading and service calling to restore the robot's
   * pose from the saved YAML file
   */
  bool restore_pose_from_file_and_publish();

private:
  // === ROS Interface Members ===

  /// Subscriber for robot pose updates
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_;

  /// Client for setting initial pose
  rclcpp::Client<nav2_msgs::srv::SetInitialPose>::SharedPtr set_pose_client_;

  /// Timer for periodic file save operations
  rclcpp::TimerBase::SharedPtr timer_;

  /// Timer for deferred initialization
  rclcpp::TimerBase::SharedPtr post_init_timer_;

  /// Timer to monitor pose publisher readiness
  rclcpp::TimerBase::SharedPtr pose_publisher_monitor_timer_;

  /// Service to start pose saving
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;

  /// Service to stop pose saving
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;

  /// Service to restore saved pose
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr restore_service_;

  // === State Members ===

  /// Most recent pose message received
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr last_pose_;

  /// File path to store pose YAML data
  std::string pose_file_path_;

  /// Whether to automatically restore pose on launch
  bool auto_restore_;

  /// Whether pose has already been restored this session
  bool pose_restored_;

  /// Status of pose publisher from last availability check
  bool was_pose_pub_up_last_check_;
};

}  // namespace nav2_toolkit

#endif  // NAV2_TOOLKIT__POSE_SAVER_NODE_HPP_
