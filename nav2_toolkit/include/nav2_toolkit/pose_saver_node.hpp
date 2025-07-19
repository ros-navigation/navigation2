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
 * @brief A utility node to periodically save and restore the robot's pose from a YAML file.
 *
 * This node subscribes to `amcl_pose`, stores the latest pose to a file periodically,
 * and can also restore that pose by calling the `set_initial_pose` service.
 *
 * Services:
 * - `start_pose_saver`: Start periodic pose saving.
 * - `stop_pose_saver`: Stop periodic pose saving.
 * - `localise_at_last_known_position`: Restore the saved pose via service call.
 */
class PoseSaverNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor.
   * @param options Node options used for configuring the ROS node.
   */
  explicit PoseSaverNode(const rclcpp::NodeOptions & options);

  // === Testing Interface Methods ===

  /// @brief Write last known pose to configured file path.
  void test_write_pose_to_file() {write_pose_to_file(pose_file_path_);}

  /// @brief Read pose from configured file path.
  geometry_msgs::msg::PoseWithCovarianceStamped test_read_pose_from_file()
  {
    return read_pose_from_file(pose_file_path_);
  }

  /// @brief Set the internal last pose (used in tests).
  void test_set_last_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr & pose)
  {last_pose_ = pose;}

  /// @brief Get the current pose file path (used in tests).
  std::string test_get_pose_file_path() const {return pose_file_path_;}

private:
  // === Callbacks ===

  /// @brief Callback to store incoming AMCL pose.
  void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  /// @brief Timer callback to periodically write pose to file.
  void timer_callback();

  /// @brief Handle service call to start saving poses.
  void start_service_cb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  /// @brief Handle service call to stop saving poses.
  void stop_service_cb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  /// @brief Handle service call to restore pose.
  void restore_service_cb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  /// @brief Check if pose publisher is available and attempt restoration.
  void pose_publisher_monitor_callback();

  /// @brief Perform initial client setup after construction.
  void post_init_setup();

  // === Helpers ===

  /// @brief Serialize current pose to file.
  void write_pose_to_file(const std::string & filepath);

  /// @brief Load pose from given file.
  geometry_msgs::msg::PoseWithCovarianceStamped read_pose_from_file(const std::string & filepath);

  /// @brief Restore pose from file and publish it using service call.
  bool restore_pose_from_file_and_publish();

  // === Members ===

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>
  ::SharedPtr sub_;    ///< Subscriber for AMCL pose.
  rclcpp::Client<nav2_msgs::srv::SetInitialPose>::SharedPtr set_pose_client_;
  rclcpp::TimerBase::SharedPtr timer_;  ///< Timer for periodic file save.
  rclcpp::TimerBase::SharedPtr post_init_timer_;  ///< Timer for deferred initialization.
  rclcpp::TimerBase::SharedPtr
    pose_publisher_monitor_timer_;  ///< Timer to monitor pose publisher readiness.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;   ///< Service to start saving.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;    ///< Service to stop saving.
  rclcpp::Service<std_srvs::srv::Trigger>
  ::SharedPtr restore_service_;    ///< Service to restore pose.
  geometry_msgs::msg::PoseWithCovarianceStamped
  ::SharedPtr last_pose_;    ///< Most recent pose message.
  std::string pose_file_path_;    ///< File path to store pose YAML.
  bool auto_restore_;             ///< Whether to auto-restore on launch.
  bool pose_restored_;            ///< Whether pose has already been restored.
  bool was_pose_pub_up_last_check_;  ///< Status of pose publisher last check.
};

}  // namespace nav2_toolkit

#endif  // NAV2_TOOLKIT__POSE_SAVER_NODE_HPP_
