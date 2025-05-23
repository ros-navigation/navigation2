// Copyright (C) 2025 Pranav Kolekar
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
#include <std_srvs/srv/trigger.hpp>

namespace nav2_toolkit
{
  /**
   * @class PoseSaverNode
   * @brief A ROS 2 node for saving and restoring robot poses.
   *
   * This node subscribes to pose messages, saves them to a file, and provides
   * services to start/stop saving, restore poses, and reset the pose file.
   */
  class PoseSaverNode : public rclcpp::Node
  {
  public:
    /**
     * @brief Constructor for PoseSaverNode.
     * @param options Node options for configuring the node.
     */
    explicit PoseSaverNode(const rclcpp::NodeOptions &options);

  private:
    /**
     * @brief Callback for receiving pose messages.
     * @param msg The received pose message.
     */
    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    /**
     * @brief Timer callback for periodic tasks.
     */
    void timer_callback();

    /**
     * @brief Service callback to start saving poses.
     * @param req The service request.
     * @param res The service response.
     */
    void start_service_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    /**
     * @brief Service callback to stop saving poses.
     * @param req The service request.
     * @param res The service response.
     */
    void stop_service_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    /**
     * @brief Service callback to restore a pose from the file.
     * @param req The service request.
     * @param res The service response.
     */
    void restore_service_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    /**
     * @brief Service callback to reset the pose file.
     * @param req The service request.
     * @param res The service response.
     */
    void reset_pose_file_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    /**
     * @brief Writes the current pose to a file.
     * @param filepath The file path to save the pose.
     */
    void write_pose_to_file(const std::string &filepath);

    /**
     * @brief Monitors AMCL (Adaptive Monte Carlo Localization) activity.
     */
    void amcl_monitor_callback();

    /**
     * @brief Restores a pose from the file and publishes it.
     * @return True if the pose was successfully restored, false otherwise.
     */
    bool restore_pose_from_file_and_publish();

    /**
     * @brief Reads a pose from a file.
     * @param filepath The file path to read the pose from.
     * @return The pose read from the file.
     */
    geometry_msgs::msg::PoseWithCovarianceStamped read_pose_from_file(const std::string &filepath);

    rclcpp::TimerBase::SharedPtr timer_; ///< Timer for periodic tasks.
    rclcpp::TimerBase::SharedPtr amcl_monitor_timer_; ///< Timer for monitoring AMCL activity.
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_; ///< Subscription to pose messages.
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_; ///< Service to start saving poses.
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_; ///< Service to stop saving poses.
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr restore_service_; ///< Service to restore poses.
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_; ///< Service to reset the pose file.
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_; ///< Publisher for initial poses.
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr last_pose_; ///< Last received pose.
    rclcpp::Time last_amcl_time_; ///< Last time AMCL was active.
    std::string pose_file_path_; ///< Path to the pose file.
    bool saving_active_; ///< Flag indicating if saving is active.
    bool auto_restore_; ///< Flag indicating if auto-restore is enabled.
    int last_sub_count_; ///< Last subscription count.
  };

} // namespace nav2_toolkit

#endif // NAV2_TOOLKIT__POSE_SAVER_NODE_HPP_