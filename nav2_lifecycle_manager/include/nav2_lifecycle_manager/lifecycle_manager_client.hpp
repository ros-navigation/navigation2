// Copyright (c) 2019 Intel Corporation
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

#ifndef NAV2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_CLIENT_HPP_
#define NAV2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_CLIENT_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "nav2_lifecycle_manager/visibility_control.h"

namespace nav2_lifecycle_manager
{
/**
 * @enum nav2_lifecycle_manager::SystemStatus
 * @brief Enum class representing the status of the system.
 */
enum class SystemStatus {ACTIVE, INACTIVE, TIMEOUT};
/**
 * @class nav2_lifecycle_manager::LifeCycleMangerClient
 * @brief The LifecycleManagerClient sends requests to the LifecycleManager to
 * control the lifecycle state of the navigation modules.
 */
class NAV2_LIFECYCLE_MANAGER_PUBLIC LifecycleManagerClient
{
public:
  /**
   * @brief A constructor for LifeCycleMangerClient
   */
  explicit LifecycleManagerClient(const std::string & name);

  // Client-side interface to the Nav2 lifecycle manager
  /**
   * @brief Make start up service call
   * @return true or false
   */
  bool startup();
  /**
   * @brief Make shutdown service call
   * @return true or false
   */
  bool shutdown();
  /**
   * @brief Make pause service call
   * @return true or false
   */
  bool pause();
  /**
   * @brief Make resume service call
   * @return true or false
   */
  bool resume();
  /**
   * @brief Make reset service call
   * @return true or false
   */
  bool reset();
  /**
   * @brief Check if lifecycle node manager server is active
   * @return ACTIVE or INACTIVE or TIMEOUT
   */
  SystemStatus is_active(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  // A couple convenience methods to facilitate scripting tests
  /**
   * @brief Set initial pose with covariance
   * @param x X position
   * @param y Y position
   * @param theta orientation
   */
  void set_initial_pose(double x, double y, double theta);
  /**
   * @brief Send goal pose to NavigationToPose action server
   * @param x X position
   * @param y Y position
   * @param theta orientation
   * @return true or false
   */
  bool navigate_to_pose(double x, double y, double theta);

protected:
  using ManageLifecycleNodes = nav2_msgs::srv::ManageLifecycleNodes;

  /**
   * @brief A generic method used to call startup, shutdown, etc.
   * @param command
   */
  bool callService(uint8_t command);

  // The node to use for the service call
  rclcpp::Node::SharedPtr node_;

  rclcpp::Client<ManageLifecycleNodes>::SharedPtr manager_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr is_active_client_;
  std::string manage_service_name_;
  std::string active_service_name_;

  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

  // For convenience, this client also supports sending the initial pose
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher_;

  // Also, for convenience, this client supports invoking the NavigateToPose action
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigate_action_client_;
};

}  // namespace nav2_lifecycle_manager

#endif  // NAV2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_CLIENT_HPP_
