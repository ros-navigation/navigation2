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

namespace nav2_lifecycle_manager
{

enum class SystemStatus {ACTIVE, INACTIVE, TIMEOUT};

class LifecycleManagerClient
{
public:
  LifecycleManagerClient();

  // Client-side interface to the Nav2 lifecycle manager
  bool startup();
  bool shutdown();
  bool pause();
  bool resume();
  bool reset();
  SystemStatus is_active(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  // A couple convenience methods to facilitate scripting tests
  void set_initial_pose(double x, double y, double theta);
  bool navigate_to_pose(double x, double y, double theta);

protected:
  using ManageLifecycleNodes = nav2_msgs::srv::ManageLifecycleNodes;

  // A generic method used to call startup, shutdown, etc.
  bool callService(uint8_t command);

  // The node to use for the service call
  rclcpp::Node::SharedPtr node_;

  rclcpp::Client<ManageLifecycleNodes>::SharedPtr manager_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr is_active_client_;
  std::string manage_service_name_{"lifecycle_manager/manage_nodes"};
  std::string active_service_name_{"lifecycle_manager/is_active"};

  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

  // For convenience, this client also supports sending the initial pose
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher_;

  // Also, for convenience, this client supports invoking the NavigateToPose action
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigate_action_client_;
};

}  // namespace nav2_lifecycle_manager

#endif  // NAV2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_CLIENT_HPP_
