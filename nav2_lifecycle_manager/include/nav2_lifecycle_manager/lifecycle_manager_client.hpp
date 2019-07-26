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

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/empty.hpp"

namespace nav2_lifecycle_manager
{

class LifecycleManagerClient
{
public:
  LifecycleManagerClient();

  // Client-side interface to the Nav2 lifecycle manager
  void startup();
  void shutdown();

  // A couple convenience methods to facilitate scripting tests
  void set_initial_pose(double x, double y, double theta);
  bool navigate_to_pose(double x, double y, double theta);

protected:
  using Empty = std_srvs::srv::Empty;

  // A generic method used to call startup, shutdown, etc.
  void callService(rclcpp::Client<Empty>::SharedPtr service_client, const char * service_name);

  // The node to use for the service call
  rclcpp::Node::SharedPtr node_;

  // The same (empty) request for all of the services
  std::shared_ptr<Empty::Request> request_;

  // The service clients
  rclcpp::Client<Empty>::SharedPtr startup_client_;
  rclcpp::Client<Empty>::SharedPtr shutdown_client_;

  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

  // For convenience, this client also supports sending the initial pose
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher_;

  // Also, for convenience, this client supports invoking the NavigateToPose action
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigate_action_client_;

  // A convenience function to convert from an algle to a Quaternion
  geometry_msgs::msg::Quaternion orientationAroundZAxis(double angle);
};

}  // namespace nav2_lifecycle_manager

#endif  // NAV2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_CLIENT_HPP_
