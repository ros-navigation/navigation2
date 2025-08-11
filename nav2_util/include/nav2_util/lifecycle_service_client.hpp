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

#ifndef NAV2_UTIL__LIFECYCLE_SERVICE_CLIENT_HPP_
#define NAV2_UTIL__LIFECYCLE_SERVICE_CLIENT_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav2_ros_common/service_client.hpp"
#include "nav2_ros_common/node_utils.hpp"


namespace nav2_util
{

using namespace std::chrono_literals;  // NOLINT

/// Helper functions to interact with a lifecycle node.
class LifecycleServiceClient
{
public:
  explicit LifecycleServiceClient(
    const std::string & lifecycle_node_name);

  template<typename NodeT>
  explicit
  LifecycleServiceClient(
    const string & lifecycle_node_name,
    NodeT parent_node)
  : change_state_(lifecycle_node_name + "/change_state", parent_node,
      true /*creates and spins an internal executor*/),
    get_state_(lifecycle_node_name + "/get_state", parent_node,
      true /*creates and spins an internal executor*/)
  {
    // Block until server is up
    rclcpp::Rate r(20);
    while (!get_state_.wait_for_service(2s)) {
      RCLCPP_INFO(
        parent_node->get_logger(),
        "Waiting for service %s...", get_state_.getServiceName().c_str());
      r.sleep();
    }
  }

  ~LifecycleServiceClient()
  {
    change_state_.stop();
    get_state_.stop();
  }

  /// Trigger a state change
  /**
   * Throws std::runtime_error on failure
   */
  bool change_state(
    const uint8_t transition,  // takes a lifecycle_msgs::msg::Transition id
    const std::chrono::milliseconds transition_timeout = std::chrono::milliseconds(-1),
    const std::chrono::milliseconds wait_for_service_timeout = std::chrono::milliseconds(5000));

  /// Get the current state as a lifecycle_msgs::msg::State id value
  /**
   * Throws std::runtime_error on failure
   */
  uint8_t get_state(const std::chrono::milliseconds timeout = std::chrono::milliseconds(2000));

protected:
  rclcpp::Node::SharedPtr node_;  // Node only used if parent_node is not provided
  nav2::ServiceClient<lifecycle_msgs::srv::ChangeState> change_state_;
  nav2::ServiceClient<lifecycle_msgs::srv::GetState> get_state_;
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__LIFECYCLE_SERVICE_CLIENT_HPP_
