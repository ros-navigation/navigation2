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
#include "nav2_util/service_client.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_util
{

/// Helper functions to interact with a lifecycle node.
class LifecycleServiceClient
{
public:
  explicit LifecycleServiceClient(const std::string & lifecycle_node_name);
  LifecycleServiceClient(
    const std::string & lifecycle_node_name,
    rclcpp::Node::SharedPtr parent_node);

  /// Trigger a state change
  /**
   * Throws std::runtime_error on failure
   */
  void change_state(
    const uint8_t transition,  // takes a lifecycle_msgs::msg::Transition id
    const std::chrono::seconds timeout);

  /// Trigger a state change, returning result
  bool change_state(std::uint8_t transition);

  /// Get the current state as a lifecycle_msgs::msg::State id value
  /**
   * Throws std::runtime_error on failure
   */
  uint8_t get_state(const std::chrono::seconds timeout = std::chrono::seconds::max());

protected:
  rclcpp::Node::SharedPtr node_;
  ServiceClient<lifecycle_msgs::srv::ChangeState> change_state_;
  ServiceClient<lifecycle_msgs::srv::GetState> get_state_;
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__LIFECYCLE_SERVICE_CLIENT_HPP_
