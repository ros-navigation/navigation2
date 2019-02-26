// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include "nav2_lifecycle/lifecycle_service_client.hpp"

#include <chrono>
#include <memory>
#include <string>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"

using namespace std::chrono_literals;
using Transition = lifecycle_msgs::msg::Transition;

namespace nav2_lifecycle
{

LifecycleServiceClient::LifecycleServiceClient(
  rclcpp::Node::SharedPtr node,
  const std::string & target_node_name)
: node_(node), target_node_name_(target_node_name)
{
  std::string change_state_topic(std::string("/") + target_node_name_ + "/change_state");
  client_ = node_->create_client<lifecycle_msgs::srv::ChangeState>(
    change_state_topic);
}

bool
LifecycleServiceClient::changeState(std::uint8_t transition, std::chrono::seconds /*time_out*/)
{
  while (!client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      throw std::runtime_error("ServiceClient: service call interrupted while waiting for service");
    }
    RCLCPP_DEBUG(node_->get_logger(), "Waiting for service to appear...");
	rclcpp::spin_some(node_);
  }

  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;

  RCLCPP_DEBUG(node_->get_logger(), "change_state: async_send_request");
  auto future_result = client_->async_send_request(request);

  rclcpp::executor::FutureReturnCode status = rclcpp::executor::FutureReturnCode::TIMEOUT;
  do {
    status = rclcpp::spin_until_future_complete(node_, future_result, std::chrono::milliseconds(500));
  } while (rclcpp::ok() && status != rclcpp::executor::FutureReturnCode::SUCCESS);

  return true;
}

const char *
LifecycleServiceClient::transition_to_str(uint8_t transition)
{
  switch (transition) {
    case Transition::TRANSITION_CREATE:
      return "CREATE";
    case Transition::TRANSITION_CONFIGURE:
      return "CONFIGURE";
    case Transition::TRANSITION_CLEANUP:
      return "CLEANUP";
    case Transition::TRANSITION_ACTIVATE:
      return "ACTIVATE";
    case Transition::TRANSITION_DEACTIVATE:
      return "DEACTIVATE";
    case Transition::TRANSITION_UNCONFIGURED_SHUTDOWN:
      return "UNCONFIGURED_SHUTDOWN";
    case Transition::TRANSITION_INACTIVE_SHUTDOWN:
      return "INACTIVE_SHUTDOWN";
    case Transition::TRANSITION_DESTROY:
      return "DESTROY";
    default:
      return "Unknown state transition";
  }
}


}  // namespace nav2_lifecycle
