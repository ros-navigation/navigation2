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

template<typename FutureT, typename WaitTimeT>
std::future_status
wait_for_result(FutureT & future, WaitTimeT time_to_wait)
{
  auto end = std::chrono::steady_clock::now() + time_to_wait;
  std::chrono::milliseconds wait_period(100);
  std::future_status status = std::future_status::timeout;

  do {
    auto now = std::chrono::steady_clock::now();
    auto time_left = end - now;
    if (time_left <= std::chrono::seconds(0)) {break;}
    status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
  } while (rclcpp::ok() && status != std::future_status::ready);

  return status;
}

LifecycleServiceClient::LifecycleServiceClient(
  rclcpp::Node::SharedPtr node,
  const std::string & target_node_name)
: node_(node), target_node_name_(target_node_name)
{
  std::string change_state_topic(std::string("/") + target_node_name_ + "/change_state");
  client_change_state_ = node_->create_client<lifecycle_msgs::srv::ChangeState>(
    change_state_topic);
}

bool
LifecycleServiceClient::changeState(std::uint8_t transition, std::chrono::seconds /*time_out*/)
{
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;

  RCLCPP_DEBUG(node_->get_logger(), "change_state: async_send_request");
  auto future_result = client_change_state_->async_send_request(request);

  RCLCPP_DEBUG(node_->get_logger(), "change_state: wait_for_result");
  auto future_status = wait_for_result(future_result, 1000s); // time_out);

  if (future_status != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(),
      "%s: Server time out while trying to change state", target_node_name_.c_str());
    return false;
  }

  if (future_result.get()->success) {
    RCLCPP_DEBUG(node_->get_logger(), "Successfully transitioned '%s' to %s",
      target_node_name_.c_str(), transition_to_str(transition));
    return true;
  } else {
    RCLCPP_ERROR(
      node_->get_logger(), "Failed to transition '%s' to %s", target_node_name_.c_str(),
      transition_to_str(transition));
    return false;
  }
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
