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

#include "nav2_util/lifecycle_service_client.hpp"

#include <string>
#include <chrono>
#include <memory>

#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

using nav2_util::generate_internal_node;
using std::chrono::seconds;
using std::make_shared;
using std::string;
using namespace std::chrono_literals;

namespace nav2_util
{

LifecycleServiceClient::LifecycleServiceClient(const string & lifecycle_node_name)
: node_(generate_internal_node(lifecycle_node_name + "_lifecycle_client")),
  change_state_(lifecycle_node_name + "/change_state", node_),
  get_state_(lifecycle_node_name + "/get_state", node_)
{
  // Block until server is up
  rclcpp::Rate r(20);
  while (!get_state_.wait_for_service(2s)) {
    RCLCPP_INFO(
      node_->get_logger(), "Waiting for service %s...", get_state_.getServiceName().c_str());
    r.sleep();
  }
}

LifecycleServiceClient::LifecycleServiceClient(
  const string & lifecycle_node_name,
  rclcpp::Node::SharedPtr parent_node)
: node_(parent_node),
  change_state_(lifecycle_node_name + "/change_state", node_),
  get_state_(lifecycle_node_name + "/get_state", node_)
{
  // Block until server is up
  rclcpp::Rate r(20);
  while (!get_state_.wait_for_service(2s)) {
    RCLCPP_INFO(
      node_->get_logger(), "Waiting for service %s...", get_state_.getServiceName().c_str());
    r.sleep();
  }
}

bool LifecycleServiceClient::change_state(
  const uint8_t transition,
  const seconds timeout)
{
  if (!change_state_.wait_for_service(timeout)) {
    throw std::runtime_error("change_state service is not available!");
  }

  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;
  auto response = change_state_.invoke(request, timeout);
  return response.get();
}

bool LifecycleServiceClient::change_state(
  std::uint8_t transition)
{
  if (!change_state_.wait_for_service(5s)) {
    throw std::runtime_error("change_state service is not available!");
  }

  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  auto response = std::make_shared<lifecycle_msgs::srv::ChangeState::Response>();
  request->transition.id = transition;
  return change_state_.invoke(request, response);
}

uint8_t LifecycleServiceClient::get_state(
  const seconds timeout)
{
  if (!get_state_.wait_for_service(timeout)) {
    throw std::runtime_error("get_state service is not available!");
  }

  auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  auto result = get_state_.invoke(request, timeout);
  return result->current_state.id;
}

}  // namespace nav2_util
