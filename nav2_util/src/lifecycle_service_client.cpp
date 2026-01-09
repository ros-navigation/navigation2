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

using nav2::generate_internal_node;
using std::chrono::milliseconds;
using std::make_shared;
using std::string;
using namespace std::chrono_literals;

namespace nav2_util
{

bool LifecycleServiceClient::change_state(
  const uint8_t transition,
  const milliseconds transition_timeout,
  const milliseconds wait_for_service_timeout)
{
  if (!change_state_.wait_for_service(wait_for_service_timeout)) {
    throw std::runtime_error("change_state service is not available!");
  }

  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;
  if (transition_timeout > 0ms) {
    auto response = change_state_.invoke(request, transition_timeout);
    return response.get();
  } else {
    auto response = std::make_shared<lifecycle_msgs::srv::ChangeState::Response>();
    return change_state_.invoke(request, response);
  }
}

uint8_t LifecycleServiceClient::get_state(
  const milliseconds timeout)
{
  if (!get_state_.wait_for_service(timeout)) {
    throw std::runtime_error("get_state service is not available!");
  }

  auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  auto result = get_state_.invoke(request, timeout);
  return result->current_state.id;
}

}  // namespace nav2_util
