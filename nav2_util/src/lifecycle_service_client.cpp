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

using nav2_util::generateInternalNode;
using std::string;
using std::chrono::seconds;
using std::make_shared;

namespace nav2_util
{

LifecycleServiceClient::LifecycleServiceClient(const string & node_name)
: node_(generateInternalNode(node_name + "_lifecycle_client")),
  change_state_(node_name + "/change_state", node_),
  get_state_(node_name + "/get_state", node_)
{
}

void LifecycleServiceClient::ChangeState(
  const uint8_t newState,
  const seconds timeout)
{
  change_state_.waitForService(timeout);
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = newState;
  change_state_.invoke(request, timeout);
}

uint8_t LifecycleServiceClient::GetState(
  const seconds timeout)
{
  get_state_.waitForService(timeout);
  auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  auto result = get_state_.invoke(request, timeout);
  return result->current_state.id;
}

}  // namespace nav2_util
