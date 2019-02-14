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

#include <string>
#include <thread>
#include <chrono>
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav2_util/lifecycle_service_client.hpp"

using std::string;

namespace nav2_util
{

void bringupLifecycleNode(const std::string & node_name)
{
  LifecycleServiceClient sc(node_name);
  sc.ChangeState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  sc.ChangeState(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  while (sc.GetState() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void bringupLifecycleNodes(const std::vector<std::string> & node_names)
{
  for (const auto & node_name : node_names) {
    bringupLifecycleNode(node_name);
  }
}

}  // namespace nav2_util
