// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2022 Pradheep Padmanabhan - Neobotix GmbH
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
#include <memory>

#include "std_msgs/msg/string.hpp"

#include "nav2_behavior_tree/plugins/action/controller_cancel_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

namespace nav2_behavior_tree
{

using std::placeholders::_1;

ControllerCancel::ControllerCancel(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
: BtServiceNode<lifecycle_msgs::srv::ChangeState>(service_node_name, conf)
{
}

void ControllerCancel::on_tick()
{
  request_->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::ControllerCancel>("CancelControl");
}
