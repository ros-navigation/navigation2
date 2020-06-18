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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__ROUND_ROBIN_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__ROUND_ROBIN_NODE_HPP_

#include <string>

#include "behaviortree_cpp_v3/control_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace nav2_behavior_tree
{

class RoundRobinNode : public BT::ControlNode
{
public:
  explicit RoundRobinNode(const std::string & name);

  BT::NodeStatus tick() override;
  void halt() override;

private:
  unsigned int current_child_idx_{0};
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__ROUND_ROBIN_NODE_HPP_
