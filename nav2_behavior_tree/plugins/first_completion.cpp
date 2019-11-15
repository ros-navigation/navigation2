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

#include <stdexcept>
#include <sstream>
#include <string>
#include "nav2_behavior_tree/first_completion.hpp"

namespace nav2_behavior_tree
{

FirstCompletion::FirstCompletion(const std::string & name)
: BT::ControlNode(name, {})
{
}

FirstCompletion::FirstCompletion(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ControlNode(name, config)
{
}

BT::NodeStatus FirstCompletion::tick()
{
  // tick every child until one returns something other than RUNNING.
  for (const auto & child_node : children_nodes_) {
    auto status = child_node->executeTick();
    switch (status) {
      case BT::NodeStatus::SUCCESS:
      case BT::NodeStatus::FAILURE:
        haltChildren(0);
        return status;
        break;
      case BT::NodeStatus::RUNNING:
        // do nothing and continue on to the next child
        break;
      default:
        std::stringstream error_msg;
        error_msg << "Invalid node status. Received status " << status <<
          "from child " << child_node->name();
        throw std::runtime_error(error_msg.str());
        break;
    }
  }
  return BT::NodeStatus::RUNNING;
}

void FirstCompletion::halt()
{
  BT::ControlNode::halt();
}

}  // namespace nav2_behavior_tree
