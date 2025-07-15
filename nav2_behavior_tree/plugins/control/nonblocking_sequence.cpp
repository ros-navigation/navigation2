// Copyright (c) 2025 Polymath Robotics
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

#include "nav2_behavior_tree/plugins/control/nonblocking_sequence.hpp"

namespace nav2_behavior_tree
{

NonblockingSequence::NonblockingSequence(const std::string & name)
: BT::ControlNode(name, {})
{
}

NonblockingSequence::NonblockingSequence(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ControlNode(name, conf)
{
}

BT::NodeStatus NonblockingSequence::tick()
{
  bool all_success = true;

  for (std::size_t i = 0; i < children_nodes_.size(); ++i) {
    auto status = children_nodes_[i]->executeTick();
    switch (status) {
      case BT::NodeStatus::FAILURE:
        ControlNode::haltChildren();
        all_success = false;  // probably not needed
        return status;
      case BT::NodeStatus::SUCCESS:
        break;
      case BT::NodeStatus::RUNNING:
        all_success = false;
        break;
      default:
        std::stringstream error_msg;
        error_msg << "Invalid node status. Received status " << status <<
          "from child " << children_nodes_[i]->name();
        throw std::runtime_error(error_msg.str());
    }
  }

  // Wrap up.
  if (all_success) {
    ControlNode::haltChildren();
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

}  // namespace nav2_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::NonblockingSequence>("NonblockingSequence");
}
