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

#ifndef NAV2_BEHAVIOR_TREE__FIRST_COMPLETION_HPP_
#define NAV2_BEHAVIOR_TREE__FIRST_COMPLETION_HPP_

#include "behaviortree_cpp/control_node.h"

namespace nav2_behavior_tree
{

/** @brief Type of parallel node that returns the status of the first child to stop running
 *
 * Usage in XML: <FirstCompletion>
 */
class FirstCompletion : public BT::ControlNode
{
  public:
    FirstCompletion(const std::string& name);
    FirstCompletion(const std::string& name, const BT::NodeConfiguration& config);
    virtual void halt() override;
    static BT::PortsList providedPorts() { return {}; }

  private:
    virtual BT::NodeStatus tick() override;
};

}  // namespace nav2_behavior_tree

#endif   // NAV2_BEHAVIOR_TREE__FIRST_COMPLETION_HPP_
