// Copyright (c) 2022 Joshua Wallace
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_ERROR_CODE_ACTIVE_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_ERROR_CODE_ACTIVE_CONDITION_HPP_

#include <string>
#include <memory>
#include <vector>
#include <set>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace nav2_behavior_tree
{

class IsErrorCodeActive : public BT::ConditionNode
{
public:
  IsErrorCodeActive(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf)
  {
    getInput<std::set<int>>("error_codes_to_check", error_codes_to_check_);
  }

  IsErrorCodeActive() = delete;

  BT::NodeStatus tick()
  {
    getInput<std::set<int>>("current_error_codes", current_error_codes_);

    for (const auto & error_code_to_check : error_codes_to_check_) {
      if (current_error_codes_.find(error_code_to_check) != current_error_codes_.end()) {
        return BT::NodeStatus::SUCCESS;
      }
    }

    return BT::NodeStatus::FAILURE;
  }

  static BT::PortsList providedPorts()
  {
    return
      {
        BT::InputPort<std::set<int>>("current_error_codes"),
        BT::InputPort<std::set<int>>("error_codes_to_check")
      };
  }

protected:
  std::set<int> current_error_codes_;
  std::set<int> error_codes_to_check_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_ERROR_CODE_ACTIVE_CONDITION_HPP_
