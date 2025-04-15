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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__ARE_ERROR_CODES_PRESENT_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__ARE_ERROR_CODES_PRESENT_CONDITION_HPP_

#include <string>
#include <memory>
#include <vector>
#include <set>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"

namespace nav2_behavior_tree
{

class AreErrorCodesPresent : public BT::ConditionNode
{
public:
  AreErrorCodesPresent(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf)
  {
    std::vector<int> error_codes_to_check_vector;
    getInput("error_codes_to_check", error_codes_to_check_vector); //NOLINT

    error_codes_to_check_ = std::set<uint16_t>(
      error_codes_to_check_vector.begin(),
      error_codes_to_check_vector.end());
  }

  AreErrorCodesPresent() = delete;

  BT::NodeStatus tick()
  {
    getInput<uint16_t>("error_code", error_code_);  //NOLINT

    if (error_codes_to_check_.find(error_code_) != error_codes_to_check_.end()) {
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }

  static BT::PortsList providedPorts()
  {
    return
      {
        BT::InputPort<uint16_t>("error_code", "The active error codes"), //NOLINT
        BT::InputPort<std::vector<int>>("error_codes_to_check", "Error codes to check")//NOLINT
      };
  }

protected:
  uint16_t error_code_; //NOLINT
  std::set<uint16_t> error_codes_to_check_; //NOLINT
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__ARE_ERROR_CODES_PRESENT_CONDITION_HPP_
