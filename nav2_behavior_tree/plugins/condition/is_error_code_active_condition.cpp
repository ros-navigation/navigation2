// Copyright (c) 2022 Joshua Wallace
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include "nav2_behavior_tree/plugins/condition/is_error_code_active_condition.hpp"


namespace nav2_behavior_tree
{

IsErrorCodeActive::IsErrorCodeActive(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  getInput<std::set<int>>("error_codes_to_check", error_codes_to_check_);
}

BT::NodeStatus IsErrorCodeActive::tick()
{
  getInput<std::set<int>>("current_error_codes", current_error_codes_);

  for (const auto & error_code_to_check : error_codes_to_check_) {
    if (current_error_codes_.find(error_code_to_check) != current_error_codes_.end()) {
      return BT::NodeStatus::SUCCESS;
    }
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsErrorCodeActive>(
    "IsErrorCodeActive");
}
