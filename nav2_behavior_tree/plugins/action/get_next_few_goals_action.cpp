// Copyright (c) 2025 Open Navigation LLC
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
#include <limits>

#include "nav2_util/geometry_utils.hpp"

#include "nav2_behavior_tree/plugins/action/get_next_few_goals_action.hpp"

namespace nav2_behavior_tree
{

GetNextFewGoals::GetNextFewGoals(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
}

inline BT::NodeStatus GetNextFewGoals::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  nav_msgs::msg::Goals input_goals, output_goals;
  unsigned int num_goals;
  getInput("input_goals", input_goals);
  getInput("num_goals", num_goals);

  if (input_goals.goals.empty()) {
    return BT::NodeStatus::FAILURE;
  }

  output_goals.header = input_goals.header;
  for (unsigned int i = 0; i < num_goals && i < input_goals.goals.size(); ++i) {
    output_goals.goals.push_back(input_goals.goals[i]);
  }
  setOutput("output_goals", output_goals);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::GetNextFewGoals>("GetNextFewGoals");
}
