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

#include "nav2_behavior_tree/plugins/action/append_goal_pose_to_goals_action.hpp"

namespace nav2_behavior_tree
{

AppendGoalPoseToGoals::AppendGoalPoseToGoals(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
}

inline BT::NodeStatus AppendGoalPoseToGoals::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  geometry_msgs::msg::PoseStamped goal_pose;
  getInput("goal_pose", goal_pose);
  nav_msgs::msg::Goals input, output;
  getInput("input_goals", input);

  output = input;
  output.goals.push_back(goal_pose);

  setOutput("output_goals", output);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::AppendGoalPoseToGoals>(
    "AppendGoalPoseToGoals");
}
