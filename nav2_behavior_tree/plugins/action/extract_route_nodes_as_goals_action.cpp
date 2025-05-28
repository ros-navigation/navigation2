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

#include "nav2_behavior_tree/plugins/action/extract_route_nodes_as_goals_action.hpp"

namespace nav2_behavior_tree
{

ExtractRouteNodesAsGoals::ExtractRouteNodesAsGoals(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
}

inline BT::NodeStatus ExtractRouteNodesAsGoals::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  nav2_msgs::msg::Route route;
  getInput("route", route);

  if (route.nodes.empty()) {
    return BT::NodeStatus::FAILURE;
  }

  nav_msgs::msg::Goals goals;
  goals.header = route.header;
  goals.goals.reserve(route.nodes.size());

  for (const auto & node : route.nodes) {
    geometry_msgs::msg::PoseStamped goal;
    goal.header = route.header;
    goal.pose.position.x = node.position.x;
    goal.pose.position.y = node.position.y;
    goals.goals.push_back(goal);
  }

  setOutput("goals", goals);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::ExtractRouteNodesAsGoals>(
    "ExtractRouteNodesAsGoals");
}
