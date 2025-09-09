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

#include <memory>
#include <string>

#include "nav2_behavior_tree/plugins/action/compute_route_action.hpp"

namespace nav2_behavior_tree
{

ComputeRouteAction::ComputeRouteAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<Action>(xml_tag_name, action_name, conf)
{
}

void ComputeRouteAction::on_tick()
{
  bool use_poses = false, use_start = false;
  getInput("use_poses", use_poses);
  if (use_poses) {
    goal_.use_poses = true;
    getInput("goal", goal_.goal);

    goal_.use_start = false;
    getInput("use_start", use_start);
    if (use_start) {
      getInput("start", goal_.start);
      goal_.use_start = true;
    }
  } else {
    getInput("start_id", goal_.start_id);
    getInput("goal_id", goal_.goal_id);
    goal_.use_start = false;
    goal_.use_poses = false;
  }
}

BT::NodeStatus ComputeRouteAction::on_success()
{
  setOutput("path", result_.result->path);
  setOutput("route", result_.result->route);
  setOutput("planning_time", result_.result->planning_time);
  // Set empty error code, action was successful
  setOutput("error_code_id", ActionResult::NONE);
  return BT::NodeStatus::SUCCESS;
}

void ComputeRouteAction::resetPorts()
{
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  nav2_msgs::msg::Route empty_route;
  setOutput("route", empty_route);
  setOutput("planning_time", builtin_interfaces::msg::Duration());
}

BT::NodeStatus ComputeRouteAction::on_aborted()
{
  resetPorts();
  setOutput("error_code_id", result_.result->error_code);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ComputeRouteAction::on_cancelled()
{
  resetPorts();
  // Set empty error code, action was cancelled
  setOutput("error_code_id", ActionResult::NONE);
  return BT::NodeStatus::SUCCESS;
}

void ComputeRouteAction::halt()
{
  resetPorts();
  // DO NOT reset "error_code_id" output port, we want to read it later
  // DO NOT reset "error_msg" output port, we want to read it later
  BtActionNode::halt();
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::ComputeRouteAction>(
        name, "compute_route", config);
    };

  factory.registerBuilder<nav2_behavior_tree::ComputeRouteAction>(
    "ComputeRoute", builder);
}
