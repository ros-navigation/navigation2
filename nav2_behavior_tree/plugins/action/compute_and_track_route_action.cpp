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

#include "nav2_behavior_tree/plugins/action/compute_and_track_route_action.hpp"

namespace nav2_behavior_tree
{

ComputeAndTrackRouteAction::ComputeAndTrackRouteAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<Action>(xml_tag_name, action_name, conf)
{
  nav_msgs::msg::Path empty_path;
  nav2_msgs::msg::Route empty_route;
  feedback_.last_node_id = 0;
  feedback_.next_node_id = 0;
  feedback_.current_edge_id = 0;
  feedback_.route = empty_route;
  feedback_.path = empty_path;
  feedback_.rerouted = false;
}

void ComputeAndTrackRouteAction::on_tick()
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

BT::NodeStatus ComputeAndTrackRouteAction::on_success()
{
  resetFeedbackAndOutputPorts();
  setOutput("execution_duration", result_.result->execution_duration);
  setOutput("error_code_id", ActionResult::NONE);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ComputeAndTrackRouteAction::on_aborted()
{
  resetFeedbackAndOutputPorts();
  setOutput("execution_duration", builtin_interfaces::msg::Duration());
  setOutput("error_code_id", result_.result->error_code);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ComputeAndTrackRouteAction::on_cancelled()
{
  resetFeedbackAndOutputPorts();
  // Set empty error code, action was cancelled
  setOutput("execution_duration", builtin_interfaces::msg::Duration());
  setOutput("error_code_id", ActionResult::NONE);
  return BT::NodeStatus::SUCCESS;
}

void ComputeAndTrackRouteAction::on_wait_for_result(
  std::shared_ptr<const Action::Feedback> feedback)
{
  // Check for request updates to the goal
  bool use_poses = false, use_start = false;
  getInput("use_start", use_start);
  getInput("use_poses", use_poses);

  if (goal_.use_poses != use_poses) {
    goal_updated_ = true;
  }

  if (use_poses) {
    geometry_msgs::msg::PoseStamped goal;
    getInput("goal", goal);
    if (goal_.goal != goal) {
      goal_updated_ = true;
    }

    if (goal_.use_start != use_start) {
      goal_updated_ = true;
    }
    if (use_start) {
      geometry_msgs::msg::PoseStamped start;
      getInput("start", start);
      if (goal_.start != start) {
        goal_updated_ = true;
      }
    }
  } else {
    // Check if the start and goal IDs have changed
    unsigned int start_id = 0;
    unsigned int goal_id = 0;
    getInput("start_id", start_id);
    getInput("goal_id", goal_id);
    if (goal_.start_id != start_id) {
      goal_updated_ = true;
    }
    if (goal_.goal_id != goal_id) {
      goal_updated_ = true;
    }
  }

  // If we're updating the request, we need to fully update the goal
  // Easier to call on_tick() again than to duplicate the code
  if (goal_updated_) {
    on_tick();
  }

  if (feedback) {
    feedback_ = *feedback;
    setOutput("last_node_id", feedback_.last_node_id);
    setOutput("next_node_id", feedback_.next_node_id);
    setOutput("current_edge_id", feedback_.current_edge_id);
    setOutput("route", feedback_.route);
    setOutput("path", feedback_.path);
    setOutput("rerouted", feedback_.rerouted);
  }
}

void ComputeAndTrackRouteAction::resetFeedbackAndOutputPorts()
{
  nav_msgs::msg::Path empty_path;
  nav2_msgs::msg::Route empty_route;
  feedback_.last_node_id = 0;
  feedback_.next_node_id = 0;
  feedback_.current_edge_id = 0;
  feedback_.route = empty_route;
  feedback_.path = empty_path;
  feedback_.rerouted = false;
  setOutput("last_node_id", feedback_.last_node_id);
  setOutput("next_node_id", feedback_.next_node_id);
  setOutput("current_edge_id", feedback_.current_edge_id);
  setOutput("route", feedback_.route);
  setOutput("path", feedback_.path);
  setOutput("rerouted", feedback_.rerouted);
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::ComputeAndTrackRouteAction>(
        name, "compute_and_track_route", config);
    };

  factory.registerBuilder<nav2_behavior_tree::ComputeAndTrackRouteAction>(
    "ComputeAndTrackRoute", builder);
}
