// Copyright (c) 2026 Yong Ling
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

#include "nav2_behavior_tree/plugins/action/track_precomputed_route_action.hpp"

namespace nav2_behavior_tree
{

TrackPrecomputedRouteAction::TrackPrecomputedRouteAction(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<Action>(xml_tag_name, action_name, conf)
{}

void TrackPrecomputedRouteAction::on_tick()
{
  if (!getInput("route", goal_.route)) {
    throw BT::RuntimeError("TrackPrecomputedRoute requires a route");
  }
  resetFeedback();
  setOutput("blocked_ids", std::vector<uint32_t>());
}

void TrackPrecomputedRouteAction::resetFeedback()
{
  setOutput("last_node_id", uint16_t{0});
  setOutput("next_node_id", uint16_t{0});
  setOutput("current_edge_id", uint16_t{0});
  setOutput("route_feedback", nav2_msgs::msg::Route());
  setOutput("path", nav_msgs::msg::Path());
  setOutput("operations_triggered", std::vector<std::string>());
}

BT::NodeStatus TrackPrecomputedRouteAction::on_success()
{
  resetFeedback();
  setOutput("execution_duration", result_.result->execution_duration);
  setOutput("blocked_ids", result_.result->blocked_ids);
  setOutput("error_code_id", Action::Result::NONE);
  setOutput("error_msg", "");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus TrackPrecomputedRouteAction::on_aborted()
{
  resetFeedback();
  setOutput("execution_duration", result_.result->execution_duration);
  setOutput("blocked_ids", result_.result->blocked_ids);
  setOutput("error_code_id", result_.result->error_code);
  setOutput("error_msg", result_.result->error_msg);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus TrackPrecomputedRouteAction::on_cancelled()
{
  resetFeedback();
  setOutput("execution_duration", builtin_interfaces::msg::Duration());
  setOutput("blocked_ids", std::vector<uint32_t>());
  setOutput("error_code_id", Action::Result::NONE);
  setOutput("error_msg", "");
  return BT::NodeStatus::SUCCESS;
}

void TrackPrecomputedRouteAction::on_wait_for_result(
  std::shared_ptr<const Action::Feedback> feedback)
{
  Action::Goal updated;
  if (!getInput("route", updated.route)) {
    throw BT::RuntimeError("TrackPrecomputedRoute requires a route");
  }
  if (updated != goal_) {
    goal_ = updated;
    goal_updated_ = true;
    resetFeedback();
    setOutput("blocked_ids", std::vector<uint32_t>());
  } else if (feedback) {
    setOutput("last_node_id", feedback->last_node_id);
    setOutput("next_node_id", feedback->next_node_id);
    setOutput("current_edge_id", feedback->current_edge_id);
    setOutput("route_feedback", feedback->route);
    setOutput("path", feedback->path);
    setOutput("operations_triggered", feedback->operations_triggered);
  }
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<nav2_behavior_tree::TrackPrecomputedRouteAction>(
        name, "track_precomputed_route", config);
    };
  factory.registerBuilder<nav2_behavior_tree::TrackPrecomputedRouteAction>(
    "TrackPrecomputedRoute", builder);
}
