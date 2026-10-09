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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__TRACK_PRECOMPUTED_ROUTE_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__TRACK_PRECOMPUTED_ROUTE_ACTION_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_msgs/action/track_precomputed_route.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"

namespace nav2_behavior_tree
{

/** @brief Track an externally planned route, specified by a Route message. */
class TrackPrecomputedRouteAction : public BtActionNode<nav2_msgs::action::TrackPrecomputedRoute>
{
  using Action = nav2_msgs::action::TrackPrecomputedRoute;

public:
  TrackPrecomputedRouteAction(
    const std::string & xml_tag_name, const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;
  void on_wait_for_result(std::shared_ptr<const Action::Feedback> feedback) override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
        BT::InputPort<nav2_msgs::msg::Route>("route",
          "Ordered nodes and edges in the loaded graph"),
        BT::OutputPort<builtin_interfaces::msg::Duration>("execution_duration",
          "Tracking duration"),
        BT::OutputPort<std::vector<uint32_t>>("blocked_ids", "IDs reported by route operations"),
        BT::OutputPort<uint16_t>("last_node_id", "Previous node ID"),
        BT::OutputPort<uint16_t>("next_node_id", "Next node ID"),
        BT::OutputPort<uint16_t>("current_edge_id", "Current edge ID"),
        BT::OutputPort<nav2_msgs::msg::Route>("route_feedback", "Canonical route"),
        BT::OutputPort<nav_msgs::msg::Path>("path", "Densified route path"),
        BT::OutputPort<std::vector<std::string>>("operations_triggered", "Triggered operations")
    });
  }

private:
  void resetFeedback();
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__TRACK_PRECOMPUTED_ROUTE_ACTION_HPP_
