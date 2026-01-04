// Copyright (c) 2024 Angsa Robotics
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

#include "nav2_behavior_tree/plugins/action/remove_in_collision_goals_action.hpp"
#include "nav2_behavior_tree/bt_utils.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_util/geometry_utils.hpp"

namespace nav2_behavior_tree
{

RemoveInCollisionGoals::RemoveInCollisionGoals(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
: BtServiceNode<nav2_msgs::srv::GetCosts>(service_node_name, conf,
    "/global_costmap/get_cost_global_costmap")
{}


void RemoveInCollisionGoals::on_tick()
{
  getInput("use_footprint", use_footprint_);
  getInput("cost_threshold", cost_threshold_);
  getInput("input_goals", input_goals_);
  getInput("consider_unknown_as_obstacle", consider_unknown_as_obstacle_);
  getInput("nb_goals_to_consider", nb_goals_to_consider_);

  if (input_goals_.goals.empty()) {
    setOutput("output_goals", input_goals_);
    should_send_request_ = false;
    return;
  }
  request_ = std::make_shared<nav2_msgs::srv::GetCosts::Request>();
  request_->use_footprint = use_footprint_;

  for (size_t i =
    0; i < static_cast<size_t>(nb_goals_to_consider_) && i < input_goals_.goals.size();
    ++i)
  {
    request_->poses.push_back(input_goals_.goals[i]);
  }
}

BT::NodeStatus RemoveInCollisionGoals::on_completion(
  std::shared_ptr<nav2_msgs::srv::GetCosts::Response> response)
{
  if (!response->success) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "GetCosts service call failed");
    setOutput("output_goals", input_goals_);
    return BT::NodeStatus::FAILURE;
  }

  // get the `waypoint_statuses` vector
  std::vector<nav2_msgs::msg::WaypointStatus> waypoint_statuses;
  auto waypoint_statuses_get_res = getInput("input_waypoint_statuses", waypoint_statuses);

  for (int i = static_cast<int>(response->costs.size()) - 1; i >= 0; --i) {
    if ((response->costs[i] != 255 || consider_unknown_as_obstacle_) &&
      response->costs[i] >= cost_threshold_)
    {
      if (waypoint_statuses_get_res) {
        const int cur_waypoint_index =
          nav2_util::geometry_utils::find_next_matching_goal_in_waypoint_statuses(
            waypoint_statuses, input_goals_.goals[i]);

        if (cur_waypoint_index >= 0 &&
            static_cast<size_t>(cur_waypoint_index) < waypoint_statuses.size()) {
          waypoint_statuses[static_cast<size_t>(cur_waypoint_index)].waypoint_status =
            nav2_msgs::msg::WaypointStatus::SKIPPED;
        } else {
          RCLCPP_WARN(
            node_->get_logger(),
            "RemoveInCollisionGoals: No matching waypoint found for goal index %d; "
            "skipping waypoint_statuses update.", i);
        }
      }
      input_goals_.goals.erase(input_goals_.goals.begin() + i);
    }
  }
  setOutput("output_goals", input_goals_);
  // set `waypoint_statuses` output
  setOutput("output_waypoint_statuses", waypoint_statuses);

  return BT::NodeStatus::SUCCESS;
}

}   // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RemoveInCollisionGoals>("RemoveInCollisionGoals");
}
