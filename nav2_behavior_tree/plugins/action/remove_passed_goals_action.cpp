// Copyright (c) 2021 Samsung Research America
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

#include "nav_msgs/msg/path.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "nav2_behavior_tree/plugins/action/remove_passed_goals_action.hpp"

namespace nav2_behavior_tree
{

RemovePassedGoals::RemovePassedGoals(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf),
  viapoint_achieved_radius_(0.5)
{}

void RemovePassedGoals::initialize()
{
  getInput("radius", viapoint_achieved_radius_);

  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  node_->get_parameter("transform_tolerance", transform_tolerance_);

  robot_base_frame_ = BT::deconflictPortAndParamFrame<std::string>(
    node_, "robot_base_frame", this);
}

inline BT::NodeStatus RemovePassedGoals::tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }

  nav_msgs::msg::Goals goal_poses;
  getInput("input_goals", goal_poses);

  if (goal_poses.goals.empty()) {
    setOutput("output_goals", goal_poses);
    return BT::NodeStatus::SUCCESS;
  }

  using namespace nav2_util::geometry_utils;  // NOLINT

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, goal_poses.goals[0].header.frame_id, robot_base_frame_,
      transform_tolerance_))
  {
    return BT::NodeStatus::FAILURE;
  }

  // get the `waypoint_statuses` vector
  std::vector<nav2_msgs::msg::WaypointStatus> waypoint_statuses;
  auto waypoint_statuses_get_res = getInput("input_waypoint_statuses", waypoint_statuses);
  if (!waypoint_statuses_get_res) {
    RCLCPP_ERROR_ONCE(node_->get_logger(), "Missing [input_waypoint_statuses] port input!");
  }

  double dist_to_goal;
  while (goal_poses.goals.size() > 1) {
    dist_to_goal = euclidean_distance(goal_poses.goals[0].pose, current_pose.pose);

    if (dist_to_goal > viapoint_achieved_radius_) {
      break;
    }

    // mark waypoint statuses before the goal is erased from goals
    if (waypoint_statuses_get_res) {
      auto cur_waypoint_index =
        find_next_matching_goal_in_waypoint_statuses(waypoint_statuses, goal_poses.goals[0]);
      if (cur_waypoint_index == -1) {
        RCLCPP_ERROR_ONCE(node_->get_logger(), "Failed to find matching goal in waypoint_statuses");
        return BT::NodeStatus::FAILURE;
      }
      waypoint_statuses[cur_waypoint_index].waypoint_status =
        nav2_msgs::msg::WaypointStatus::COMPLETED;
    }

    goal_poses.goals.erase(goal_poses.goals.begin());
  }

  setOutput("output_goals", goal_poses);
  // set `waypoint_statuses` output
  setOutput("output_waypoint_statuses", waypoint_statuses);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RemovePassedGoals>("RemovePassedGoals");
}
