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
#include "tf2/utils.h"
#include "angles/angles.h"

#include "nav2_behavior_tree/plugins/action/remove_passed_goals_action.hpp"

namespace nav2_behavior_tree
{

RemovePassedGoals::RemovePassedGoals(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf),
  viapoint_achieved_radius_(0.5),
  viapoint_achieved_yaw_(1.57)
{}

void RemovePassedGoals::initialize()
{
  getInput("radius", viapoint_achieved_radius_);
  getInput("yaw", viapoint_achieved_yaw_);  // new input for yaw threshold

  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  node_ = config().blackboard->get<nav2::LifecycleNode::SharedPtr>("node");
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
  getInput("nb_goals_to_consider", nb_goals_to_consider_);
  nb_goals_to_consider_ = std::min(nb_goals_to_consider_,
      static_cast<int>(goal_poses.goals.size()));

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

  bool goal_reached = false;
  int reached_goal_index = -1;

  // Iterate over the first `nb_goals_to_consider` goals
  for (int i = 0; i < nb_goals_to_consider_; ++i) {
    double dist_to_goal = euclidean_distance(goal_poses.goals[i].pose, current_pose.pose);
    double yaw_goal = tf2::getYaw(goal_poses.goals[i].pose.orientation);
    double yaw_current = tf2::getYaw(current_pose.pose.orientation);
    double yaw_diff = std::fabs(angles::shortest_angular_distance(yaw_current, yaw_goal));
    if ((dist_to_goal <= viapoint_achieved_radius_ && yaw_diff <= viapoint_achieved_yaw_) 
    || dist_to_goal <= 0.2) // Added a small tolerance without yaw check to avoid robot stucked near collision (allow to achieve pose without orientation)
    {
      reached_goal_index = i;
      goal_reached = true;
      break;
    }
  }
  if (goal_reached) {
    if (waypoint_statuses_get_res) {
      for (int i = 0; i <= reached_goal_index; ++i) {
        auto cur_waypoint_index =
          find_next_matching_goal_in_waypoint_statuses(waypoint_statuses, goal_poses.goals[i]);
        if (cur_waypoint_index >= 0 &&
            cur_waypoint_index < static_cast<int>(waypoint_statuses.size()))
        {
          waypoint_statuses.at(cur_waypoint_index).waypoint_status =
            (i == reached_goal_index) ? nav2_msgs::msg::WaypointStatus::COMPLETED :
                                        nav2_msgs::msg::WaypointStatus::SKIPPED;
        } else {
          RCLCPP_WARN(node_->get_logger(),
            "RemovePassedGoals: No matching waypoint found for goal %d", i);
        }
      }
    }
    // If the reached goal is NOT the last one, erase all COMPLETED and SKIPPED goals
    if (reached_goal_index < static_cast<int>(goal_poses.goals.size()) - 1) {
      goal_poses.goals.erase(goal_poses.goals.begin(),
          goal_poses.goals.begin() + reached_goal_index + 1);
    }
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
