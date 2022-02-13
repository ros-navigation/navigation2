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

#include "nav2_behavior_tree/plugins/action/remove_passed_goals_with_multiple_planners_action.hpp"

namespace nav2_behavior_tree
{

RemovePassedGoalsWithMultiplePlanners::RemovePassedGoalsWithMultiplePlanners(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf),
  viapoint_achieved_radius_(0.5)
{
  getInput("radius", viapoint_achieved_radius_);

  getInput("global_frame", global_frame_);
  getInput("robot_base_frame", robot_base_frame_);
  
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  node->get_parameter("transform_tolerance", transform_tolerance_);
}

inline BT::NodeStatus RemovePassedGoalsWithMultiplePlanners::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  auto prev_planner_ids = planner_ids_;
  getInput("input_planner_ids", planner_ids_);

  // If input planners changed(e.g by another BT node), then we should populate remaining list of planners again.
  if (prev_planner_ids != planner_ids_){
    remaining_planner_ids_ = planner_ids_;
  }  

  
  Goals goal_poses;
  getInput("input_goals", goal_poses);

  if (goal_poses.empty()) {
    setOutput("output_goals", goal_poses);
    return BT::NodeStatus::SUCCESS;
  }
  if (remaining_planner_ids_.size() != goal_poses.size() && planner_ids_.size()  != 1 && planner_ids_.size() != 0){
    RCLCPP_WARN(
      config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
      "RemovePassedGoalsWithMultiplePlanners requested with a planner_ids array length mismatch. It should be either 0 - use default, \
        1 - use one for all, or length should be equal to the number of goals. Current Planner Ids len %d, goal len %d", planner_ids_.size(), goal_poses.size());
    return BT::NodeStatus::FAILURE;
  }

  using namespace nav2_util::geometry_utils;  // NOLINT

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    return BT::NodeStatus::FAILURE;
  }

  double dist_to_goal;
  while (goal_poses.size() > 1) {
    dist_to_goal = euclidean_distance(goal_poses[0].pose, current_pose.pose);

    if (dist_to_goal > viapoint_achieved_radius_) {
      break;
    }

    if (remaining_planner_ids_.size() > 0){
      remaining_planner_ids_.erase(remaining_planner_ids_.begin());
    }
    goal_poses.erase(goal_poses.begin());
  }

  std::string planner_ids_str = "";
  for (auto i=0; i < remaining_planner_ids_.size(); i++){
    planner_ids_str += remaining_planner_ids_[i];
    if (i != remaining_planner_ids_.size()-1){
      planner_ids_str += ";";
    }
  }

  setOutput("output_goals", goal_poses);
  setOutput("output_planner_ids", planner_ids_str);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RemovePassedGoalsWithMultiplePlanners>("RemovePassedGoalsWithMultiplePlanners");
}
