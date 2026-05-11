// Copyright (c) 2025 Maurice Alexander Purnawan
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

#include "nav2_behavior_tree/plugins/action/check_pose_occupancy_action.hpp"

namespace nav2_behavior_tree
{

CheckPoseOccupancy::CheckPoseOccupancy(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
: BtServiceNode<nav2_msgs::srv::GetCosts>(service_node_name, conf,
    "/global_costmap/get_cost_global_costmap")
{
}


void CheckPoseOccupancy::on_tick()
{
  getInput<double>("cost_threshold", cost_threshold_);
  getInput<bool>("use_footprint", use_footprint_);
  getInput<bool>("consider_unknown_as_obstacle", consider_unknown_as_obstacle_);
  geometry_msgs::msg::PoseStamped pose;
  getInput("pose", pose);

  request_ = std::make_shared<nav2_msgs::srv::GetCosts::Request>();
  request_->use_footprint = use_footprint_;
  request_->poses.push_back(pose);
}

BT::NodeStatus CheckPoseOccupancy::on_completion(
  std::shared_ptr<nav2_msgs::srv::GetCosts::Response> response)
{
  if (!response->success) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "GetCosts service call failed");
    return BT::NodeStatus::FAILURE;
  }

  if ((response->costs[0] == 255 && !consider_unknown_as_obstacle_) ||
    response->costs[0] < cost_threshold_)
  {
    return BT::NodeStatus::FAILURE;
  } else {
    return BT::NodeStatus::SUCCESS;
  }
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::CheckPoseOccupancy>("CheckPoseOccupancy");
}
