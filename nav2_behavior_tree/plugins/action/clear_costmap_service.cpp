// Copyright (c) 2019 Samsung Research America
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

#include "nav2_behavior_tree/plugins/action/clear_costmap_service.hpp"

namespace nav2_behavior_tree
{

ClearEntireCostmapService::ClearEntireCostmapService(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
: BtServiceNode<nav2_msgs::srv::ClearEntireCostmap>(service_node_name, conf)
{
}

void ClearEntireCostmapService::on_tick()
{
  increment_recovery_count();
}

ClearCostmapExceptRegionService::ClearCostmapExceptRegionService(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
: BtServiceNode<nav2_msgs::srv::ClearCostmapExceptRegion>(service_node_name, conf)
{
}

void ClearCostmapExceptRegionService::on_tick()
{
  getInput("reset_distance", request_->reset_distance);
  increment_recovery_count();
}

ClearCostmapAroundRobotService::ClearCostmapAroundRobotService(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
: BtServiceNode<nav2_msgs::srv::ClearCostmapAroundRobot>(service_node_name, conf)
{
}

void ClearCostmapAroundRobotService::on_tick()
{
  getInput("reset_distance", request_->reset_distance);
  increment_recovery_count();
}

ClearCostmapAroundPoseService::ClearCostmapAroundPoseService(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
: BtServiceNode<nav2_msgs::srv::ClearCostmapAroundPose>(service_node_name, conf)
{
}

void ClearCostmapAroundPoseService::on_tick()
{
  getInput("pose", request_->pose);
  getInput("reset_distance", request_->reset_distance);
  increment_recovery_count();
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::ClearEntireCostmapService>("ClearEntireCostmap");
  factory.registerNodeType<nav2_behavior_tree::ClearCostmapExceptRegionService>(
    "ClearCostmapExceptRegion");
  factory.registerNodeType<nav2_behavior_tree::ClearCostmapAroundRobotService>(
    "ClearCostmapAroundRobot");
  factory.registerNodeType<nav2_behavior_tree::ClearCostmapAroundPoseService>(
    "ClearCostmapAroundPose");
}
