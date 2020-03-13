// Copyright (c) 2020, Samsung Research America
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
// limitations under the License. Reserved.


// tolerenace : in search check heurici if close enough to call it done
//    keep going, but if fails, use that closest
//    backing out from intended goal if bad
// smoothing

// benefits list:
//  - for tolerance, only search once
//  - we have inflation + dynamic processing: cached gradiant map not used
//  - not searching then backtracing with grad descent for 2x go through
//  - lower memory (?) and faster (?)
//  - modern data structures

#include <string>
#include <memory>
#include <vector>
#include "smac_planner/smac_planner.hpp"

namespace smac_planner
{

SmacPlanner::SmacPlanner()
: a_star_(nullptr),
  tf_(nullptr),
  node_(nullptr),
  costmap_(nullptr)
{
}

SmacPlanner::~SmacPlanner()
{
  RCLCPP_INFO(
    node_->get_logger(), "Destroying plugin %s of type SmacPlanner",
    name_.c_str());
}

void SmacPlanner::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  name_ = name;
  global_frame_ = costmap_ros->getGlobalFrameID();

  RCLCPP_INFO(
    node_->get_logger(), "Configuring plugin %s of type SmacPlanner",
    name_.c_str());

  bool allow_unknown /*, tolerance*/;
  int max_iterations;
  float travel_cost;
  std::string neighborhood_for_search;
  // declare_parameter_if_not_declared(
  //   node_, name + ".tolerance", rclcpp::ParameterValue(2.0));
  // node_->get_parameter(name + ".tolerance", tolerance);
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".allow_unknown", rclcpp::ParameterValue(true));
  node_->get_parameter(name + ".allow_unknown", allow_unknown);
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".max_iterations", rclcpp::ParameterValue(1000000));
  node_->get_parameter(name + ".max_iterations", max_iterations);
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".travel_cost", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name + ".travel_cost", travel_cost);

  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".neighborhood_for_search", rclcpp::ParameterValue(std::string("MOORE")));
  node_->get_parameter(name + ".neighborhood_for_search", neighborhood_for_search);
  Neighborhood neighborhood;
  if (neighborhood_for_search == std::string("MOORE")) {
    neighborhood = Neighborhood::MOORE;
  } else if (neighborhood_for_search == std::string("VAN_NEUMANN")) {
    neighborhood = Neighborhood::VAN_NEUMANN;
  } else {
    neighborhood = Neighborhood::MOORE;
    RCLCPP_WARN(node_->get_logger(),
      "Unable to get Neighborhood search type. Given %s, "
      "valid options are MOORE and VAN_NEUMANN. Using MOORE as default",
      neighborhood_for_search.c_str());
  }

  a_star_ = std::make_unique<AStarAlgorithm>(neighborhood);
  a_star_->initialize(travel_cost, allow_unknown, static_cast<unsigned int>(max_iterations));
}

void SmacPlanner::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type SmacPlanner",
    name_.c_str());
}

void SmacPlanner::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type SmacPlanner",
    name_.c_str());
}

void SmacPlanner::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "Cleaning up plugin %s of type SmacPlanner",
    name_.c_str());
  a_star_.reset();
}

nav_msgs::msg::Path SmacPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  a_star_->setCosts(
    costmap_->getSizeInCellsX(),
    costmap_->getSizeInCellsY(),
    costmap_->getCharMap());

  unsigned int mx, my, index;
  costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx, my);
  index = costmap_->getIndex(mx, my);
  a_star_->setStart(index);

  costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my);
  index = costmap_->getIndex(mx, my);
  a_star_->setGoal(index);

  nav_msgs::msg::Path plan;
  plan.header.stamp = node_->now();
  plan.header.frame_id = global_frame_;
  geometry_msgs::msg::PoseStamped pose;
  pose.header = plan.header;

  IndexPath path;
  try {
    if (!a_star_->createPath(path)) {
      RCLCPP_WARN(
        node_->get_logger(),
        "%s: failed to create plan, exceeded maximum iterations.",
        name_.c_str());
      return plan;
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      node_->get_logger(),
      "%s: failed to create plan, invalid use: %s", name_.c_str(), e.what());
    return plan;
  }

  for (int i = path.size() - 1; i >= 0; --i) {
    double world_x, world_y;
    unsigned int index_x, index_y;
    costmap_->indexToCells(path[i], index_x, index_y);
    costmap_->mapToWorld(index_x, index_y, world_x, world_y);

    pose.pose.position.x = world_x;
    pose.pose.position.y = world_y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    plan.poses.push_back(pose);
  }

  return plan;
}

}  // namespace smac_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(smac_planner::SmacPlanner, nav2_core::GlobalPlanner)
