// Copyright 2020 Anshumaan Singh
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <vector>
#include <memory>
#include <string>
#include "lazy_theta_star_b/planner.h"

namespace planner
{
void planner::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmapRos)
{
  planner_ = std::make_unique<lazyThetaStarB::LazyThetaStarB>();
  node_ = parent;
  name_ = name;
  tf_ = tf;
  planner_->costmap_ = costmapRos->getCostmap();
  costmap_ = costmapRos->getCostmap();
  globalFrame_ = costmapRos->getGlobalFrameID();

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".howManyCorners", rclcpp::ParameterValue(
      8));

  node_->get_parameter(name_ + ".howManyCorners", how_many_corners_);

  if (how_many_corners_ != 8) {
    how_many_corners_ = 4;
  }

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_dist_", rclcpp::ParameterValue(
      0.1));

  node_->get_parameter(name_ + ".interpolation_dist_", interpolation_dist_);
}

void planner::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type planner",
    name_.c_str());
}

void planner::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type planner",
    name_.c_str());
}

void planner::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type planner",
    name_.c_str());
}

nav_msgs::msg::Path planner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;

  RCLCPP_INFO(node_->get_logger(), "STARTED!!!");

  planner_->costmap_->worldToMap(start.pose.position.x, start.pose.position.y,
    reinterpret_cast<unsigned int &>(planner_->src.x),
    reinterpret_cast<unsigned int &>(planner_->src.y));

  planner_->costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y,
    reinterpret_cast<unsigned int &>(planner_->dst.x),
    reinterpret_cast<unsigned int &>(planner_->dst.y));

  planner_->node_ = node_;
//  planner_->costmap_ = costmap_;
  planner_->how_many_corners = how_many_corners_;
  RCLCPP_INFO(node_->get_logger(), "STOPPED APPARENTLY!!!");
  planner_->makePlan(x, y);
  RCLCPP_INFO(node_->get_logger(), "NOT REALLY THOUGH!!!");

  global_path = linearInterpolation(interpolation_dist_);
  x.clear();
  y.clear();
  geometry_msgs::msg::PoseStamped pEnd;
  pEnd.pose.position.x = goal.pose.position.x;
  pEnd.pose.position.y = goal.pose.position.y;
  global_path.poses.push_back(pEnd);

  global_path.header.stamp = node_->now();
  global_path.header.frame_id = globalFrame_;

  return global_path;
}

nav_msgs::msg::Path planner::linearInterpolation(double distBwPoints)
{
  nav_msgs::msg::Path pa;
  pa.header.frame_id = "map";
  for (int i = 0; i < static_cast<int>(x.size() - 1); i++) {
    geometry_msgs::msg::PoseStamped p;

    pts<float> ptCopy = {x[i], y[i]};
    p.pose.position.x = ptCopy.x;
    p.pose.position.y = ptCopy.y;
    pa.poses.push_back(p);

    pts<float> pt2Copy = {x[i + 1], y[i + 1]};

    double distance = std::hypot(pt2Copy.x - ptCopy.x, pt2Copy.y - ptCopy.y);
    int loops = static_cast<int>(distance / distBwPoints);
    double sinAlpha = (pt2Copy.y - ptCopy.y) / distance;
    double cosAlpha = (pt2Copy.x - ptCopy.x) / distance;

    for (int j = 1; j < loops; j++) {
      p.pose.position.x = ptCopy.x + j * distBwPoints * cosAlpha;
      p.pose.position.y = ptCopy.y + j * distBwPoints * sinAlpha;
      pa.poses.push_back(p);
    }

    p.pose.position.x = pt2Copy.x;
    p.pose.position.y = pt2Copy.y;
    pa.poses.push_back(p);
  }

  return pa;
}

}  // namespace planner
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(planner::planner, nav2_core::GlobalPlanner)
