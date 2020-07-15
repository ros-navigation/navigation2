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
#include "lazy_theta_star_planner/lazy_theta_star_planner.h"
#include "lazy_theta_star_planner/lazy_theta_star2.h"


namespace lazyThetaStarPlanner
{
void LazyThetaStarPlanner::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmapRos)
{
  planner_ = std::make_unique<lazyThetaStar::LazyThetaStar>();
  node_ = parent;
  name_ = name;
  planner_->costmap_ = costmapRos->getCostmap();
  globalFrame_ = costmapRos->getGlobalFrameID();

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".how_many_corners", rclcpp::ParameterValue(
      8));

  node_->get_parameter(name_ + ".how_many_corners", how_many_corners_);

  if (how_many_corners_ != 8) {
    how_many_corners_ = 4;
  }

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_dist", rclcpp::ParameterValue(
      0.1));

  node_->get_parameter(name_ + ".interpolation_dist", interpolation_dist_);

}

void LazyThetaStarPlanner::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type lazyThetaStarPlanner",
    name_.c_str());
}

void LazyThetaStarPlanner::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type lazyThetaStarPlanner",
    name_.c_str());
}

void LazyThetaStarPlanner::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type lazyThetaStarPlanner",
    name_.c_str());
}

nav_msgs::msg::Path LazyThetaStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;
  std::vector<coords<world_pts>> path;

  planner_->costmap_->worldToMap(start.pose.position.x, start.pose.position.y, src_[0], src_[1]);
  planner_->src = {static_cast<int>(src_[0]), static_cast<int>(src_[1])};

  planner_->costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, dst_[0], dst_[1]);
  planner_->dst = {static_cast<int>(dst_[0]), static_cast<int>(dst_[1])};

  RCLCPP_INFO(
    node_->get_logger(), "Got the src and dst... (%i, %i) && (%i, %i)", planner_->src.x, planner_->src.y,
    planner_->dst.x, planner_->dst.y);

  planner_->node_ = node_;
  planner_->how_many_corners = how_many_corners_;
  planner_->lethal_cost = LETHAL_COST;

  planner_->initializeStuff();
  planner_->getPath(path);
  planner_->clearStuff();

  if (path.size() > 1) {
    global_path = linearInterpolation(path, 0.1);
  } else {
    global_path.poses.clear();
  }
  path.clear();
  std::cout << global_path.poses.size() << '\n';
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = globalFrame_;
  RCLCPP_INFO(node_->get_logger(), "SENT OUT THE PATH");
  return global_path;
}

nav_msgs::msg::Path LazyThetaStarPlanner::linearInterpolation(
  std::vector<coords<world_pts> > & raw_path,
  double dist_bw_points)
{
  nav_msgs::msg::Path pa;

  for (int j = 0; j < raw_path.size() - 1; j++) {
    geometry_msgs::msg::PoseStamped p;
    coords<world_pts> pt1 = {raw_path[j].x, raw_path[j].y};
    p.pose.position.x = pt1.x;
    p.pose.position.y = pt1.y;
    pa.poses.push_back(p);

    coords<world_pts> pt2 = {raw_path[j + 1].x, raw_path[j + 1].y};
    geometry_msgs::msg::PoseStamped p1;
    double distance = std::hypot(pt2.x - pt1.x, pt2.y - pt1.y);
    int loops = distance / dist_bw_points;
    double sin_alpha = (pt2.y - pt1.y) / distance;
    double cos_alpha = (pt2.x - pt1.x) / distance;
    for (int k = 1; k < loops; k++) {
      p1.pose.position.x = pt1.x + k * dist_bw_points * cos_alpha;
      p1.pose.position.y = pt1.y + k * dist_bw_points * sin_alpha;
      pa.poses.push_back(p1);
    }
  }
  return pa;
}

}  // namespace lazyThetaStarPlanner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(lazyThetaStarPlanner::LazyThetaStarPlanner, nav2_core::GlobalPlanner)
