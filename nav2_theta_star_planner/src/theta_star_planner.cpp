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
#include "nav2_theta_star_planner/theta_star_planner.hpp"
#include "nav2_theta_star_planner/theta_star.hpp"

namespace nav2_theta_star_planner
{
void ThetaStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  planner_ = std::make_unique<theta_star::ThetaStar>();
  auto node = parent.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  name_ = name;
  tf_ = tf;
  planner_->costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".how_many_corners", rclcpp::ParameterValue(8));

  node->get_parameter(name_ + ".how_many_corners", planner_->how_many_corners_);

  if (planner_->how_many_corners_ != 8 && planner_->how_many_corners_ != 4) {
    planner_->how_many_corners_ = 8;
    RCLCPP_WARN(logger_, "Your value for - .how_many_corners  was overridden, and is now set to 8");
  }

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".w_euc_cost", rclcpp::ParameterValue(4.0));
  node->get_parameter(name_ + ".w_euc_cost", planner_->w_euc_cost_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".w_traversal_cost", rclcpp::ParameterValue(7.0));
  node->get_parameter(name_ + ".w_traversal_cost", planner_->w_traversal_cost_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".w_heuristic_cost", rclcpp::ParameterValue(1.0));
  node->get_parameter(name_ + ".w_heuristic_cost", planner_->w_heuristic_cost_);
}

void ThetaStarPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "CleaningUp plugin %s of type nav2_theta_star_planner", name_.c_str());
}

void ThetaStarPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating plugin %s of type nav2_theta_star_planner", name_.c_str());
}

void ThetaStarPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating plugin %s of type nav2_theta_star_planner", name_.c_str());
}

nav_msgs::msg::Path ThetaStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;
  auto start_time = std::chrono::steady_clock::now();
  planner_->setStartAndGoal(start, goal);
  RCLCPP_DEBUG(
    logger_, "Got the src and dst... (%i, %i) && (%i, %i)",
    planner_->src_.x, planner_->src_.y, planner_->dst_.x, planner_->dst_.y);
  getPlan(global_path);
  auto stop_time = std::chrono::steady_clock::now();
  auto dur = std::chrono::duration_cast<std::chrono::microseconds>(stop_time - start_time);
  RCLCPP_DEBUG(logger_, "the time taken is : %i", static_cast<int>(dur.count()));
  return global_path;
}

void ThetaStarPlanner::getPlan(nav_msgs::msg::Path & global_path)
{
  std::vector<coordsW> path;

  if (planner_->isSafeToPlan()) {
    RCLCPP_ERROR(logger_, "Either of the start or goal pose are an obstacle! ");
    coordsW world{};
    planner_->costmap_->mapToWorld(planner_->src_.x, planner_->src_.y, world.x, world.y);
    path.push_back({world.x, world.y});
    planner_->costmap_->mapToWorld(planner_->dst_.x, planner_->dst_.y, world.x, world.y);
    path.push_back({world.x, world.y});
    global_path = linearInterpolation(path, planner_->costmap_->getResolution());
    global_path.poses.clear();
  } else if (planner_->generatePath(path)) {
    global_path = linearInterpolation(path, planner_->costmap_->getResolution());
  } else {
    RCLCPP_ERROR(logger_, "Could not generate path between the given poses");
    global_path.poses.clear();
  }
  path.clear();
  global_path.header.stamp = clock_->now();
  global_path.header.frame_id = global_frame_;
}

nav_msgs::msg::Path ThetaStarPlanner::linearInterpolation(
  const std::vector<coordsW> & raw_path,
  const double & dist_bw_points)
{
  nav_msgs::msg::Path pa;

  for (unsigned int j = 0; j < raw_path.size() - 1; j++) {
    geometry_msgs::msg::PoseStamped p;
    coordsW pt1 = {raw_path[j].x, raw_path[j].y};
    p.pose.position.x = pt1.x;
    p.pose.position.y = pt1.y;
    pa.poses.push_back(p);

    coordsW pt2 = {raw_path[j + 1].x, raw_path[j + 1].y};
    geometry_msgs::msg::PoseStamped p1;
    double distance = std::hypot(pt2.x - pt1.x, pt2.y - pt1.y);
    int loops = static_cast<int>(distance / dist_bw_points);
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


}  // namespace nav2_theta_star_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_theta_star_planner::ThetaStarPlanner, nav2_core::GlobalPlanner)
