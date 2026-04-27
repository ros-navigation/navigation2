// Copyright (c) 2024 Nav2 Contributors
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

#include <vector>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "nav2_dstar_lite_planner/dstar_lite_planner.hpp"
#include "nav2_dstar_lite_planner/dstar_lite.hpp"

namespace nav2_dstar_lite_planner
{

void DStarLitePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_node_ = parent;
  auto node = parent_node_.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  name_ = name;
  tf_ = tf;
  global_frame_ = costmap_ros->getGlobalFrameID();

  param_handler_ = std::make_unique<ParameterHandler>(node, name_, logger_);
  params_ = param_handler_->getParams();

  planner_ = std::make_unique<DStarLite>(params_);
  planner_->costmap_ = costmap_ros->getCostmap();

  prev_path_cost_ = std::numeric_limits<double>::max();
}

void DStarLitePlanner::cleanup()
{
  RCLCPP_INFO(
    logger_, "CleaningUp plugin %s of type nav2_dstar_lite_planner",
    name_.c_str());
  planner_.reset();
  param_handler_.reset();
  prev_path_.poses.clear();
  prev_path_cost_ = std::numeric_limits<double>::max();
}

void DStarLitePlanner::activate()
{
  RCLCPP_INFO(
    logger_, "Activating plugin %s of type nav2_dstar_lite_planner",
    name_.c_str());
  param_handler_->activate();
}

void DStarLitePlanner::deactivate()
{
  RCLCPP_INFO(
    logger_, "Deactivating plugin %s of type nav2_dstar_lite_planner",
    name_.c_str());
  param_handler_->deactivate();
}

nav_msgs::msg::Path DStarLitePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  std::lock_guard<std::mutex> lock_reinit(param_handler_->getMutex());

  nav_msgs::msg::Path global_path;
  auto start_time = std::chrono::steady_clock::now();

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(
    *(planner_->costmap_->getMutex()));

  unsigned int mx_start, my_start, mx_goal, my_goal;
  if (!planner_->costmap_->worldToMap(
      start.pose.position.x, start.pose.position.y, mx_start, my_start))
  {
    throw nav2_core::PlannerException(
            "Start Coordinates of(" + std::to_string(start.pose.position.x) + ", " +
            std::to_string(start.pose.position.y) + ") was outside bounds");
  }

  if (!planner_->costmap_->worldToMap(
      goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal))
  {
    throw nav2_core::PlannerException(
            "Goal Coordinates of(" + std::to_string(goal.pose.position.x) + ", " +
            std::to_string(goal.pose.position.y) + ") was outside bounds");
  }

  if (planner_->costmap_->getCost(mx_goal, my_goal) ==
    nav2_costmap_2d::LETHAL_OBSTACLE)
  {
    throw nav2_core::PlannerException(
            "Goal Coordinates of(" + std::to_string(goal.pose.position.x) + ", " +
            std::to_string(goal.pose.position.y) + ") was in lethal cost");
  }

  if (mx_start == mx_goal && my_start == my_goal) {
    global_path.header.stamp = clock_->now();
    global_path.header.frame_id = global_frame_;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = global_path.header;
    pose.pose.position.z = 0.0;
    pose.pose = start.pose;

    if (start.pose.orientation != goal.pose.orientation &&
      !params_->use_final_approach_orientation)
    {
      pose.pose.orientation = goal.pose.orientation;
    }
    global_path.poses.push_back(pose);
    return global_path;
  }

  planner_->clearStart();
  planner_->setStartAndGoal(start, goal);

  RCLCPP_DEBUG(
    logger_, "D* Lite: src=(%d, %d) dst=(%d, %d)",
    planner_->src_.x, planner_->src_.y,
    planner_->dst_.x, planner_->dst_.y);

  auto cancel_checker = []() {return false;};
  getPlan(global_path, cancel_checker);

  size_t plan_size = global_path.poses.size();
  if (plan_size > 0) {
    global_path.poses.back().pose.orientation = goal.pose.orientation;
  }

  if (params_->use_final_approach_orientation) {
    if (plan_size == 1) {
      global_path.poses.back().pose.orientation = start.pose.orientation;
    } else if (plan_size > 1) {
      double dx, dy, theta;
      auto last_pose = global_path.poses.back().pose.position;
      auto approach_pose = global_path.poses[plan_size - 2].pose.position;
      dx = last_pose.x - approach_pose.x;
      dy = last_pose.y - approach_pose.y;
      theta = atan2(dy, dx);
      global_path.poses.back().pose.orientation =
        nav2_util::geometry_utils::orientationAroundZAxis(theta);
    }
  }

  auto stop_time = std::chrono::steady_clock::now();
  auto dur = std::chrono::duration_cast<std::chrono::microseconds>(
    stop_time - start_time);
  RCLCPP_DEBUG(
    logger_, "D* Lite planning time: %i us, nodes: %i",
    static_cast<int>(dur.count()), planner_->nodes_opened);

  return global_path;
}

void DStarLitePlanner::getPlan(
  nav_msgs::msg::Path & global_path,
  std::function<bool()> cancel_checker)
{
  std::vector<WorldCoord> raw_path;

  if (planner_->isUnsafeToPlan()) {
    global_path.poses.clear();
    throw nav2_core::PlannerException(
            "Either the start or goal pose is an obstacle!");
  }

  bool found_path = planner_->generatePath(raw_path, cancel_checker);

  if (!found_path || raw_path.empty()) {
    global_path.poses.clear();
    throw nav2_core::PlannerException(
            "Could not generate path between the given poses");
  }

  nav_msgs::msg::Path new_path = linearInterpolation(
    raw_path, planner_->costmap_->getResolution());
  new_path.header.stamp = clock_->now();
  new_path.header.frame_id = global_frame_;

  if (shouldSwitchPath(new_path)) {
    global_path = new_path;
    prev_path_ = new_path;
    prev_path_cost_ = planner_->lastPathCost();
  } else {
    global_path = prev_path_;
    RCLCPP_DEBUG(
      logger_, "D* Lite hysteresis: keeping previous path "
      "(new cost=%.3f, prev cost=%.3f, factor=%.3f)",
      planner_->lastPathCost(), prev_path_cost_,
      params_->hysteresis_factor);
  }
}

bool DStarLitePlanner::shouldSwitchPath(
  const nav_msgs::msg::Path & /*new_path*/) const
{
  if (prev_path_.poses.empty() || prev_path_cost_ >= std::numeric_limits<double>::max()) {
    return true;
  }

  double new_cost = planner_->lastPathCost();
  return new_cost < prev_path_cost_ / params_->hysteresis_factor;
}

double DStarLitePlanner::computePathCost(
  const nav_msgs::msg::Path & path) const
{
  if (path.poses.size() < 2) {
    return 0.0;
  }

  double cost = 0.0;
  for (size_t i = 1; i < path.poses.size(); i++) {
    double dx = path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x;
    double dy = path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y;
    cost += std::hypot(dx, dy);
  }
  return cost;
}

nav_msgs::msg::Path DStarLitePlanner::linearInterpolation(
  const std::vector<WorldCoord> & raw_path,
  double resolution)
{
  nav_msgs::msg::Path pa;
  geometry_msgs::msg::PoseStamped p1;

  for (size_t j = 0; j < raw_path.size() - 1; j++) {
    WorldCoord pt1 = raw_path[j];
    p1.pose.position.x = pt1.x;
    p1.pose.position.y = pt1.y;
    pa.poses.push_back(p1);

    WorldCoord pt2 = raw_path[j + 1];
    double distance = std::hypot(pt2.x - pt1.x, pt2.y - pt1.y);
    if (distance < 1e-6) {
      continue;
    }
    int loops = static_cast<int>(distance / resolution);
    double sin_alpha = (pt2.y - pt1.y) / distance;
    double cos_alpha = (pt2.x - pt1.x) / distance;
    for (int k = 1; k < loops; k++) {
      p1.pose.position.x = pt1.x + static_cast<double>(k) * resolution * cos_alpha;
      p1.pose.position.y = pt1.y + static_cast<double>(k) * resolution * sin_alpha;
      pa.poses.push_back(p1);
    }
  }

  if (!raw_path.empty()) {
    p1.pose.position.x = raw_path.back().x;
    p1.pose.position.y = raw_path.back().y;
    pa.poses.push_back(p1);
  }

  return pa;
}

}  // namespace nav2_dstar_lite_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  nav2_dstar_lite_planner::DStarLitePlanner,
  nav2_core::GlobalPlanner)
