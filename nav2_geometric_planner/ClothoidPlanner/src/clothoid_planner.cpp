// Copyright (c) 2025 Nav2 Contributors
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

#include "clothoid_planner.hpp"
#include "clothoid_math.hpp"
#include "geometric_planner_utils.hpp"

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(nav2_geometric_planners::ClothoidPlanner, nav2_core::GlobalPlanner)

namespace nav2_geometric_planners
{

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void ClothoidPlanner::configure(
  const nav2::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = name;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();

  loadParameters();
  setupDynamicParameters();
}

void ClothoidPlanner::cleanup()
{
  cb_handle_.reset();
  param_handler_.reset();
}

void ClothoidPlanner::activate() {}
void ClothoidPlanner::deactivate() {}

// ---------------------------------------------------------------------------
// Parameters
// ---------------------------------------------------------------------------

void ClothoidPlanner::loadParameters()
{
  auto node = node_.lock();

  node->declare_parameter(name_ + ".step_size", params_.step_size_);
  node->declare_parameter(name_ + ".max_curvature", params_.max_curvature_);
  node->declare_parameter(name_ + ".collision_check_resolution", params_.collision_check_resolution_);

  node->get_parameter(name_ + ".step_size", params_.step_size_);
  node->get_parameter(name_ + ".max_curvature", params_.max_curvature_);
  node->get_parameter(name_ + ".collision_check_resolution", params_.collision_check_resolution_);
}

void ClothoidPlanner::setupDynamicParameters()
{
  auto node = node_.lock();
  param_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(node);

  cb_handle_ = param_handler_->add_parameter_callback(
    name_,
    [this](const rclcpp::Parameter & p) {
      if (p.get_name() == name_ + ".step_size") {
        params_.step_size_ = p.as_double();
      } else if (p.get_name() == name_ + ".max_curvature") {
        params_.max_curvature_ = p.as_double();
      } else if (p.get_name() == name_ + ".collision_check_resolution") {
        params_.collision_check_resolution_ = p.as_double();
      }
    });
}

// ---------------------------------------------------------------------------
// Planning
// ---------------------------------------------------------------------------

nav_msgs::msg::Path ClothoidPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::vector<geometry_msgs::msg::PoseStamped> & viapoints,
  std::function<bool()> /*cancel_checker*/)
{
  validateInputs(start, goal);

  // Assemble ordered waypoints: start → viapoints → goal.
  std::vector<geometry_msgs::msg::PoseStamped> waypoints;
  waypoints.reserve(viapoints.size() + 2);
  waypoints.push_back(start);
  waypoints.insert(waypoints.end(), viapoints.begin(), viapoints.end());
  waypoints.push_back(goal);

  nav_msgs::msg::Path path = planThroughWaypoints(waypoints);
  nav2_geometric_planners::utils::checkPathCollisions(path, costmap_);
  return path;
}

void ClothoidPlanner::validateInputs(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav2_geometric_planners::utils::validateStartGoal(start, goal, costmap_);
}

nav_msgs::msg::Path ClothoidPlanner::planThroughWaypoints(
  const std::vector<geometry_msgs::msg::PoseStamped> & waypoints)
{
  nav_msgs::msg::Path path;
  path.header = waypoints.front().header;

  for (std::size_t i = 0; i + 1 < waypoints.size(); ++i) {
    appendClothoidSegment(waypoints[i], waypoints[i + 1], path);
  }

  return path;
}

void ClothoidPlanner::appendClothoidSegment(
  const geometry_msgs::msg::PoseStamped & from,
  const geometry_msgs::msg::PoseStamped & to,
  nav_msgs::msg::Path & path)
{
  using nav2_geometric_planners::utils::extractYaw;
  using nav2_geometric_planners::utils::makePose;

  ClothoidSegment seg;
  const bool ok = seg.buildG1(
    from.pose.position.x, from.pose.position.y, extractYaw(from.pose.orientation),
    to.pose.position.x,   to.pose.position.y,   extractYaw(to.pose.orientation));

  if (!ok || seg.length_ < 1e-10) {return;}

  for (double s = 0.0; s <= seg.length_; s += params_.step_size_) {
    if (std::abs(seg.kappa(s)) > params_.max_curvature_) {continue;}

    double x{}, y{};
    seg.eval(s, x, y);
    path.poses.push_back(makePose(x, y, seg.theta(s), from.header));
  }
}

}  // namespace nav2_geometric_planners
