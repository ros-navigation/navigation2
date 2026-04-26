// Copyright (c) 2026 Sanchit Badamikar
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

#include "nav2_geometric_planner/geometric_planner.hpp"
#include "nav2_geometric_planner/solvers/bezier_solver.hpp"
#include "nav2_geometric_planner/solvers/cubic_spline_solver.hpp"
#include "nav2_geometric_planner/solvers/clothoid_solver.hpp"
#include "geometric_planner_utils.hpp"

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(nav2_geometric_planners::GeometricPlanner, nav2_core::GlobalPlanner)

namespace nav2_geometric_planners
{

void GeometricPlanner::configure(
  const nav2::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = name;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();

  auto node = node_.lock();

  std::string solver_type;
  node->declare_parameter(name_ + ".solver", std::string("bezier"));
  node->get_parameter(name_ + ".solver", solver_type);

  if (solver_type == "bezier") {
    solver_ = std::make_unique<BezierSolver>();
  } else if (solver_type == "cubic_spline") {
    solver_ = std::make_unique<CubicSplineSolver>();
  } else if (solver_type == "clothoid") {
    solver_ = std::make_unique<ClothoidSolver>();
  } else {
    RCLCPP_FATAL(node->get_logger(), "GeometricPlanner: unknown solver '%s'", solver_type.c_str());
    throw std::runtime_error("GeometricPlanner: unknown solver '" + solver_type + "'");
  }

  solver_->configure(parent, name_);
}

void GeometricPlanner::cleanup()
{
  solver_.reset();
}

void GeometricPlanner::activate() {}
void GeometricPlanner::deactivate() {}

nav_msgs::msg::Path GeometricPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::vector<geometry_msgs::msg::PoseStamped> & viapoints,
  std::function<bool()> /*cancel_checker*/)
{
  nav2_geometric_planners::utils::validateStartGoal(start, goal, costmap_);

  nav_msgs::msg::Path path = solver_->createPath(
    start, goal, viapoints, costmap_->getResolution());

  nav2_geometric_planners::utils::checkPathCollisions(path, costmap_);

  return path;
}

}  // namespace nav2_geometric_planners
