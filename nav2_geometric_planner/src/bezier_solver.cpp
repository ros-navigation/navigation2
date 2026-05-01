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

#include "nav2_geometric_planner/solvers/bezier_solver.hpp"
#include "bezier_math.hpp"
#include "geometric_planner_utils.hpp"

#include <cmath>

#include "nav2_core/planner_exceptions.hpp"

namespace nav2_geometric_planners
{

void BezierSolver::configure(
  const nav2::LifecycleNode::WeakPtr & parent,
  const std::string & name)
{
  auto node = parent.lock();
  node->declare_parameter(name + ".num_samples", num_samples_);
  node->get_parameter(name + ".num_samples", num_samples_);
}

nav_msgs::msg::Path BezierSolver::createPath(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::vector<geometry_msgs::msg::PoseStamped> & viapoints,
  double resolution)
{
  validateViapoints(viapoints);

  const auto control_pts = buildControlPoints(start, viapoints, goal);
  return sampleBezier(control_pts, resolution, start.header);
}

void BezierSolver::validateViapoints(
  const std::vector<geometry_msgs::msg::PoseStamped> & viapoints)
{
  if (viapoints.empty()) {
    throw nav2_core::InsufficientViapoints(
      "BezierSolver requires at least one viapoint as a control point");
  }
}

std::vector<geometry_msgs::msg::Point> BezierSolver::buildControlPoints(
  const geometry_msgs::msg::PoseStamped & start,
  const std::vector<geometry_msgs::msg::PoseStamped> & viapoints,
  const geometry_msgs::msg::PoseStamped & goal)
{
  std::vector<geometry_msgs::msg::Point> pts;
  pts.reserve(viapoints.size() + 2);

  pts.push_back(start.pose.position);
  for (const auto & vp : viapoints) {
    pts.push_back(vp.pose.position);
  }
  pts.push_back(goal.pose.position);

  return pts;
}

nav_msgs::msg::Path BezierSolver::sampleBezier(
  const std::vector<geometry_msgs::msg::Point> & control_points,
  double resolution,
  const std_msgs::msg::Header & header)
{
  using nav2_geometric_planners::utils::makePose;

  nav_msgs::msg::Path path;
  path.header = header;

  const int n = num_samples_;
  double prev_x = control_points.front().x;
  double prev_y = control_points.front().y;
  path.poses.push_back(makePose(prev_x, prev_y, 0.0, header));

  for (int i = 1; i <= n; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(n);
    const auto pt = evaluateBezier(control_points, t);
    const double dist = std::hypot(pt.x - prev_x, pt.y - prev_y);

    if (dist >= resolution) {
      const double yaw = std::atan2(pt.y - prev_y, pt.x - prev_x);
      path.poses.push_back(makePose(pt.x, pt.y, yaw, header));
      prev_x = pt.x;
      prev_y = pt.y;
    }
  }

  // Ensure the exact goal position is the last pose.
  const auto & last_ctrl = control_points.back();
  if (!path.poses.empty()) {
    const double yaw = std::atan2(last_ctrl.y - prev_y, last_ctrl.x - prev_x);
    path.poses.push_back(makePose(last_ctrl.x, last_ctrl.y, yaw, header));
  }

  return path;
}

}  // namespace nav2_geometric_planners
