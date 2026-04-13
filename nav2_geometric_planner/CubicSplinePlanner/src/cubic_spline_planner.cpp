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

#include "cubic_spline_planner.hpp"
#include "cubic_spline_math.hpp"
#include "geometric_planner_utils.hpp"

#include <cmath>

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(nav2_geometric_planners::CubicSplinePlanner, nav2_core::GlobalPlanner)

namespace nav2_geometric_planners
{

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void CubicSplinePlanner::configure(
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
}

void CubicSplinePlanner::cleanup() {}
void CubicSplinePlanner::activate() {}
void CubicSplinePlanner::deactivate() {}

// ---------------------------------------------------------------------------
// Parameters
// ---------------------------------------------------------------------------

void CubicSplinePlanner::loadParameters()
{
  auto node = node_.lock();
  node->declare_parameter(name_ + ".interpolation_resolution", params_.interpolation_resolution_);
  node->get_parameter(name_ + ".interpolation_resolution", params_.interpolation_resolution_);
}

// ---------------------------------------------------------------------------
// Planning
// ---------------------------------------------------------------------------

nav_msgs::msg::Path CubicSplinePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::vector<geometry_msgs::msg::PoseStamped> & viapoints,
  std::function<bool()> /*cancel_checker*/)
{
  validateInputs(start, goal, viapoints);

  // Assemble ordered waypoints.
  std::vector<geometry_msgs::msg::PoseStamped> waypoints;
  waypoints.reserve(viapoints.size() + 2);
  waypoints.push_back(start);
  waypoints.insert(waypoints.end(), viapoints.begin(), viapoints.end());
  waypoints.push_back(goal);

  const std::vector<double> times = extractTimes(waypoints);

  const double resolution = (params_.interpolation_resolution_ > 0.0)
    ? params_.interpolation_resolution_
    : costmap_->getResolution();

  nav_msgs::msg::Path path = sampleSpline(waypoints, times, resolution);
  nav2_geometric_planners::utils::checkPathCollisions(path, costmap_);
  return path;
}

void CubicSplinePlanner::validateInputs(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::vector<geometry_msgs::msg::PoseStamped> & viapoints)
{
  if (viapoints.empty()) {
    throw nav2_core::InsufficientViapoints(
      "CubicSplinePlanner requires at least one viapoint for a non-trivial spline");
  }
  nav2_geometric_planners::utils::validateStartGoal(start, goal, costmap_);
}

std::vector<double> CubicSplinePlanner::extractTimes(
  const std::vector<geometry_msgs::msg::PoseStamped> & waypoints)
{
  std::vector<double> times;
  times.reserve(waypoints.size());

  for (const auto & wp : waypoints) {
    if (!isValidStamp(wp.header.stamp)) {
      throw nav2_core::InvalidViapoints(
        "CubicSplinePlanner: all waypoints must have non-zero timestamps");
    }
    times.push_back(stampToSec(wp.header.stamp));
  }

  // Verify strictly increasing.
  for (std::size_t i = 1; i < times.size(); ++i) {
    if (times[i] <= times[i - 1]) {
      throw nav2_core::InvalidViapoints(
        "CubicSplinePlanner: waypoint timestamps must be strictly increasing");
    }
  }

  return times;
}

nav_msgs::msg::Path CubicSplinePlanner::sampleSpline(
  const std::vector<geometry_msgs::msg::PoseStamped> & waypoints,
  const std::vector<double> & times,
  double resolution)
{
  using nav2_geometric_planners::utils::makePose;

  // Collect x and y values from all waypoints.
  std::vector<double> xs, ys;
  xs.reserve(waypoints.size());
  ys.reserve(waypoints.size());
  for (const auto & wp : waypoints) {
    xs.push_back(wp.pose.position.x);
    ys.push_back(wp.pose.position.y);
  }

  CubicSpline1D spline_x, spline_y;
  spline_x.build(times, xs);
  spline_y.build(times, ys);

  // Sample at fine resolution and keep only poses spaced <= resolution apart.
  const double t_start = times.front();
  const double t_end   = times.back();
  const double dt_fine = (t_end - t_start) / 1000.0;

  nav_msgs::msg::Path path;
  path.header = waypoints.front().header;

  double prev_x = spline_x.evaluate(t_start);
  double prev_y = spline_y.evaluate(t_start);
  path.poses.push_back(makePose(prev_x, prev_y, 0.0, waypoints.front().header));

  for (double t = t_start + dt_fine; t <= t_end + 1e-9; t += dt_fine) {
    const double cx = spline_x.evaluate(t);
    const double cy = spline_y.evaluate(t);
    const double dist = std::hypot(cx - prev_x, cy - prev_y);

    if (dist >= resolution) {
      const double yaw = std::atan2(cy - prev_y, cx - prev_x);
      path.poses.push_back(makePose(cx, cy, yaw, waypoints.front().header));
      prev_x = cx;
      prev_y = cy;
    }
  }

  // Ensure goal pose is included.
  const double gx = spline_x.evaluate(t_end);
  const double gy = spline_y.evaluate(t_end);
  if (!path.poses.empty()) {
    const double yaw = std::atan2(gy - prev_y, gx - prev_x);
    path.poses.push_back(makePose(gx, gy, yaw, waypoints.back().header));
  }

  return path;
}

}  // namespace nav2_geometric_planners
