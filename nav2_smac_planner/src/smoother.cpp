// Copyright (c) 2021, Samsung Research America
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

#include <vector>
#include "nav2_smac_planner/smoother.hpp"

namespace nav2_smac_planner
{
using namespace nav2_util::geometry_utils;  // NOLINT
using namespace std::chrono;  // NOLINT

Smoother::Smoother(const SmootherParams & params)
{
  tolerance_ = params.tolerance_;
  max_its_ = params.max_its_;
  data_w_ = params.w_data_;
  smooth_w_ = params.w_smooth_;
  is_holonomic_ = params.holonomic_;
}

void Smoother::initialize(const double & min_turning_radius)
{
  min_turning_rad_ = min_turning_radius;
}

bool Smoother::smooth(
  nav_msgs::msg::Path & path,
  const nav2_costmap_2d::Costmap2D * costmap,
  const double & max_time,
  const bool do_refinement)
{
  steady_clock::time_point start = steady_clock::now();
  double time_remaining = max_time;
  bool success = true;
  nav_msgs::msg::Path curr_path_segment;
  curr_path_segment.header = path.header;
  std::vector<PathSegment> path_segments = findDirectionalPathSegments(path);

  for (unsigned int i = 0; i != path_segments.size(); i++) {
    if (path_segments[i].end - path_segments[i].start > 6) {
      curr_path_segment.poses.clear();
      std::copy(
        path.poses.begin() + path_segments[i].start,
        path.poses.begin() + path_segments[i].end + 1,
        std::back_inserter(curr_path_segment.poses));

      steady_clock::time_point now = steady_clock::now();
      time_remaining = max_time - duration_cast<duration<double>>(now - start).count();
      success = success && smoothImpl(curr_path_segment, costmap, time_remaining, do_refinement);

      // Assemble the path changes to the main path
      std::copy(
        curr_path_segment.poses.begin(),
        curr_path_segment.poses.end(),
        path.poses.begin() + path_segments[i].start);
    }
  }

  return success;
}

bool Smoother::smoothImpl(
  nav_msgs::msg::Path & path,
  const nav2_costmap_2d::Costmap2D * costmap,
  const double & max_time,
  const bool do_refinement)
{
  steady_clock::time_point a = steady_clock::now();
  rclcpp::Duration max_dur = rclcpp::Duration::from_seconds(max_time);

  int its = 0;
  double change = tolerance_;
  const unsigned int & path_size = path.poses.size();
  double x_i, y_i, y_m1, y_ip1, y_im2, y_ip2, y_i_org, curvature;
  unsigned int mx, my;

  // Adding 5% margin due to floating point error
  const double max_curvature = (1.0 / min_turning_rad_) * 1.05;
  nav_msgs::msg::Path new_path = path;
  nav_msgs::msg::Path last_path = path;

  while (change >= tolerance_) {
    its += 1;
    change = 0.0;

    // Make sure the smoothing function will converge
    if (its >= max_its_) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("SmacPlannerSmoother"),
        "Number of iterations has exceeded limit of %i.", max_its_);
      path = last_path;
      updateApproximatePathOrientations(path);
      return false;
    }

    // Make sure still have time left to process
    steady_clock::time_point b = steady_clock::now();
    rclcpp::Duration timespan(duration_cast<duration<double>>(b - a));
    if (timespan > max_dur) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("SmacPlannerSmoother"),
        "Smoothing time exceeded allowed duration of %0.2f.", max_time);
      path = last_path;
      updateApproximatePathOrientations(path);
      return false;
    }

    for (unsigned int i = 2; i != path_size - 2; i++) {
      for (unsigned int j = 0; j != 2; j++) {
        x_i = getFieldByDim(path.poses[i], j);
        y_i = getFieldByDim(new_path.poses[i], j);
        y_m1 = getFieldByDim(new_path.poses[i - 1], j);
        y_ip1 = getFieldByDim(new_path.poses[i + 1], j);
        y_i_org = y_i;

        if (i > 2 && i < path_size - 2) {
          // Smooth based on local 5 point neighborhood and original data locations
          y_im2 = getFieldByDim(new_path.poses[i - 2], j);
          y_ip2 = getFieldByDim(new_path.poses[i + 2], j);
          y_i += data_w_ * (x_i - y_i) + smooth_w_ * (y_im2 + y_ip2 + y_ip1 + y_m1 - (4.0 * y_i));
        } else {
          // Smooth based on local 3 point neighborhood and original data locations
          // At boundary conditions, need to use a more local neighborhood because the first
          // and last 2 points cannot move to ensure the boundry conditions are upheld
          y_i += data_w_ * (x_i - y_i) + smooth_w_ * (y_ip1 + y_m1 - (2.0 * y_i));
        }

        setFieldByDim(new_path.poses[i], j, y_i);
        change += abs(y_i - y_i_org);
      }

      // validate update is admissible, only checks cost if a valid costmap pointer is provided
      float cost = 0.0;
      if (costmap) {
        costmap->worldToMap(
          getFieldByDim(new_path.poses[i], 0),
          getFieldByDim(new_path.poses[i], 1),
          mx, my);
        cost = static_cast<float>(costmap->getCost(mx, my));
      }
      if (cost > MAX_NON_OBSTACLE) {
        RCLCPP_DEBUG(
          rclcpp::get_logger("SmacPlannerSmoother"),
          "Smoothing process resulted in an infeasible collision. "
          "Returning the last path before the infeasibility was introduced.");
        path = last_path;
        updateApproximatePathOrientations(path);
        return false;
      }
    }

    last_path = new_path;
  }

  // Lets do additional refinement, it shouldn't take more than a couple milliseconds
  // but really puts the path quality over the top.
  if (do_refinement) {
    smoothImpl(new_path, costmap, max_time, false);
  }

  for (unsigned int i = 3; i != path_size - 3; i++) {
    if (getCurvature(new_path, i) > max_curvature) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("SmacPlannerSmoother"),
        "Smoothing process resulted in an infeasible curvature somewhere on the path. "
        "This is most likely at the end point boundary conditions which will be further "
        "refined as a perfect curve as you approafch the goal. If this becomes a practical "
        "issue for you, please file a ticket mentioning this message.");
      updateApproximatePathOrientations(new_path);
      path = new_path;
      return false;
    }
  }

  updateApproximatePathOrientations(new_path);
  path = new_path;
  return true;
}

double Smoother::getFieldByDim(
  const geometry_msgs::msg::PoseStamped & msg, const unsigned int & dim)
{
  if (dim == 0) {
    return msg.pose.position.x;
  } else if (dim == 1) {
    return msg.pose.position.y;
  } else {
    return msg.pose.position.z;
  }
}

void Smoother::setFieldByDim(
  geometry_msgs::msg::PoseStamped & msg, const unsigned int dim,
  const double & value)
{
  if (dim == 0) {
    msg.pose.position.x = value;
  } else if (dim == 1) {
    msg.pose.position.y = value;
  } else {
    msg.pose.position.z = value;
  }
}

std::vector<PathSegment> Smoother::findDirectionalPathSegments(const nav_msgs::msg::Path & path)
{
  std::vector<PathSegment> segments;
  PathSegment curr_segment;
  curr_segment.start = 0;

  // If holonomic, no directional changes and
  // may have abrupt angular changes from naive grid search
  if (is_holonomic_) {
    curr_segment.end = path.poses.size() - 1;
    segments.push_back(curr_segment);
    return segments;
  }

  // Iterating through the path to determine the position of the cusp
  for (unsigned int idx = 1; idx < path.poses.size() - 1; ++idx) {
    // We have two vectors for the dot product OA and AB. Determining the vectors.
    double oa_x = path.poses[idx].pose.position.x -
      path.poses[idx - 1].pose.position.x;
    double oa_y = path.poses[idx].pose.position.y -
      path.poses[idx - 1].pose.position.y;
    double ab_x = path.poses[idx + 1].pose.position.x -
      path.poses[idx].pose.position.x;
    double ab_y = path.poses[idx + 1].pose.position.y -
      path.poses[idx].pose.position.y;

    // Checking for the existance of cusp, in the path, using the dot product.
    double dot_product = (oa_x * ab_x) + (oa_y * ab_y);
    if (dot_product < 0.0) {
      curr_segment.end = idx;
      segments.push_back(curr_segment);
      curr_segment.start = idx;
    }
  }

  curr_segment.end = path.poses.size() - 1;
  segments.push_back(curr_segment);
  return segments;
}

double Smoother::getCurvature(const nav_msgs::msg::Path & path, const unsigned int i)
{
  // k = 1 / r = acos(delta_phi) / |xi|, where delta_phi = (xi dot xi+1) / (|xi| * |xi+1|)
  const double dxi_x = getFieldByDim(path.poses[i], 0) - getFieldByDim(path.poses[i - 1], 0);
  const double dxi_y = getFieldByDim(path.poses[i], 1) - getFieldByDim(path.poses[i - 1], 1);
  const double dxip1_x = getFieldByDim(path.poses[i + 1], 0) - getFieldByDim(path.poses[i], 0);
  const double dxip1_y = getFieldByDim(path.poses[i + 1], 1) - getFieldByDim(path.poses[i], 1);
  const double norm_dx_i = hypot(dxi_x, dxi_y);
  const double norm_dx_ip1 = hypot(dxip1_x, dxip1_y);
  double arg = (dxi_x * dxip1_x + dxi_y * dxip1_y) / (norm_dx_i * norm_dx_ip1);

  // In case of small out of bounds issues from floating point error
  if (arg > 1.0) {
    arg = 1.0;
  } else if (arg < -1.0) {
    arg = -1.0;
  }

  return acos(arg) / norm_dx_i;
}

void Smoother::updateApproximatePathOrientations(nav_msgs::msg::Path & path)
{
  double dx, dy, theta, pt_yaw;
  bool reverse = false;

  // Find if this path segment is in reverse
  dx = path.poses[2].pose.position.x - path.poses[1].pose.position.x;
  dy = path.poses[2].pose.position.y - path.poses[1].pose.position.y;
  theta = atan2(dy, dx);
  pt_yaw = tf2::getYaw(path.poses[1].pose.orientation);
  if (!is_holonomic_ && fabs(angles::shortest_angular_distance(pt_yaw, theta)) > M_PI_2) {
    reverse = true;
  }

  for (unsigned int i = 0; i != path.poses.size() - 1; i++) {
    dx = path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x;
    dy = path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y;
    theta = atan2(dy, dx);

    // If points are overlapping, pass
    if (fabs(dx) < 1e-4 && fabs(dy) < 1e-4) {
      continue;
    }

    // Flip the angle if this path segment is in reverse
    if (reverse) {
      theta += M_PI;  // orientationAroundZAxis will normalize
    }

    path.poses[i].pose.orientation = orientationAroundZAxis(theta);
  }
}

}  // namespace nav2_smac_planner
