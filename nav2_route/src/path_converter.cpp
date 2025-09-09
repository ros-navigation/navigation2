// Copyright (c) 2025, Open Navigation LLC
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

#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <mutex>
#include <algorithm>

#include "nav2_route/path_converter.hpp"

namespace nav2_route
{

void PathConverter::configure(nav2_util::LifecycleNode::SharedPtr node)
{
  // Density to make path points
  nav2_util::declare_parameter_if_not_declared(
    node, "path_density", rclcpp::ParameterValue(0.05));
  density_ = static_cast<float>(node->get_parameter("path_density").as_double());
  nav2_util::declare_parameter_if_not_declared(
    node, "smoothing_radius", rclcpp::ParameterValue(1.0));
  smoothing_radius_ = static_cast<float>(node->get_parameter("smoothing_radius").as_double());
  nav2_util::declare_parameter_if_not_declared(
    node, "smooth_corners", rclcpp::ParameterValue(false));
  smooth_corners_ = node->get_parameter("smooth_corners").as_bool();

  path_pub_ = node->create_publisher<nav_msgs::msg::Path>("plan", 10);
  path_pub_->on_activate();
  logger_ = node->get_logger();
}

nav_msgs::msg::Path PathConverter::densify(
  const Route & route,
  const ReroutingState & rerouting_info,
  const std::string & frame,
  const rclcpp::Time & now)
{
  nav_msgs::msg::Path path;
  path.header.stamp = now;
  path.header.frame_id = frame;

  // If we're rerouting and covering the same previous edge to start,
  // the path should contain the relevant partial information along edge
  // to avoid unnecessary free-space planning where state is retained
  if (rerouting_info.curr_edge) {
    const Coordinates & start = rerouting_info.closest_pt_on_edge;
    const Coordinates & end = rerouting_info.curr_edge->end->coords;
    interpolateEdge(start.x, start.y, end.x, end.y, path.poses);
  }

  Coordinates start;
  Coordinates end;

  if (!route.edges.empty()) {
    start = route.edges[0]->start->coords;

    // Fill in path via route edges
    for (unsigned int i = 0; i < route.edges.size() - 1; i++) {
      const EdgePtr edge = route.edges[i];
      const EdgePtr & next_edge = route.edges[i + 1];
      end = edge->end->coords;

      CornerArc corner_arc(start, end, next_edge->end->coords, smoothing_radius_);
      if (corner_arc.isCornerValid() && smooth_corners_) {
        // if an arc exists, end of the first edge is the start of the arc
        end = corner_arc.getCornerStart();

        // interpolate to start of arc
        interpolateEdge(start.x, start.y, end.x, end.y, path.poses);

        // interpolate arc
        corner_arc.interpolateArc(density_ / smoothing_radius_, path.poses);

        // new start of next edge is end of smoothing arc
        start = corner_arc.getCornerEnd();
      } else {
        if (smooth_corners_) {
          RCLCPP_WARN(
            logger_, "Unable to smooth corner between edge %i and edge %i", edge->edgeid,
            next_edge->edgeid);
        }
        interpolateEdge(start.x, start.y, end.x, end.y, path.poses);
        start = end;
      }
    }
  }

  if (route.edges.empty()) {
    path.poses.push_back(utils::toMsg(route.start_node->coords.x, route.start_node->coords.y));
  } else {
    interpolateEdge(
      start.x, start.y, route.edges.back()->end->coords.x,
      route.edges.back()->end->coords.y, path.poses);

    path.poses.push_back(
      utils::toMsg(route.edges.back()->end->coords.x, route.edges.back()->end->coords.y));
  }

  // Set path poses orientations for each point
  for (size_t i = 0; i < path.poses.size() - 1; ++i) {
    const auto & pose = path.poses[i];
    const auto & next_pose = path.poses[i + 1];
    const double dx = next_pose.pose.position.x - pose.pose.position.x;
    const double dy = next_pose.pose.position.y - pose.pose.position.y;
    const double yaw = atan2(dy, dx);
    path.poses[i].pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw);
  }

  // Set the last pose orientation to the last edge
  if (!route.edges.empty()) {
    const auto & last_edge = route.edges.back();
    const double dx = last_edge->end->coords.x - last_edge->start->coords.x;
    const double dy = last_edge->end->coords.y - last_edge->start->coords.y;
    path.poses.back().pose.orientation =
      nav2_util::geometry_utils::orientationAroundZAxis(atan2(dy, dx));
  }

  // publish path similar to planner server
  path_pub_->publish(std::make_unique<nav_msgs::msg::Path>(path));

  return path;
}

void PathConverter::interpolateEdge(
  float x0, float y0, float x1, float y1,
  std::vector<geometry_msgs::msg::PoseStamped> & poses)
{
  // Find number of points to populate by given density
  const float mag = hypotf(x1 - x0, y1 - y0);
  const unsigned int num_pts = ceil(mag / density_);
  // For zero-length edges, we can just push the start point and return
  if (num_pts < 1) {
    return;
  }

  const float iterpolated_dist = mag / num_pts;

  // Find unit vector direction
  float ux = (x1 - x0) / mag;
  float uy = (y1 - y0) / mag;

  // March along it until dist
  float x = x0;
  float y = y0;
  poses.push_back(utils::toMsg(x, y));

  unsigned int pt_ctr = 0;
  while (pt_ctr < num_pts - 1) {
    x += ux * iterpolated_dist;
    y += uy * iterpolated_dist;
    pt_ctr++;
    poses.push_back(utils::toMsg(x, y));
  }
}

}  // namespace nav2_route
