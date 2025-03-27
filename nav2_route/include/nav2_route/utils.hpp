// Copyright (c) 2025 Open Navigation LLC
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
#include <vector>
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_msgs/msg/route.hpp"
#include "nav2_route/types.hpp"

#ifndef NAV2_ROUTE__UTILS_HPP_
#define NAV2_ROUTE__UTILS_HPP_

namespace nav2_route
{

namespace utils
{

/**
 * @brief Convert the position into a pose
 * @param x X Coordinates
 * @param y Y Coordinates
 * @return PoseStamped of the position
 */
inline geometry_msgs::msg::PoseStamped toMsg(const float x, const float y)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  return pose;
}

/**
 * @brief Convert the route graph into a visualization marker array for visualization
 * @param graph Graph of nodes and edges
 * @param frame Frame ID to use
 * @param now Current time to use
 * @return MarkerArray of the graph
 */
inline visualization_msgs::msg::MarkerArray toMsg(
  const nav2_route::Graph & graph, const std::string & frame, const rclcpp::Time & now)
{
  visualization_msgs::msg::MarkerArray msg;
  visualization_msgs::msg::Marker curr_marker;
  curr_marker.header.frame_id = frame;
  curr_marker.header.stamp = now;
  curr_marker.action = 0;

  auto getSphereSize = []() {
      geometry_msgs::msg::Vector3 v_msg;
      v_msg.x = 0.05;
      v_msg.y = 0.05;
      v_msg.z = 0.05;
      return v_msg;
    };

  auto getSphereColor = []() {
      std_msgs::msg::ColorRGBA c_msg;
      c_msg.r = 1.0;
      c_msg.g = 0.0;
      c_msg.b = 0.0;
      c_msg.a = 1.0;
      return c_msg;
    };

  auto getLineColor = []() {
      std_msgs::msg::ColorRGBA c_msg;
      c_msg.r = 0.0;
      c_msg.g = 1.0;
      c_msg.b = 0.0;
      c_msg.a = 0.5;  // So bi-directional connections stand out overlapping
      return c_msg;
    };

  unsigned int marker_idx = 1;
  for (unsigned int i = 0; i != graph.size(); i++) {
    curr_marker.ns = "route_graph";
    curr_marker.id = marker_idx++;
    curr_marker.type = visualization_msgs::msg::Marker::SPHERE;
    curr_marker.pose.position.x = graph[i].coords.x;
    curr_marker.pose.position.y = graph[i].coords.y;
    curr_marker.scale = getSphereSize();
    curr_marker.color = getSphereColor();
    msg.markers.push_back(curr_marker);

    // Add text
    curr_marker.ns = "route_graph_ids";
    curr_marker.id = marker_idx++;
    curr_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    curr_marker.pose.position.x = graph[i].coords.x + 0.07;
    curr_marker.pose.position.y = graph[i].coords.y;
    curr_marker.text = std::to_string(graph[i].nodeid);
    curr_marker.scale.z = 0.1;
    msg.markers.push_back(curr_marker);

    for (unsigned int j = 0; j != graph[i].neighbors.size(); j++) {
      curr_marker.ns = "route_graph";
      curr_marker.id = marker_idx++;
      curr_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      curr_marker.pose.position.x = 0;  // Set to 0 since points are relative to this frame
      curr_marker.pose.position.y = 0;  // Set to 0 since points are relative to this frame
      curr_marker.points.resize(2);
      curr_marker.points[0].x = graph[i].coords.x;
      curr_marker.points[0].y = graph[i].coords.y;
      curr_marker.points[1].x = graph[i].neighbors[j].end->coords.x;
      curr_marker.points[1].y = graph[i].neighbors[j].end->coords.y;
      curr_marker.scale.x = 0.03;
      curr_marker.color = getLineColor();
      msg.markers.push_back(curr_marker);
      curr_marker.points.clear();  // Reset for next node marker

      // Add text
      curr_marker.ns = "route_graph_ids";
      curr_marker.id = marker_idx++;
      curr_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      curr_marker.pose.position.x =
        graph[i].coords.x + ((graph[i].neighbors[j].end->coords.x - graph[i].coords.x) / 2.0) +
        0.07;

      // Deal with overlapping bi-directional text markers by offsetting locations
      float y_offset = 0.0;
      if (graph[i].nodeid > graph[i].neighbors[j].end->nodeid) {
        y_offset = 0.05;
      } else {
        y_offset = -0.05;
      }

      curr_marker.pose.position.y =
        graph[i].coords.y + ((graph[i].neighbors[j].end->coords.y - graph[i].coords.y) / 2.0) +
        y_offset;
      curr_marker.text = std::to_string(graph[i].neighbors[j].edgeid);
      curr_marker.scale.z = 0.1;
      msg.markers.push_back(curr_marker);
    }
  }

  return msg;
}

/**
 * @brief Convert the route into a message
 * @param route Route of nodes and edges
 * @param frame Frame ID to use
 * @param now Current time to use
 * @return Route message
 */
inline nav2_msgs::msg::Route toMsg(
  const nav2_route::Route & route, const std::string & frame, const rclcpp::Time & now)
{
  nav2_msgs::msg::Route msg;
  msg.header.frame_id = frame;
  msg.header.stamp = now;
  msg.route_cost = route.route_cost;

  nav2_msgs::msg::RouteNode route_node;
  nav2_msgs::msg::RouteEdge route_edge;
  route_node.nodeid = route.start_node->nodeid;
  route_node.position.x = route.start_node->coords.x;
  route_node.position.y = route.start_node->coords.y;
  msg.nodes.push_back(route_node);

  // Provide the Node info and Edge IDs we're traversing through
  for (unsigned int i = 0; i != route.edges.size(); i++) {
    route_edge.edgeid = route.edges[i]->edgeid;
    route_edge.start.x = route.edges[i]->start->coords.x;
    route_edge.start.y = route.edges[i]->start->coords.y;
    route_edge.end.x = route.edges[i]->end->coords.x;
    route_edge.end.y = route.edges[i]->end->coords.y;
    msg.edges.push_back(route_edge);

    route_node.nodeid = route.edges[i]->end->nodeid;
    route_node.position.x = route.edges[i]->end->coords.x;
    route_node.position.y = route.edges[i]->end->coords.y;
    msg.nodes.push_back(route_node);
  }

  return msg;
}

/**
 * @brief Finds the normalized dot product of 2 vectors
 * @param v1x Vector 1's x component
 * @param v1y Vector 1's y component
 * @param v2x Vector 2's x component
 * @param v2y Vector 2's y component
 * @return Value of dot product
 */
inline float normalizedDot(
  const float v1x, const float v1y,
  const float v2x, const float v2y)
{
  const float mag1 = std::hypotf(v1x, v1y);
  const float mag2 = std::hypotf(v2x, v2y);
  if (mag1 < 1e-6 || mag2 < 1e-6) {
    return 0.0;
  }
  return (v1x / mag1) * (v2x / mag2) + (v1y / mag1) * (v2y / mag2);
}

/**
 * @brief Finds the closest point on the line segment made up of start-end to pose
 * @param pose Pose to find point closest on the line with respect to
 * @param start Start of line segment
 * @param end End of line segment
 * @return Coordinates of point on the line closest to the pose
 */
inline Coordinates findClosestPoint(
  const geometry_msgs::msg::PoseStamped & pose,
  const Coordinates & start, const Coordinates & end)
{
  Coordinates pt;
  const float vx = end.x - start.x;
  const float vy = end.y - start.y;
  const float ux = start.x - pose.pose.position.x;
  const float uy = start.y - pose.pose.position.y;
  const float uv = vx * ux + vy * uy;
  const float vv = vx * vx + vy * vy;

  // They are the same point, so only one option
  if (vv < 1e-6) {
    return start;
  }

  const float t = -uv / vv;
  if (t > 0.0 && t < 1.0) {
    pt.x = (1.0 - t) * start.x + t * end.x;
    pt.y = (1.0 - t) * start.y + t * end.y;
  } else if (t <= 0.0) {
    pt = start;
  } else {
    pt = end;
  }

  return pt;
}

inline float distance(const Coordinates & coords, const geometry_msgs::msg::PoseStamped & pose)
{
  return hypotf(coords.x - pose.pose.position.x, coords.y - pose.pose.position.y);
}

}  // namespace utils

}  // namespace nav2_route

#endif  // NAV2_ROUTE__UTILS_HPP_
