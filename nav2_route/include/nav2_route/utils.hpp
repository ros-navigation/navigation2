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

#include <limits>
#include <string>
#include <vector>
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_msgs/msg/route.hpp"
#include "nav2_route/types.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/line_iterator.hpp"

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

  visualization_msgs::msg::Marker nodes_marker;
  nodes_marker.header.frame_id = frame;
  nodes_marker.header.stamp = now;
  nodes_marker.action = 0;
  nodes_marker.ns = "route_graph_nodes";
  nodes_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  nodes_marker.scale.x = 0.1;
  nodes_marker.scale.y = 0.1;
  nodes_marker.scale.z = 0.1;
  nodes_marker.color.r = 1.0;
  nodes_marker.color.a = 1.0;
  nodes_marker.points.reserve(graph.size());

  visualization_msgs::msg::Marker edges_marker;
  edges_marker.header.frame_id = frame;
  edges_marker.header.stamp = now;
  edges_marker.action = 0;
  edges_marker.ns = "route_graph_edges";
  edges_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  edges_marker.scale.x = 0.05;  // Line width
  edges_marker.color.g = 1.0;
  edges_marker.color.a = 0.5;  // Semi-transparent green so bidirectional connections stand out
  constexpr size_t points_per_edge = 2;
  // This probably under-reserves but saves some initial reallocations
  constexpr size_t likely_min_edges_per_node = 2;
  edges_marker.points.reserve(graph.size() * points_per_edge * likely_min_edges_per_node);

  geometry_msgs::msg::Point node_pos;
  geometry_msgs::msg::Point edge_start;
  geometry_msgs::msg::Point edge_end;

  visualization_msgs::msg::Marker node_id_marker;
  node_id_marker.header.frame_id = frame;
  node_id_marker.header.stamp = now;
  node_id_marker.action = 0;
  node_id_marker.ns = "route_graph_node_ids";
  node_id_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  node_id_marker.scale.x = 0.1;
  node_id_marker.scale.y = 0.1;
  node_id_marker.scale.z = 0.1;
  node_id_marker.color.a = 1.0;
  node_id_marker.color.r = 1.0;

  visualization_msgs::msg::Marker edge_id_marker;
  edge_id_marker.header.frame_id = frame;
  edge_id_marker.header.stamp = now;
  edge_id_marker.action = 0;
  edge_id_marker.ns = "route_graph_edge_ids";
  edge_id_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  edge_id_marker.scale.x = 0.1;
  edge_id_marker.scale.y = 0.1;
  edge_id_marker.scale.z = 0.1;
  edge_id_marker.color.a = 1.0;
  edge_id_marker.color.g = 1.0;

  for (const auto & node : graph) {
    node_pos.x = node.coords.x;
    node_pos.y = node.coords.y;
    nodes_marker.points.push_back(node_pos);

    // Add text for Node ID
    node_id_marker.id++;
    node_id_marker.pose.position.x = node.coords.x + 0.07;
    node_id_marker.pose.position.y = node.coords.y;
    node_id_marker.text = std::to_string(node.nodeid);
    msg.markers.push_back(node_id_marker);

    for (const auto & neighbor : node.neighbors) {
      edge_start.x = node.coords.x;
      edge_start.y = node.coords.y;
      edge_end.x = neighbor.end->coords.x;
      edge_end.y = neighbor.end->coords.y;
      edges_marker.points.push_back(edge_start);
      edges_marker.points.push_back(edge_end);

      // Deal with overlapping bi-directional text markers by offsetting locations
      float y_offset = 0.0;
      if (node.nodeid > neighbor.end->nodeid) {
        y_offset = 0.05;
      } else {
        y_offset = -0.05;
      }
      const float x_offset = 0.07;

      // Add text for Edge ID
      edge_id_marker.id++;
      edge_id_marker.pose.position.x =
        node.coords.x + ((neighbor.end->coords.x - node.coords.x) / 2.0) + x_offset;
      edge_id_marker.pose.position.y =
        node.coords.y + ((neighbor.end->coords.y - node.coords.y) / 2.0) + y_offset;
      edge_id_marker.text = std::to_string(neighbor.edgeid);
      msg.markers.push_back(edge_id_marker);
    }
  }

  msg.markers.push_back(edges_marker);
  msg.markers.push_back(nodes_marker);
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
