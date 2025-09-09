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


#include <math.h>
#include <memory>
#include <string>

#include "nav2_route/plugins/route_operations/collision_monitor.hpp"

namespace nav2_route
{

void CollisionMonitor::configure(
  const nav2_util::LifecycleNode::SharedPtr node,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber,
  const std::string & name)
{
  name_ = name;
  clock_ = node->get_clock();
  logger_ = node->get_logger();
  last_check_time_ = clock_->now();

  std::string server_costmap_topic = node->get_parameter("costmap_topic").as_string();
  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".costmap_topic", rclcpp::ParameterValue("local_costmap/costmap_raw"));
  std::string costmap_topic = node->get_parameter(getName() + ".costmap_topic").as_string();
  if (costmap_topic != server_costmap_topic) {
    RCLCPP_INFO(node->get_logger(),
      "Using costmap topic: %s instead of server costmap topic: %s for CollisionMonitor.",
      costmap_topic.c_str(), server_costmap_topic.c_str());
    costmap_subscriber_ = std::make_shared<nav2_costmap_2d::CostmapSubscriber>(node, costmap_topic);
    topic_ = costmap_topic;
  } else {
    costmap_subscriber_ = costmap_subscriber;
    topic_ = server_costmap_topic;
  }

  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".rate", rclcpp::ParameterValue(1.0));
  double checking_rate = node->get_parameter(getName() + ".rate").as_double();
  checking_duration_ = rclcpp::Duration::from_seconds(1.0 / checking_rate);

  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".reroute_on_collision", rclcpp::ParameterValue(true));
  reroute_on_collision_ = node->get_parameter(getName() + ".reroute_on_collision").as_bool();

  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".max_cost", rclcpp::ParameterValue(253.0));
  max_cost_ = static_cast<float>(node->get_parameter(getName() + ".max_cost").as_double());

  // Resolution to check the costmap over (1=every cell, 2=every other cell, etc.)
  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".check_resolution", rclcpp::ParameterValue(1));
  check_resolution_ = static_cast<unsigned int>(
    node->get_parameter(getName() + ".check_resolution").as_int());

  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".max_collision_dist", rclcpp::ParameterValue(5.0));
  max_collision_dist_ = static_cast<float>(
    node->get_parameter(getName() + ".max_collision_dist").as_double());
  if (max_collision_dist_ <= 0.0) {
    RCLCPP_INFO(
      logger_, "Max collision distance to evaluate is zero or negative, checking the full route.");
    max_collision_dist_ = std::numeric_limits<float>::max();
  }
}

void CollisionMonitor::getCostmap()
{
  try {
    costmap_ = costmap_subscriber_->getCostmap();
  } catch (...) {
    throw nav2_core::OperationFailed(
            "Collision Monitor could not obtain a costmap from topic: " + topic_);
  }
}

OperationResult CollisionMonitor::perform(
  NodePtr /*node*/,
  EdgePtr curr_edge,
  EdgePtr /*edge_exited*/,
  const Route & route,
  const geometry_msgs::msg::PoseStamped & curr_pose,
  const Metadata * /*mdata*/)
{
  // Not time yet to check or before getting to first route edge
  auto now = clock_->now();
  if (now - last_check_time_ < checking_duration_ || !curr_edge) {
    return OperationResult();
  }
  last_check_time_ = now;

  OperationResult result;
  getCostmap();

  float dist_checked = 0.0;
  Coordinates end = curr_edge->end->coords;
  Coordinates start = utils::findClosestPoint(
    curr_pose, curr_edge->start->coords, end);
  unsigned int curr_edge_id = curr_edge->edgeid;

  bool final_edge = false;
  while (!final_edge) {
    // Track how far we've checked and should check for collisions
    const float edge_dist = hypotf(end.x - start.x, end.y - start.y);
    if (dist_checked + edge_dist >= max_collision_dist_) {
      float dist_to_eval = max_collision_dist_ - dist_checked;
      end = backoutValidEndPoint(start, end, dist_to_eval);
      final_edge = true;
    }
    dist_checked += edge_dist;

    // Find the valid edge grid coords, in case the edge is partially off the grid
    LineSegment line;
    if (!lineToMap(start, end, line)) {
      final_edge = true;
      if (!backoutValidEndPoint(start, line)) {
        break;
      }
    }

    // Collision check edge on grid within max distance and
    // report blocked edges for rerouting or exit task
    if (isInCollision(line)) {
      RCLCPP_INFO(
        logger_, "Collision has been detected within %0.2fm of robot pose!", max_collision_dist_);

      if (reroute_on_collision_) {
        result.reroute = true;
        result.blocked_ids.push_back(curr_edge_id);
        return result;
      }

      throw nav2_core::OperationFailed(
              "Collision detected, but rerouting is not enabled, canceling tracking task.");
    }

    // Restart loop for next edge until complete
    start = end;
    if (!final_edge) {
      auto isCurrEdge = [&](const EdgePtr & edge) {return edge->edgeid == curr_edge_id;};
      auto iter = std::find_if(route.edges.begin(), route.edges.end(), isCurrEdge);
      if (iter != route.edges.end() && ++iter != route.edges.end()) {
        // If we found the edge and the next edge is also valid
        curr_edge_id = (*iter)->edgeid;
        end = (*iter)->end->coords;
      } else {
        final_edge = true;
      }
    }
  }

  return result;
}

Coordinates CollisionMonitor::backoutValidEndPoint(
  const Coordinates & start, const Coordinates & end, const float dist)
{
  Coordinates new_end;
  const float dx = end.x - start.x;
  const float dy = end.y - start.y;
  const float mag = hypotf(dx, dy);
  if (mag < 1e-6) {
    return start;
  }
  new_end.x = (dx / mag) * dist + start.x;
  new_end.y = (dy / mag) * dist + start.y;
  return new_end;
}

bool CollisionMonitor::backoutValidEndPoint(
  const Coordinates & start, LineSegment & line)
{
  // Check if any part of this edge is potentially valid
  if (!costmap_->worldToMap(start.x, start.y, line.x0, line.y0)) {
    return false;
  }

  // Since worldToMap will populate the out-of-bounds (x1, y1), we can
  // iterate through the partially valid line until we hit invalid coords
  unsigned int last_end_x = line.x0, last_end_y = line.y0;
  nav2_util::LineIterator iter(line.x0, line.y0, line.x1, line.y1);
  int size_x = static_cast<int>(costmap_->getSizeInCellsX());
  int size_y = static_cast<int>(costmap_->getSizeInCellsY());
  for (; iter.isValid(); iter.advance()) {
    if (iter.getX() >= size_x || iter.getY() >= size_y) {
      line.x1 = last_end_x;
      line.y1 = last_end_y;
      return true;
    }
    last_end_x = iter.getX();
    last_end_y = iter.getY();
  }

  return false;
}

bool CollisionMonitor::lineToMap(
  const Coordinates & start, const Coordinates & end, LineSegment & line)
{
  if (!costmap_->worldToMap(start.x, start.y, line.x0, line.y0) ||
    !costmap_->worldToMap(end.x, end.y, line.x1, line.y1))
  {
    return false;
  }
  return true;
}

bool CollisionMonitor::isInCollision(const LineSegment & line)
{
  nav2_util::LineIterator iter(line.x0, line.y0, line.x1, line.y1);
  for (; iter.isValid(); ) {
    float cost = static_cast<float>(costmap_->getCost(iter.getX(), iter.getY()));
    if (cost >= max_cost_ && cost != 255.0 /*unknown*/) {
      return true;
    }

    // Advance the iterator by the check resolution on the edge, pruning to a coarse resolution
    for (unsigned int i = 0; i < check_resolution_; i++) {
      iter.advance();
    }
  }
  return false;
}

}  // namespace nav2_route

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_route::CollisionMonitor, nav2_route::RouteOperation)
