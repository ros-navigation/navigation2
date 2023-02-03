// Copyright (c) 2023, Samsung Research America
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

#include "nav2_route/route_tracker.hpp"

namespace nav2_route
{

void RouteTracker::configure(
  nav2_util::LifecycleNode::SharedPtr node,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  std::shared_ptr<ActionServerTrack> action_server,
  const std::string & route_frame,
  const std::string & base_frame)
{
  clock_ = node->get_clock();
  logger_ = node->get_logger();
  route_frame_ = route_frame;
  base_frame_ = base_frame;
  action_server_ = action_server;
  tf_buffer_ = tf_buffer;

  nav2_util::declare_parameter_if_not_declared(
    node, "radius_to_achieve_node", rclcpp::ParameterValue(1.0));
  radius_threshold_ = node->get_parameter("radius_to_achieve_node").as_double();
  nav2_util::declare_parameter_if_not_declared(
    node, "tracker_update_rate", rclcpp::ParameterValue(100.0));
  tracker_update_rate_ = node->get_parameter("tracker_update_rate").as_double();
  nav2_util::declare_parameter_if_not_declared(
    node, "aggregate_blocked_ids", rclcpp::ParameterValue(false));
  aggregate_blocked_ids_ = node->get_parameter("aggregate_blocked_ids").as_bool();

  operations_manager_ = std::make_unique<OperationsManager>(node);
}

geometry_msgs::msg::PoseStamped RouteTracker::getRobotPose()
{
  geometry_msgs::msg::PoseStamped pose;
  if (!nav2_util::getCurrentPose(pose, *tf_buffer_, route_frame_, base_frame_)) {
    throw nav2_core::RouteTFError("Unable to get robot pose in route frame: " + route_frame_);
  }
  return pose;
}

// TODO(sm) test and visualize this with dot products and window
bool RouteTracker::nodeAchieved(
  const geometry_msgs::msg::PoseStamped & pose,
  RouteTrackingState & state,
  const Route & route)
{
  // check if inside a *generous* radius window
  const double dx = state.next_node->coords.x - pose.pose.position.x;
  const double dy = state.next_node->coords.y - pose.pose.position.y;
  const double dist_mag = std::sqrt(dx * dx + dy * dy);
  const bool in_radius = (dist_mag <= radius_threshold_);

  // Within 0.1mm is achieved
  if (dist_mag < 1e-4) {
    return true;
  }

  // If we were within radius and now not, consider node achieved in case we just barely kiss
  // the radial threshold set by the user coming in at an odd angle due to dynamic behavior
  if (!in_radius && state.within_radius) {
    return true;
  }

  state.within_radius = in_radius;

  // If start or end node, use the radius check only since the final node may not formally pass
  // the vectorized threshold depending on the local trajectory / goal checker configurations.
  // The start node has no last_node for computing the vector bisector. If this is an issue
  // for any users, please file a ticket to discuss.
  if (isStartOrEndNode(state.route_edges_idx)) {
    return state.within_radius;
  }

  // If we're within the radius, we can evaluate the unit distance vector from the node w.r.t. the
  // unit vector bisecting the last and current edges to find the average whose orthogonal is an
  // imaginery line representing the migration from one edge's spatial domain to the other.
  // Thus, when the dot product is positive, it means that there exists a projection between
  // the vectors and that the robot position has passed this imaginary orthogonal line.
  // This enables a more refined definition of when a node is considered achieved while enabling
  // the use of dynamic behavior local trajectory planners to deviate from the path non-trivially
  if (state.within_radius) {
    NodePtr last_node = state.current_edge->start;
    const double nx = state.next_node->coords.x - last_node->coords.x;
    const double ny = state.next_node->coords.y - last_node->coords.y;
    const double n_mag = std::sqrt(nx * nx + ny * ny);

    // If not the next node is not the route's end, then there exist another edge
    NodePtr future_next_node = route.edges[state.route_edges_idx + 1]->end;
    const double mx = future_next_node->coords.x - state.next_node->coords.x;
    const double my = future_next_node->coords.y - state.next_node->coords.y;
    const double m_mag = std::sqrt(mx * mx + my * my);

    // If nodes overlap so there is no vector, use radius check only (divide by zero)
    if (n_mag < 1e-6 || m_mag < 1e-6) {
      return true;
    }

    // Unnormalized Bisector = |n|*m + |m|*n
    const double bx = nx * m_mag + mx * n_mag;
    const double by = ny * m_mag + my * n_mag;
    const double b_mag = std::sqrt(bx * bx + by * by);
    return (dx / dist_mag) * (bx / b_mag) + (dy / dist_mag) * (by / b_mag) >= 0 ? true : false;
  }

  return false;
}

bool RouteTracker::isStartOrEndNode(int idx)
{
  return idx == static_cast<int>(route_msg_.edge_ids.size() - 1) || idx == -1;
}

void RouteTracker::publishFeedback(
  const bool rereouted,
  const unsigned int next_node_id,
  const unsigned int last_node_id,
  const unsigned int edge_id,
  const std::vector<std::string> & operations)
{
  auto feedback = std::make_unique<Feedback>();
  feedback->route = route_msg_;
  feedback->path = path_;
  feedback->rerouted = rereouted;
  feedback->next_node_id = next_node_id;
  feedback->last_node_id = last_node_id;
  feedback->current_edge_id = edge_id;
  feedback->operations_triggered = operations;
  action_server_->publish_feedback(std::move(feedback));
}

TrackerResult RouteTracker::trackRoute(
  const Route & route, const nav_msgs::msg::Path & path,
  std::vector<unsigned int> & blocked_ids)
{
  // Manage important data
  route_msg_ = utils::toMsg(route, route_frame_, clock_->now());
  path_ = path;
  RouteTrackingState state;
  state.next_node = route.start_node;

  // Publish initial feedback after routing or rerouting
  publishFeedback(true, route.start_node->nodeid, 0, 0, {});

  rclcpp::Rate r(tracker_update_rate_);
  while (rclcpp::ok()) {
    bool status_change = false, completed = false;

    // Check action server state is still OK to keep processing
    if (action_server_->is_cancel_requested()) {
      return TrackerResult::REROUTE;
    } else if (action_server_->is_preempt_requested()) {
      return TrackerResult::REROUTE;
    }

    // Update the tracking state
    geometry_msgs::msg::PoseStamped robot_pose = getRobotPose();
    if (nodeAchieved(robot_pose, state, route)) {
      status_change = true;
      state.within_radius = false;
      state.last_node = state.next_node;
      if (++state.route_edges_idx < static_cast<int>(route.edges.size())) {
        state.current_edge = route.edges[state.route_edges_idx];
        state.next_node = state.current_edge->end;
      } else {  // At achieved the last node in the route
        state.current_edge = nullptr;
        state.next_node = nullptr;
        completed = true;
      }
    }

    // Process any operations necessary
    OperationsResult ops_result =
      operations_manager_->process(status_change, state, route, robot_pose);

    if (completed) {
      RCLCPP_INFO(logger_, "Routing to goal completed!");
      return TrackerResult::COMPLETED;
    }

    if ((status_change || !ops_result.operations_triggered.empty()) && state.current_edge) {
      publishFeedback(
        false,  // No rerouting occurred
        state.next_node->nodeid, state.last_node->nodeid,
        state.current_edge->edgeid, ops_result.operations_triggered);
    }

    if (ops_result.reroute) {
      if (!aggregate_blocked_ids_) {
        blocked_ids = ops_result.blocked_ids;
      } else {
        blocked_ids.insert(
          blocked_ids.end(), ops_result.blocked_ids.begin(), ops_result.blocked_ids.end());
      }
      RCLCPP_INFO(logger_, "Rerouting requested by route tracking operations!");
      return TrackerResult::REROUTE;
    }

    r.sleep();
  }

  return TrackerResult::INTERRUPTED;
}

}  // namespace nav2_route
