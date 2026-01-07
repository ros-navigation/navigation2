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

#include "nav2_route/route_tracker.hpp"

namespace nav2_route
{

void RouteTracker::configure(
  nav2_util::LifecycleNode::SharedPtr node,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber,
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
    node, "radius_to_achieve_node", rclcpp::ParameterValue(2.0));
  radius_threshold_ = node->get_parameter("radius_to_achieve_node").as_double();
  nav2_util::declare_parameter_if_not_declared(
    node, "boundary_radius_to_achieve_node", rclcpp::ParameterValue(1.0));
  boundary_radius_threshold_ = node->get_parameter("boundary_radius_to_achieve_node").as_double();
  nav2_util::declare_parameter_if_not_declared(
    node, "tracker_update_rate", rclcpp::ParameterValue(50.0));
  tracker_update_rate_ = node->get_parameter("tracker_update_rate").as_double();
  nav2_util::declare_parameter_if_not_declared(
    node, "aggregate_blocked_ids", rclcpp::ParameterValue(false));
  aggregate_blocked_ids_ = node->get_parameter("aggregate_blocked_ids").as_bool();

  operations_manager_ = std::make_unique<OperationsManager>(node, costmap_subscriber);
}

geometry_msgs::msg::PoseStamped RouteTracker::getRobotPose()
{
  geometry_msgs::msg::PoseStamped pose;
  if (!nav2_util::getCurrentPose(pose, *tf_buffer_, route_frame_, base_frame_)) {
    throw nav2_core::RouteTFError("Unable to get robot pose in route frame: " + route_frame_);
  }
  return pose;
}

bool RouteTracker::nodeAchieved(
  const geometry_msgs::msg::PoseStamped & pose,
  RouteTrackingState & state,
  const Route & route)
{
  // check if inside a *generous* radius window
  const double dx = state.next_node->coords.x - pose.pose.position.x;
  const double dy = state.next_node->coords.y - pose.pose.position.y;
  const double dist_mag = std::sqrt(dx * dx + dy * dy);
  const bool is_boundary_node = isStartOrEndNode(state, route);
  const bool in_radius =
    (dist_mag <= (is_boundary_node ? boundary_radius_threshold_ : radius_threshold_));

  // Within 0.1mm is achieved or within radius and now not, consider node achieved
  if (dist_mag < 1e-4 || (!in_radius && state.within_radius)) {
    return true;
  }

  // Update the state for the next iteration
  state.within_radius = in_radius;

  // If start or end node, use the radius check only since the final node may not pass
  // threshold depending on the configurations. The start node has no last_node for
  // computing the vector bisector. If this is an issue, please file a ticket to discuss.
  if (is_boundary_node) {
    return state.within_radius;
  }

  // We can evaluate the unit distance vector from the node w.r.t. the unit vector bisecting
  // the last and current edges to find the average whose orthogonal is an imaginary
  // line representing the migration from one edge's spatial domain to the other.
  // When the dot product is negative, it means that there exists a projection between
  // the vectors and that the robot position has passed this imaginary orthogonal line.
  // This enables a more refined definition of when a node is considered achieved while
  // enabling the use of dynamic behavior that may deviate from the path non-trivially
  if (state.within_radius) {
    NodePtr last_node = state.current_edge->start;
    const double nx = state.next_node->coords.x - last_node->coords.x;
    const double ny = state.next_node->coords.y - last_node->coords.y;
    const double n_mag = std::sqrt(nx * nx + ny * ny);

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
    return utils::normalizedDot(bx, by, dx, dy) <= 0;
  }

  return false;
}

bool RouteTracker::isStartOrEndNode(RouteTrackingState & state, const Route & route)
{
  // Check if current_edge is nullptr in case we have a rerouted previous
  // edge to use for the refined node achievement vectorized estimate
  return
    (state.route_edges_idx == static_cast<int>(route.edges.size() - 1)) ||
    (state.route_edges_idx == -1 && !state.current_edge);
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
  ReroutingState & rerouting_info)
{
  route_msg_ = utils::toMsg(route, route_frame_, clock_->now());
  path_ = path;
  RouteTrackingState state;
  state.next_node = route.start_node;

  // If we're rerouted but still covering the same previous edge to
  // start, retain the state so we can continue as previously set with
  // refined node achievement logic and performing edge operations on exit
  if (rerouting_info.curr_edge) {
    // state.next_node is not updated since the first edge is removed from route when rerouted
    // along the same edge in the goal intent extractor. Thus, state.next_node is still the
    // future node to reach in this case and we add in the state.last_node and state.current_edge
    // to represent the 'currently' progressing edge that is omitted from the route (and its start)
    state.current_edge = rerouting_info.curr_edge;
    state.last_node = state.current_edge->start;
    publishFeedback(
      true, route.start_node->nodeid, state.last_node->nodeid, state.current_edge->edgeid, {});
  } else {
    publishFeedback(true, route.start_node->nodeid, 0, 0, {});
  }

  rclcpp::Rate r(tracker_update_rate_);
  while (rclcpp::ok()) {
    bool status_change = false, completed = false;

    // Check if OK to keep processing
    if (action_server_->is_cancel_requested()) {
      return TrackerResult::INTERRUPTED;
    } else if (action_server_->is_preempt_requested()) {
      return TrackerResult::INTERRUPTED;
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
      operations_manager_->process(status_change, state, route, robot_pose, rerouting_info);

    if (completed) {
      RCLCPP_INFO(logger_, "Routing to goal completed!");
      // Publishing last feedback
      publishFeedback(false, 0, state.last_node->nodeid, 0, ops_result.operations_triggered);
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
        rerouting_info.blocked_ids = ops_result.blocked_ids;
      } else {
        rerouting_info.blocked_ids.insert(
          rerouting_info.blocked_ids.end(),
          ops_result.blocked_ids.begin(), ops_result.blocked_ids.end());
      }

      if (state.last_node) {
        rerouting_info.rerouting_start_id = state.last_node->nodeid;
        rerouting_info.rerouting_start_pose = robot_pose;
      } else {
        rerouting_info.rerouting_start_id = std::numeric_limits<unsigned int>::max();
        rerouting_info.rerouting_start_pose = geometry_msgs::msg::PoseStamped();
      }

      // Update so during rerouting we can check if we are continuing on the same edge
      rerouting_info.curr_edge = state.current_edge;
      RCLCPP_INFO(logger_, "Rerouting requested by route tracking operations!");
      return TrackerResult::INTERRUPTED;
    }

    r.sleep();
  }

  return TrackerResult::EXITED;
}

}  // namespace nav2_route
