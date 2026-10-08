// Copyright (c) 2026 Yong Ling
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

#include <cmath>
#include <utility>

#include "nav2_route/route_server.hpp"

namespace nav2_route
{

void RouteServer::trackPrecomputedRoute()
{
  auto & server = track_precomputed_route_server_;
  auto goal = server->get_current_goal();
  auto result = std::make_shared<TrackPrecomputedRoute::Result>();
  result->route_id = goal->route_id;
  std::unique_lock<std::mutex> tracking_lock(tracking_mutex_, std::try_to_lock);
  if (!tracking_lock.owns_lock()) {
    result->error_code = TrackPrecomputedRoute::Result::BUSY;
    result->error_msg = "Another route tracking request is active";
    server->terminate_current(result);
    return;
  }
  std::shared_lock<std::shared_mutex> graph_lock(graph_mutex_);
  auto start_time = now();
  try {
    while (rclcpp::ok() && server->is_server_active()) {
      if (server->is_cancel_requested()) {
        result->execution_duration = now() - start_time;
        server->terminate_current(result);
        return;
      }
      if (server->is_preempt_requested()) {
        result->execution_duration = now() - start_time;
        server->terminate_current(result);
        goal = server->accept_pending_goal();
        result = std::make_shared<TrackPrecomputedRoute::Result>();
        result->route_id = goal->route_id;
        start_time = now();
      }

      auto route = resolvePrecomputedRoute(graph_, goal->start_node_id, goal->edge_ids);
      const auto pose = route_tracker_->getRobotPose();
      const double distance = std::hypot(
        pose.pose.position.x - route.start_node->coords.x,
        pose.pose.position.y - route.start_node->coords.y);
      const double start_radius = get_parameter("boundary_radius_to_achieve_node").as_double();
      if (distance > start_radius) {
        throw nav2_core::NoValidRouteCouldBeFound(
                "Robot must be within boundary_radius_to_achieve_node of the starting node");
      }
      ReroutingState rerouting;
      auto path = path_converter_->densify(route, rerouting, route_frame_, now());
      publishRoute(route);

      RouteTracker::TrackingContext context;
      context.is_active = [&server]() {return server->is_server_active();};
      context.is_cancel_requested = [&server]() {return server->is_cancel_requested();};
      context.is_preempt_requested = [&server]() {return server->is_preempt_requested();};
      context.publish_feedback = [&server, &goal](
        std::unique_ptr<RouteTracker::Feedback> tracked)
        {
          auto feedback = std::make_unique<TrackPrecomputedRoute::Feedback>();
          feedback->route_id = goal->route_id;
          feedback->last_node_id = tracked->last_node_id;
          feedback->next_node_id = tracked->next_node_id;
          feedback->current_edge_id = tracked->current_edge_id;
          feedback->route = std::move(tracked->route);
          feedback->path = std::move(tracked->path);
          feedback->operations_triggered = std::move(tracked->operations_triggered);
          server->publish_feedback(std::move(feedback));
        };
      const auto status = route_tracker_->trackRoute(route, path, rerouting, context);
      result->execution_duration = now() - start_time;
      if (status == TrackerResult::COMPLETED) {
        server->succeeded_current(result);
        return;
      }
      if (status == TrackerResult::REROUTE_REQUESTED) {
        result->error_code = TrackPrecomputedRoute::Result::REROUTE_REQUIRED;
        result->error_msg = "Route operations requested replanning by the route provider";
        result->blocked_ids = rerouting.blocked_ids;
        server->terminate_current(result);
        return;
      }
      if (status == TrackerResult::EXITED) {
        break;
      }
      // Cancellation and same-action preemption are handled at the top of the loop.
    }
  } catch (const nav2_core::NoValidGraph & ex) {
    result->error_code = TrackPrecomputedRoute::Result::NO_VALID_GRAPH;
    result->error_msg = ex.what();
  } catch (const nav2_core::NoValidRouteCouldBeFound & ex) {
    result->error_code = TrackPrecomputedRoute::Result::INVALID_ROUTE;
    result->error_msg = ex.what();
  } catch (const nav2_core::RouteTFError & ex) {
    result->error_code = TrackPrecomputedRoute::Result::TF_ERROR;
    result->error_msg = ex.what();
  } catch (const nav2_core::OperationFailed & ex) {
    result->error_code = TrackPrecomputedRoute::Result::OPERATION_FAILED;
    result->error_msg = ex.what();
  } catch (const std::exception & ex) {
    result->error_code = TrackPrecomputedRoute::Result::UNKNOWN;
    result->error_msg = ex.what();
  }
  if (result->error_code == TrackPrecomputedRoute::Result::NONE) {
    result->error_code = TrackPrecomputedRoute::Result::UNKNOWN;
    result->error_msg = "Route tracking stopped before completion";
  }
  result->execution_duration = now() - start_time;
  server->terminate_current(result);
}

}  // namespace nav2_route
