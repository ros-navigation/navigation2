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
// limitations under the License. Reserved.

#include "nav2_route/route_server.hpp"

using nav2_util::declare_parameter_if_not_declared;
using std::placeholders::_1;
using std::placeholders::_2;

namespace nav2_route
{

RouteServer::RouteServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("route_server", "", options)
{}

nav2_util::CallbackReturn
RouteServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(),
    get_node_timers_interface());
  tf_->setCreateTimerInterface(timer_interface);
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

  auto node = shared_from_this();
  graph_vis_publisher_ =
    node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "route_graph", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  compute_route_server_ = std::make_shared<ComputeRouteServer>(
    node, "compute_route",
    std::bind(&RouteServer::computeRoute, this),
    nullptr, std::chrono::milliseconds(500), true);

  compute_and_track_route_server_ = std::make_shared<ComputeAndTrackRouteServer>(
    node, "compute_and_track_route",
    std::bind(&RouteServer::computeAndTrackRoute, this),
    nullptr, std::chrono::milliseconds(500), true);

  set_graph_service_ = node->create_service<nav2_msgs::srv::SetRouteGraph>(
    std::string(node->get_name()) + "/set_route_graph",
    std::bind(
      &RouteServer::setRouteGraph, this,
      std::placeholders::_1, std::placeholders::_2));

  declare_parameter_if_not_declared(
    node, "route_frame", rclcpp::ParameterValue(std::string("map")));
  declare_parameter_if_not_declared(
    node, "base_frame", rclcpp::ParameterValue(std::string("base_link")));
  declare_parameter_if_not_declared(
    node, "max_planning_time", rclcpp::ParameterValue(2.0));

  route_frame_ = node->get_parameter("route_frame").as_string();
  base_frame_ = node->get_parameter("base_frame").as_string();
  max_planning_time_ = node->get_parameter("max_planning_time").as_double();

  try {
    graph_loader_ = std::make_shared<GraphLoader>(node, tf_, route_frame_);
    if (!graph_loader_->loadGraphFromFile(graph_, id_to_graph_map_)) {
      return nav2_util::CallbackReturn::FAILURE;
    }

    goal_intent_extractor_ = std::make_shared<GoalIntentExtractor>();
    goal_intent_extractor_->configure(
      node, graph_, &id_to_graph_map_, tf_, route_frame_, base_frame_);

    route_planner_ = std::make_shared<RoutePlanner>();
    route_planner_->configure(node);

    route_tracker_ = std::make_shared<RouteTracker>();
    route_tracker_->configure(
      node, tf_, compute_and_track_route_server_, route_frame_, base_frame_);

    path_converter_ = std::make_shared<PathConverter>();
    path_converter_->configure(node);
  } catch (std::exception & e) {
    RCLCPP_FATAL(get_logger(), "Failed to configure route server: %s", e.what());
    return nav2_util::CallbackReturn::FAILURE;
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
RouteServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");
  compute_route_server_->activate();
  compute_and_track_route_server_->activate();
  graph_vis_publisher_->on_activate();
  graph_vis_publisher_->publish(utils::toMsg(graph_, route_frame_, this->now()));
  createBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
RouteServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  compute_route_server_->deactivate();
  compute_and_track_route_server_->deactivate();
  graph_vis_publisher_->on_deactivate();
  destroyBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
RouteServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  compute_route_server_.reset();
  compute_and_track_route_server_.reset();
  set_graph_service_.reset();
  graph_loader_.reset();
  route_planner_.reset();
  route_tracker_.reset();
  path_converter_.reset();
  goal_intent_extractor_.reset();
  graph_vis_publisher_.reset();
  transform_listener_.reset();
  tf_.reset();
  graph_.clear();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
RouteServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

rclcpp::Duration
RouteServer::findPlanningDuration(const rclcpp::Time & start_time)
{
  auto cycle_duration = this->now() - start_time;
  if (max_planning_time_ && cycle_duration.seconds() > max_planning_time_) {
    RCLCPP_WARN(
      get_logger(),
      "Route planner missed its desired rate of %.4f Hz. Current loop rate is %.4f Hz",
      1 / max_planning_time_, 1 / cycle_duration.seconds());
  }

  return cycle_duration;
}

template<typename ActionT>
bool
RouteServer::isRequestValid(
  std::shared_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server)
{
  if (!action_server || !action_server->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
    return false;
  }

  if (action_server->is_cancel_requested()) {
    RCLCPP_INFO(get_logger(), "Goal was canceled. Canceling route planning action.");
    action_server->terminate_all();
    return false;
  }

  if (graph_.empty()) {
    RCLCPP_INFO(get_logger(), "No graph set! Aborting request.");
    action_server->terminate_all();
    return false;
  }

  return true;
}

void RouteServer::populateActionResult(
  std::shared_ptr<ComputeRoute::Result> result,
  const Route & route,
  const nav_msgs::msg::Path & path,
  const rclcpp::Duration & planning_duration)
{
  result->route = utils::toMsg(route, route_frame_, this->now());
  result->path = path;
  result->planning_time = planning_duration;
}

template<typename GoalT>
Route RouteServer::findRoute(
  const std::shared_ptr<const GoalT> goal,
  ReroutingState & rerouting_info)
{
  // Find the search boundaries
  auto [start_route, end_route] = goal_intent_extractor_->findStartandGoal(goal);

  if (rerouting_info.rerouting_start_id != std::numeric_limits<unsigned int>::max()) {
    start_route = id_to_graph_map_.at(rerouting_info.rerouting_start_id);
    goal_intent_extractor_->setStart(rerouting_info.rerouting_start_pose);
  }

  Route route;
  if (start_route == end_route) {
    // Succeed with a single-point route
    route.route_cost = 0.0;
    route.start_node = &graph_.at(start_route);
  } else {
    // Compute the route via graph-search, returns a node-edge sequence
    route = route_planner_->findRoute(graph_, start_route, end_route, rerouting_info.blocked_ids);
  }

  return goal_intent_extractor_->pruneStartandGoal(route, goal, rerouting_info);
}

template<typename ActionT>
void
RouteServer::processRouteRequest(
  std::shared_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server)
{
  auto goal = action_server->get_current_goal();
  auto result = std::make_shared<typename ActionT::Result>();
  ReroutingState rerouting_info;

  try {
    while (rclcpp::ok()) {
      if (!isRequestValid(action_server)) {
        return;
      }

      if (action_server->is_preempt_requested()) {
        RCLCPP_INFO(get_logger(), "Computing new preempted route to goal.");
        goal = action_server->accept_pending_goal();
        rerouting_info.reset();
      }

      // Find the route
      auto start_time = this->now();
      Route route = findRoute(goal, rerouting_info);
      auto path = path_converter_->densify(route, rerouting_info, route_frame_, this->now());
      auto planning_duration = findPlanningDuration(start_time);

      if (std::is_same<ActionT, ComputeAndTrackRoute>::value) {
        // blocks until re-route requested or task completion, publishes feedback
        switch (route_tracker_->trackRoute(route, path, rerouting_info)) {
          case TrackerResult::COMPLETED:
            populateActionResult(result, route, path, planning_duration);
            action_server->succeeded_current(result);
            return;
          case TrackerResult::REROUTE:
            break;
          case TrackerResult::INTERRUPTED:
            return;
        }
      } else {
        // Return route if not tracking
        populateActionResult(result, route, path, planning_duration);
        action_server->succeeded_current(result);
        return;
      }
    }
  } catch (nav2_core::NoValidRouteCouldBeFound & ex) {
    exceptionWarning(goal, ex);
    result->error_code = ActionT::Goal::NO_VALID_ROUTE;
    action_server->terminate_current(result);
  } catch (nav2_core::TimedOut & ex) {
    exceptionWarning(goal, ex);
    result->error_code = ActionT::Goal::TIMEOUT;
    action_server->terminate_current(result);
  } catch (nav2_core::RouteTFError & ex) {
    exceptionWarning(goal, ex);
    result->error_code = ActionT::Goal::TF_ERROR;
    action_server->terminate_current(result);
  } catch (nav2_core::NoValidGraph & ex) {
    exceptionWarning(goal, ex);
    result->error_code = ActionT::Goal::NO_VALID_GRAPH;
    action_server->terminate_current(result);
  } catch (nav2_core::IndeterminantNodesOnGraph & ex) {
    exceptionWarning(goal, ex);
    result->error_code = ActionT::Goal::INDETERMINANT_NODES_ON_GRAPH;
    action_server->terminate_current(result);
  } catch (nav2_core::OperationFailed & ex) {
    // A special case since Operation Failed is only in Compute & Track
    // actions, specifying it to allow otherwise fully shared code
    exceptionWarning(goal, ex);
    result->error_code = ComputeAndTrackRoute::Goal::OPERATION_FAILED;
    action_server->terminate_current(result);
  } catch (nav2_core::RouteException & ex) {
    exceptionWarning(goal, ex);
    result->error_code = ActionT::Goal::UNKNOWN;
    action_server->terminate_current(result);
  } catch (std::exception & ex) {
    exceptionWarning(goal, ex);
    result->error_code = ActionT::Goal::UNKNOWN;
    action_server->terminate_current(result);
  }
}

void
RouteServer::computeRoute()
{
  RCLCPP_INFO(get_logger(), "Computing route to goal.");
  processRouteRequest(compute_route_server_);
}

void
RouteServer::computeAndTrackRoute()
{
  RCLCPP_INFO(get_logger(), "Computing and tracking route to goal.");
  processRouteRequest(compute_and_track_route_server_);
}

void RouteServer::setRouteGraph(
  const std::shared_ptr<nav2_msgs::srv::SetRouteGraph::Request> request,
  std::shared_ptr<nav2_msgs::srv::SetRouteGraph::Response> response)
{
  RCLCPP_INFO(get_logger(), "Setting new route graph: %s.", request->graph_filepath.c_str());
  graph_.clear();
  id_to_graph_map_.clear();
  try {
    if (!graph_loader_->loadGraphFromFile(graph_, id_to_graph_map_, request->graph_filepath)) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to set new route graph: %s!", request->graph_filepath.c_str());
      response->success = false;
      return;
    }
  } catch (std::exception & ex) {
    RCLCPP_WARN(
      get_logger(),
      "Failed to set new route graph due to %s!", ex.what());
    response->success = false;
  }

  goal_intent_extractor_->setGraph(graph_, &id_to_graph_map_);
  graph_vis_publisher_->publish(utils::toMsg(graph_, route_frame_, this->now()));
  response->success = true;
}

template<typename GoalT>
void RouteServer::exceptionWarning(
  const std::shared_ptr<const GoalT> goal,
  const std::exception & ex)
{
  RCLCPP_WARN(
    get_logger(),
    "Route server failed on request: Start: [(%0.2f, %0.2f) / %i] Goal: [(%0.2f, %0.2f) / %i]:"
    " \"%s\"", goal->start.pose.position.x, goal->start.pose.position.y, goal->start_id,
    goal->goal.pose.position.x, goal->goal.pose.position.y, goal->goal_id, ex.what());
}

}  // namespace nav2_route

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_route::RouteServer)
