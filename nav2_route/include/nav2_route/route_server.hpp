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

#ifndef NAV2_ROUTE__ROUTE_SERVER_HPP_
#define NAV2_ROUTE__ROUTE_SERVER_HPP_

#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <unordered_map>
#include <mutex>

#include "nav2_util/lifecycle_node.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "nav2_util/simple_action_server.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_msgs/action/compute_route.hpp"
#include "nav2_msgs/action/compute_and_track_route.hpp"
#include "nav2_msgs/msg/route.hpp"
#include "nav2_msgs/msg/route_node.hpp"
#include "nav2_msgs/srv/set_route_graph.hpp"
#include "nav2_core/route_exceptions.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "nav2_route/types.hpp"
#include "nav2_route/utils.hpp"
#include "nav2_route/graph_loader.hpp"
#include "nav2_route/route_planner.hpp"
#include "nav2_route/path_converter.hpp"
#include "nav2_route/route_tracker.hpp"
#include "nav2_route/goal_intent_extractor.hpp"

namespace nav2_route
{
/**
 * @class nav2_route::RouteServer
 * @brief An action server implements a Navigation Route-Graph planner
 * to compliment free-space planning in the Planner Server
 */
class RouteServer : public nav2_util::LifecycleNode
{
public:
  using ComputeRoute = nav2_msgs::action::ComputeRoute;
  using ComputeRouteGoal = ComputeRoute::Goal;
  using ComputeRouteResult = ComputeRoute::Result;
  using ComputeRouteServer = nav2_util::SimpleActionServer<ComputeRoute>;

  using ComputeAndTrackRoute = nav2_msgs::action::ComputeAndTrackRoute;
  using ComputeAndTrackRouteGoal = ComputeAndTrackRoute::Goal;
  using ComputeAndTrackRouteFeedback = ComputeAndTrackRoute::Feedback;
  using ComputeAndTrackRouteResult = ComputeAndTrackRoute::Result;
  using ComputeAndTrackRouteServer = nav2_util::SimpleActionServer<ComputeAndTrackRoute>;

  /**
   * @brief A constructor for nav2_route::RouteServer
   * @param options Additional options to control creation of the node.
   */
  explicit RouteServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief A destructor for nav2_route::RouteServer
   */
  ~RouteServer() = default;

protected:
  /**
   * @brief Configure member variables and initializes planner
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Activate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Deactivate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Reset member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Called when in shutdown state
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Main route action server callbacks for computing and tracking a route
   */
  void computeRoute();
  void computeAndTrackRoute();

  /**
   * @brief Abstract method combining finding the starting/ending nodes and the route planner
   * to find the Node locations of interest and route to the goal
   * @param goal The request goal information
   * @param blocked_ids The IDs of blocked graphs / edges
   * @param updated_start_id The nodeID of an updated starting point when tracking
   * a route that corresponds to the next point to route to from to continue progress
   * @return A route of the request
   */
  template<typename GoalT>
  Route findRoute(
    const std::shared_ptr<const GoalT> goal,
    ReroutingState & rerouting_info = ReroutingState());

  /**
   * @brief Main processing called by both action server callbacks to centralize
   * the great deal of shared code between them
   */
  template<typename ActionT>
  void processRouteRequest(
    std::shared_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server);

  /**
   * @brief Find the planning duration of the request and log warnings
   * @param start_time Start of planning time
   * @return Duration of planning time
   */
  rclcpp::Duration findPlanningDuration(const rclcpp::Time & start_time);

  /**
   * @brief Find the routing request is valid (action server OK and not cancelled)
   * @param action_server Actions server to check
   * @return if the request is valid
   */
  template<typename ActionT>
  bool isRequestValid(std::shared_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server);

  /**
   * @brief Populate result for compute route action
   * @param result Result to populate
   * @param route Route to use to populate result
   * @param path Path to use to populate result
   * @param planning_duration Time to create a valid route
   */
  void populateActionResult(
    std::shared_ptr<ComputeRoute::Result> result,
    const Route & route,
    const nav_msgs::msg::Path & path,
    const rclcpp::Duration & planning_duration);

  /**
   * @brief Populate result for compute and track route action
   * @param result Result to populate
   * @param route Route to use to populate result
   * @param path Path to use to populate result
   * @param planning_duration Time to create a valid route
   */
  void populateActionResult(
    std::shared_ptr<ComputeAndTrackRoute::Result>/*result*/,
    const Route & /*route*/,
    const nav_msgs::msg::Path & /*path*/,
    const rclcpp::Duration & /*planning_duration*/);

  /**
   * @brief The service callback to set a new route graph
   * @param request_header to the service
   * @param request to the service
   * @param response from the service
   */
  void setRouteGraph(
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<nav2_msgs::srv::SetRouteGraph::Request> request,
    std::shared_ptr<nav2_msgs::srv::SetRouteGraph::Response> response);

  /**
   * @brief Log exception warnings, templated by action message type
   * @param goal Goal that failed
   * @param exception Exception message
   */
  template<typename GoalT>
  void exceptionWarning(const std::shared_ptr<const GoalT> goal, const std::exception & ex);

  std::shared_ptr<ComputeRouteServer> compute_route_server_;
  std::shared_ptr<ComputeAndTrackRouteServer> compute_and_track_route_server_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

  // Publish the route for visualization
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    graph_vis_publisher_;

  // Set or modify graph
  rclcpp::Service<nav2_msgs::srv::SetRouteGraph>::SharedPtr set_graph_service_;

  // Internal tools
  std::shared_ptr<GraphLoader> graph_loader_;
  std::shared_ptr<RoutePlanner> route_planner_;
  std::shared_ptr<RouteTracker> route_tracker_;
  std::shared_ptr<PathConverter> path_converter_;
  std::shared_ptr<GoalIntentExtractor> goal_intent_extractor_;

  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber_;

  // State Data
  Graph graph_;
  GraphToIDMap id_to_graph_map_;
  std::string route_frame_, base_frame_, global_frame_;
  double max_planning_time_;
};

}  // namespace nav2_route

#endif  // NAV2_ROUTE__ROUTE_SERVER_HPP_
