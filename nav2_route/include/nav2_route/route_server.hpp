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
#include "nav2_route/node_spatial_tree.hpp"
#include "nav2_route/path_converter.hpp"
#include "nav2_route/route_tracker.hpp"

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
   * @brief Finds the start and goal Node locations in the graph closest to the request
   * @param The request goal information
   * @return A pair of NodeIDs belonging to the start and goal nodes for route search
   */
  template<typename GoalT>
  NodeExtents findStartandGoalNodeLocations(const std::shared_ptr<const GoalT> goal);

  /**
   * @brief Abstract method combining findStartandGoalNodeLocations and the route planner
   * to find the Node locations of interest and route to the goal
   * @param The request goal information
   * @return A route of the request
   */
  template<typename GoalT>
  Route findRoute(const std::shared_ptr<const GoalT> goal);

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
  template<typename T>
  bool isRequestValid(std::shared_ptr<nav2_util::SimpleActionServer<T>> & action_server);

  /**
   * @brief The service callback to set a new route graph
   * @param request to the service
   * @param response from the service
   */
  void setRouteGraph(
    const std::shared_ptr<nav2_msgs::srv::SetRouteGraph::Request> request,
    std::shared_ptr<nav2_msgs::srv::SetRouteGraph::Response> response);

  /**
   * @brief Log exception warnings, templated by action message type
   * @param goal Goal that failed
   * @param exception Exception message
   */
  template<typename GoalT>
  void exceptionWarning(const std::shared_ptr<const GoalT> goal, const std::exception & ex);

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::mutex dynamic_params_lock_;

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
  std::shared_ptr<NodeSpatialTree> node_spatial_tree_;
  std::shared_ptr<RoutePlanner> route_planner_;
  std::shared_ptr<RouteTracker> route_tracker_;
  std::shared_ptr<PathConverter> path_converter_;

  // Data
  Graph graph_;
  GraphToIDMap id_to_graph_map_;
  std::string route_frame_, base_frame_;
  double max_planning_time_;
};

}  // namespace nav2_route

#endif  // NAV2_ROUTE__ROUTE_SERVER_HPP_
