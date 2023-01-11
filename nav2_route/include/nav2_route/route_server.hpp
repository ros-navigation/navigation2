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
#include "visualization_msgs/msg/marker_array.hpp"

#include "nav2_route/types.hpp"
#include "nav2_route/utils.hpp"
#include "nav2_route/graph_file_loader.hpp"
#include "nav2_route/node_spatial_tree.hpp"

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
  using ActionBasic = nav2_msgs::action::ComputeRoute;
  using ActionBasicGoal = ActionBasic::Goal;
  using ActionBasicResult = ActionBasic::Result;
  using ActionServerBasic = nav2_util::SimpleActionServer<ActionBasic>;

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
   * @brief Main route action server for computing a route and returning it to the requester
   */
  void computeRoute();

  /**
   * @brief Finds the start and goal Node IDs closest to the request
   * @param The request goal information
   * @return A pair of NodeIDs belonging to the start and goal nodes for route search
   */
  NodeExtents findStartandGoalNodeIDs(std::shared_ptr<const ActionBasicGoal> goal);

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::mutex dynamic_params_lock_;

  std::unique_ptr<ActionServerBasic> action_server_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

  // Publish the route for visualization
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr graph_vis_publisher_;

  // Get, set or modify graph
  // rclcpp::Service<nav2_msgs::srv::GetRouteGraph>::SharedPtr get_graph_service_;
  // rclcpp::Service<nav2_msgs::srv::SetRouteGraph>::SharedPtr set_graph_service_;
  // rclcpp::Service<nav2_msgs::srv::ModifyRouteGraph>::SharedPtr modify_graph_service_;

  // Interal tools
  std::shared_ptr<GraphFileLoader> graph_loader_;
  std::shared_ptr<NodeSpatialTree> node_spatial_tree_;

  // Navigation graph object and utils
  nav2_route::Graph graph_;
  std::string route_frame_, base_frame_;
  double max_planning_time_;
  // Plugin class loader + plugin to get it
};

}  // namespace nav2_route

#endif  // NAV2_ROUTE__ROUTE_SERVER_HPP_
