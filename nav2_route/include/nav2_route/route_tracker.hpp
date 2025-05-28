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

#include <vector>
#include <string>
#include <memory>

#include "tf2_ros/transform_listener.h"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav2_msgs/action/compute_and_track_route.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/route_exceptions.hpp"
#include "nav2_route/operations_manager.hpp"

#ifndef NAV2_ROUTE__ROUTE_TRACKER_HPP_
#define NAV2_ROUTE__ROUTE_TRACKER_HPP_

namespace nav2_route
{

/**
 * @class nav2_route::RouteTracker
 * @brief Takes a processing route request and tracks it for progress
 * in accordance with executing behavioral operations
 */
class RouteTracker
{
public:
  using ActionServerTrack = nav2_util::SimpleActionServer<nav2_msgs::action::ComputeAndTrackRoute>;
  using Feedback = nav2_msgs::action::ComputeAndTrackRoute::Feedback;

  /**
   * @brief A constructor for nav2_route::RouteTracker
   */
  RouteTracker() = default;

  /**
   * @brief A constructor for nav2_route::RouteTracker
   */
  ~RouteTracker() = default;

  /**
   * @brief Configure route tracker
   * @param node Node to grab info from
   * @param route_frame Frame of route navigation
   */
  void configure(
    nav2_util::LifecycleNode::SharedPtr node,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber,
    std::shared_ptr<ActionServerTrack> action_server,
    const std::string & route_frame,
    const std::string & base_frame);

  /**
   * @brief Determine if a node is to be considered achieved at the current position
   * @param pose Current robot pose to query
   * @param state Tracker state
   * @param route Route to check against
   * @return bool if node is achieved
   */
  bool nodeAchieved(
    const geometry_msgs::msg::PoseStamped & pose,
    RouteTrackingState & state,
    const Route & route);

  /**
   * @brief Determine if a node is the start or last node in the route
   * @param idx Idx of the current edge being tracked
   * @param route Route to check
   * @return bool if this node is the last node
   */
  bool isStartOrEndNode(RouteTrackingState & state, const Route & route);

  /**
   * @brief Get the current robot's base_frame pose in route_frame
   * @return Robot pose
   */
  geometry_msgs::msg::PoseStamped getRobotPose();

  /**
   * @brief A utility to publish feedback for the action on important changes
   * @param rerouted If the route has been rerouted
   * @param next_node_id Id of the next node the route is to pass
   * @param last_node_id Id of the last node the route passed
   * @param edge_id Id of the current edge being processed
   * @param operations A set of operations which were performed this iteration
   */
  void publishFeedback(
    const bool rereouted,
    const unsigned int next_node_id,
    const unsigned int last_node_id,
    const unsigned int edge_id,
    const std::vector<std::string> & operations);

  /**
   * @brief Main function to track route, manage state, and trigger operations
   * @param route Route to track progress of
   * @param path Path that comprises this route for publication of feedback messages
   * @param blocked_ids A set of blocked IDs to modify if rerouting is necessary
   * @return TrackerResult if the route is completed, should be rerouted, or was interrupted
   */
  TrackerResult trackRoute(
    const Route & route, const nav_msgs::msg::Path & path,
    ReroutingState & rerouting_info);

protected:
  nav2_msgs::msg::Route route_msg_;
  nav_msgs::msg::Path path_;
  std::string route_frame_, base_frame_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("RouteTracker")};
  double radius_threshold_, boundary_radius_threshold_, tracker_update_rate_;
  bool aggregate_blocked_ids_;
  std::shared_ptr<ActionServerTrack> action_server_;
  std::unique_ptr<OperationsManager> operations_manager_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

}  // namespace nav2_route

#endif  // NAV2_ROUTE__ROUTE_TRACKER_HPP_
