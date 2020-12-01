// Copyright (c) 2020 Fetullah Atas
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
#ifndef NAV2_GPS_WAYPOINT_FOLLOWER__GPS_WAYPOINT_FOLLOWER_HPP_
#define NAV2_GPS_WAYPOINT_FOLLOWER__GPS_WAYPOINT_FOLLOWER_HPP_

#include <vector>
#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/msg/point32.hpp"
#include "nav2_msgs/action/follow_gps_waypoints.hpp"
#include "nav2_lifecycle_manager/lifecycle_manager_client.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "robot_localization/srv/to_ll.hpp"
#include "robot_localization/srv/from_ll.hpp"

/**
 * @brief namespace for way point following, points are from a yaml file
 *
 */
namespace nav2_gps_waypoint_follower
{
enum class ActionStatus
{
  UNKNOWN = 0,
  PROCESSING = 1,
  FAILED = 2,
  SUCCEEDED = 3
};
/**
 * @brief A ros lifecyle node that drives robot through gievn way points from YAML file
 *
 */
class GPSWaypointFollower : public nav2_util::LifecycleNode
{
public:
  // Shorten the types
  using ActionT = nav2_msgs::action::FollowGPSWaypoints;
  using ClientT = nav2_msgs::action::FollowWaypoints;
  using ActionServer = nav2_util::SimpleActionServer<ActionT>;
  using ActionClient = rclcpp_action::Client<ClientT>;
  using WaypointFollowerGoalHandle =
    rclcpp_action::ClientGoalHandle<ClientT>;

  /**
   * @brief Construct a new Way Point Folllower Demo object
   *
   */
  GPSWaypointFollower();

  /**
   * @brief Destroy the Way Point Folllower Demo object
   *
   */
  ~GPSWaypointFollower();

protected:
  /**
   * @brief Configures member variables
   *
   * Initializes action server for "FollowWaypoints"
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Activates action server
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Deactivates action server
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Resets member variables
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
   * @brief Action client result callback
   * @param result Result of action server updated asynchronously
   */
  void resultCallback(const rclcpp_action::ClientGoalHandle<ClientT>::WrappedResult & result);

  /**
   * @brief Action client goal response callback
   * @param goal Response of action server updated asynchronously
   */
  void goalResponseCallback(const rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr & goal);

  /**
   * @brief send robot through each of the pose in poses vector
   *
   * @param poses
   */
  void followGPSWaypoints();

  /**
   * @brief
   *
   */
  static std::vector<geometry_msgs::msg::PoseStamped> convertGPSWaypointstoPosesinMap(
    const
    std::vector<sensor_msgs::msg::NavSatFix> & gps_waypoints,
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & parent_node,
    const rclcpp::Client<robot_localization::srv::FromLL>::SharedPtr & fromll_client);

  // dedicated node of client(FollowWaypoints)
  rclcpp::Node::SharedPtr client_node_;
  // FollowGPSWaypoints action server
  std::unique_ptr<ActionServer> action_server_;
  // client to call server from robot_localization to do UTM -> Map conversion
  rclcpp::Client<robot_localization::srv::FromLL>::SharedPtr from_ll_to_map_client_;
  // client to connect waypoint follower service(FollowWaypoints)
  rclcpp_action::Client<ClientT>::SharedPtr waypoint_follower_action_client_;
  // global var to get information about current goal state
  ActionStatus current_goal_status_;
  // stores the waypoints in a vector with additional info such as
  // "int32[] missed_waypoints" and "uint32
  // current_waypoint"
  ClientT::Goal waypoint_follower_goal_;
  // goal handler to query state of goal
  WaypointFollowerGoalHandle::SharedPtr waypoint_follower_goal_handle_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
};
}  // namespace nav2_gps_waypoint_follower

#endif  // NAV2_GPS_WAYPOINT_FOLLOWER__GPS_WAYPOINT_FOLLOWER_HPP_
