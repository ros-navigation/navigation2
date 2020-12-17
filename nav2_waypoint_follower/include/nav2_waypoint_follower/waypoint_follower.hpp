// Copyright (c) 2019 Samsung Research America
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

#ifndef NAV2_WAYPOINT_FOLLOWER__WAYPOINT_FOLLOWER_HPP_
#define NAV2_WAYPOINT_FOLLOWER__WAYPOINT_FOLLOWER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp_action/rclcpp_action.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_msgs/msg/oriented_nav_sat_fix.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_msgs/action/follow_gps_waypoints.hpp"
#include "nav2_util/service_client.hpp"
#include "nav2_core/waypoint_task_executor.hpp"

#include "robot_localization/srv/from_ll.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"

namespace nav2_waypoint_follower
{

enum class ActionStatus
{
  UNKNOWN = 0,
  PROCESSING = 1,
  FAILED = 2,
  SUCCEEDED = 3
};

/**
 * @class nav2_waypoint_follower::WaypointFollower
 * @brief An action server that uses behavior tree for navigating a robot to its
 * goal position.
 */
class WaypointFollower : public nav2_util::LifecycleNode
{
public:
  using ActionT = nav2_msgs::action::FollowWaypoints;
  using ClientT = nav2_msgs::action::NavigateToPose;
  using ActionServer = nav2_util::SimpleActionServer<ActionT>;
  using ActionClient = rclcpp_action::Client<ClientT>;

  // Shorten the types for GPS waypoint following
  using ActionTGPS = nav2_msgs::action::FollowGPSWaypoints;
  using ActionServerGPS = nav2_util::SimpleActionServer<ActionTGPS>;

  /**
   * @brief A constructor for nav2_waypoint_follower::WaypointFollower class
   */
  WaypointFollower();
  /**
   * @brief A destructor for nav2_waypoint_follower::WaypointFollower class
   */
  ~WaypointFollower();

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
   * @brief Templated function to perform internal logic behind waypoint following,
   *        Both GPS and non GPS waypoint following callbacks makes use of this function when a client asked to do so.
   *        Callbacks fills in appropriate types for the tempelated types, see followWaypointCallback funtions for details.
   *
   * @tparam T action_server
   * @tparam V feedback
   * @tparam Z result
   * @param action_server
   * @param poses
   * @param feedback
   * @param result
   */
  template<typename T, typename V, typename Z>
  void followWaypointsLogic(
    const T & action_server,
    const V & feedback,
    const Z & result);

  /**
   * @brief Action server callbacks
   */
  void followWaypointsCallback();

  /**
   * @brief send robot through each of GPS
   *        point , which are converted to map frame first then using a client to
   *        `FollowWaypoints` action.
   *
   * @param waypoints, acquired from action client
   */
  void followGPSWaypointsCallback();

  /**
 * @brief Action client result callback
 * @param result Result of action server updated asynchronously
 */
  template<typename T>
  void resultCallback(const T & result);

  /**
   * @brief Action client goal response callback
   * @param goal Response of action server updated asynchronously
   */
  template<typename T>
  void goalResponseCallback(const T & goal);

/**
 * @brief given some gps_poses, converts them to map frame using robot_localization's service `fromLL`.
 *        Constructs a vector of stamped poses in map frame and returns them.
 *
 * @param gps_poses
 * @param parent_node
 * @param fromll_client
 * @return std::vector<geometry_msgs::msg::PoseStamped>
 */
  std::vector<geometry_msgs::msg::PoseStamped> convertGPSPoses2MapPoses(
    const std::vector<nav2_msgs::msg::OrientedNavSatFix> & gps_poses,
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & parent_node,
    const
    std::unique_ptr<nav2_util::ServiceClient<robot_localization::srv::FromLL>> & fromll_client);

  template<typename T>
  std::vector<geometry_msgs::msg::PoseStamped> getLatestGoalPoses(const T & action_server);

  // Common vars used for both GPS and cartesian point following
  rclcpp::Node::SharedPtr client_node_;
  std::vector<int> failed_ids_;
  int loop_rate_;
  bool stop_on_failure_;

  // Our action server for waypoint following
  std::unique_ptr<ActionServer> action_server_;
  ActionClient::SharedPtr nav_to_pose_client_;
  std::shared_future<rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr> future_goal_handle_;
  ActionStatus current_goal_status_;

  // Our action server for GPS waypoint following
  std::unique_ptr<ActionServerGPS> gps_action_server_;
  std::unique_ptr<nav2_util::ServiceClient<robot_localization::srv::FromLL>> from_ll_to_map_client_;

  // Task Execution At Waypoint Plugin
  pluginlib::ClassLoader<nav2_core::WaypointTaskExecutor>
  waypoint_task_executor_loader_;
  pluginlib::UniquePtr<nav2_core::WaypointTaskExecutor>
  waypoint_task_executor_;
  std::string waypoint_task_executor_id_;
  std::string waypoint_task_executor_type_;

  // tf buffer to get transfroms
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace nav2_waypoint_follower

#endif  // NAV2_WAYPOINT_FOLLOWER__WAYPOINT_FOLLOWER_HPP_
