// Copyright (c) 2020 Samsung Research America
// Copyright (c) 2020 ymd-stella
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

#ifndef NAV2_BT_WAYPOINT_FOLLOWER__BT_WAYPOINT_FOLLOWER_HPP_
#define NAV2_BT_WAYPOINT_FOLLOWER__BT_WAYPOINT_FOLLOWER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/behavior_tree_engine.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"

namespace nav2_bt_waypoint_follower
{

/**
 * @class nav2_bt_waypoint_follower::BtWaypointFollower
 * @brief WIP
 */
class BtWaypointFollower : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief A constructor for nav2_bt_waypoint_follower::BtWaypointFollower class
   */
  explicit BtWaypointFollower(bool use_bond = true);
  /**
   * @brief A destructor for nav2_bt_waypoint_follower::BtWaypointFollower class
   */
  ~BtWaypointFollower();

protected:
  /**
   * @brief Configures member variables
   *
   * Initializes action server for "NavigationToPose"; subscription to
   * "goal_sub"; and builds behavior tree from xml file.
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

  using Action = nav2_msgs::action::FollowWaypoints;

  using ActionServer = nav2_util::SimpleActionServer<Action>;

  // Our action server implements the FollowWaypoints action
  std::unique_ptr<ActionServer> action_server_;

  /**
   * @brief Action server callbacks
   */
  void followWaypoints();

  /**
   * @brief Initialize blackboard
   */
  void initializeBlackboard(std::shared_ptr<const Action::Goal> goal);

  /**
   * @brief Replace current BT with another one
   * @param bt_xml_filename The file containing the new BT
   * @return true if the resulting BT correspond to the one in bt_xml_filename. false
   * if something went wrong, and previous BT is mantained
   */
  bool loadBehaviorTree(const std::string & bt_id);

  BT::Tree tree_;

  // The blackboard shared by all of the nodes in the tree
  BT::Blackboard::Ptr blackboard_;

  // The XML string that defines the Behavior Tree to create
  std::string xml_string_;

  // The wrapper class for the BT functionality
  std::unique_ptr<nav2_behavior_tree::BehaviorTreeEngine> bt_;

  // Libraries to pull plugins (BT Nodes) from
  std::vector<std::string> plugin_lib_names_;

  // A regular, non-spinning ROS node that we can use for calls to the action client
  rclcpp::Node::SharedPtr client_node_;

  // The parameters for test
  // Whether or not to use bond
  bool use_bond_;
};

}  // namespace nav2_bt_waypoint_follower

#endif  // NAV2_BT_WAYPOINT_FOLLOWER__BT_WAYPOINT_FOLLOWER_HPP_
