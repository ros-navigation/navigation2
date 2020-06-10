// Copyright (c) 2018 Intel Corporation
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

#ifndef NAV2_BT_NAVIGATOR__FOLLOW_WAYPOINTS_BT_NAVIGATOR_HPP_
#define NAV2_BT_NAVIGATOR__FOLLOW_WAYPOINTS_BT_NAVIGATOR_HPP_

#include "nav2_bt_navigator/bt_navigator.hpp"

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace nav2_bt_navigator
{

class FollowWaypointsBtNavigator : public BtNavigatorBase
{
public:
  /**
   * @brief A constructor for nav2_bt_navigator::FollowWaypointsBtNavigator class
   */
  FollowWaypointsBtNavigator();
  /**
   * @brief A destructor for nav2_bt_navigator::FollowWaypointsBtNavigator class
   */
  ~FollowWaypointsBtNavigator() = default;

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
   * @brief Action server callbacks
   */
  void actionCallback();

  /**
   * @brief Goal pose initialization on the blackboard
   */
  void initializeGoalPose();

  using Action = nav2_msgs::action::FollowWaypoints;

  using ActionServer = nav2_util::SimpleActionServer<Action>;

  // Our action server implements the FollowWaypoints action
  std::unique_ptr<ActionServer> action_server_;
};

}  // namespace nav2_bt_navigator

#endif  // NAV2_BT_NAVIGATOR__FOLLOW_WAYPOINTS_BT_NAVIGATOR_HPP_
