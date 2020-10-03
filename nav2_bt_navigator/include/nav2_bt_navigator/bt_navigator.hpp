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

#ifndef NAV2_BT_NAVIGATOR__BT_NAVIGATOR_HPP_
#define NAV2_BT_NAVIGATOR__BT_NAVIGATOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_bt_navigator/ros_topic_logger.hpp"
#include "nav2_behavior_tree/bt_action_server.hpp"

namespace nav2_bt_navigator
{
/**
 * @class nav2_bt_navigator::BtNavigator
 * @brief An action server that uses behavior tree for navigating a robot to its
 * goal position.
 */
class BtNavigator : public nav2_behavior_tree::BtActionServer<nav2_msgs::action::NavigateToPose>
{
public:
  /**
   * @brief A constructor for nav2_bt_navigator::BtNavigator class
   */
  BtNavigator();
  /**
   * @brief A destructor for nav2_bt_navigator::BtNavigator class
   */
  ~BtNavigator();

protected:
  nav2_util::CallbackReturn on_configure() override;

  nav2_util::CallbackReturn on_cleanup() override;

  bool on_goal_received() override;

  void on_loop() override;

  bool is_canceling() override;

  /**
   * @brief A subscription and callback to handle the topic-based goal published
   * from rviz
   */
  void onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr pose);

  using Action = nav2_msgs::action::NavigateToPose;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

  // A client that we'll use to send a command message to our own task server
  rclcpp_action::Client<Action>::SharedPtr self_client_;

  // Metrics for feedback
  rclcpp::Time start_time_;
  std::string robot_frame_;
  std::string global_frame_;
  double transform_tolerance_;

  std::unique_ptr<RosTopicLogger> topic_logger_;

  std::shared_ptr<Action::Feedback> feedback_msg_;
};

}  // namespace nav2_bt_navigator

#endif  // NAV2_BT_NAVIGATOR__BT_NAVIGATOR_HPP_
