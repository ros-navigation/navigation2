// Copyright (c) 2021 Samsung Research
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

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/navigator.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"

namespace nav2_bt_navigator
{

class NavigateToPoseNavigator : public nav2_core::Navigator<nav2_msgs::action::NavigateToPose>
{
public:
  using ActionT = nav2_msgs::action::NavigateToPose;

  NavigateToPoseNavigator() : Navigator() {};

  bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;
  bool cleanup() override;

  /**
   * @brief A subscription and callback to handle the topic-based goal published
   * from rviz
   */
  void onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr pose);

  /**
   * @brief Get action name for this navigator
   */
  std::string getName() {return std::string("navigate_to_pose");}

protected:

  /**
   * @brief A callback to be called when a new goal is received by the BT action server
   * Can be used to check if goal is valid and put values on
   * the blackboard which depend on the received goal
   */
  bool onGoalReceived(ActionT::Goal::ConstSharedPtr goal) override;

  /**
   * @brief A callback that defines execution that happens on one iteration through the BT
   * Can be used to publish action feedback
   */
  void onLoop() override;

  /**
   * @brief A callback that is called when a preempt is requested
   */
  void onPreempt() override;

  /**
   * @brief A callback that is called when a the action is completed, can fill in
   * action result message or indicate that this action is done.
   */
  void onCompletion(typename ActionT::Result::SharedPtr result) override;

  /**
   * @brief Goal pose initialization on the blackboard
   */
  void initializeGoalPose(ActionT::Goal::ConstSharedPtr goal);

  rclcpp::Time start_time_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp_action::Client<ActionT>::SharedPtr self_client_;

  std::string goal_blackboard_id_;
};

}  // namespace nav2_bt_navigator
