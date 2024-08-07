// Copyright (c) 2024 Open Navigation LLC
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

#ifndef OPENNAV_DOCKING__NAVIGATOR_HPP_
#define OPENNAV_DOCKING__NAVIGATOR_HPP_

#include <vector>
#include <memory>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_util/node_utils.hpp"
#include "opennav_docking/utils.hpp"
#include "opennav_docking/types.hpp"

namespace opennav_docking
{
/**
 * @class opennav_docking::Navigator
 * @brief An object the call navigation to obtain initial staging pose
 */
class Navigator
{
public:
  using Nav2Pose = nav2_msgs::action::NavigateToPose;
  using ActionClient = rclcpp_action::Client<Nav2Pose>;

  /**
   * @brief A constructor for opennav_docking::Navigator
   * @param parent Weakptr to the node to use to get interances and parameters
   */
  explicit Navigator(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent);

  /**
   * @brief A destructor for opennav_docking::Navigator
   */
  ~Navigator() = default;

  /**
   * @brief An activation method
   */
  void activate();

  /**
   * @brief An deactivation method
   */
  void deactivate();

  /**
   * @brief An method to go to a particular pose
   * May throw exception if fails to navigate or communicate
   * Blocks until completion.
   * @param pose Pose to go to
   * @param remaining_staging_duration Remaining time to get to the staging pose
   * @param isPreempted Function to check if preempted
   * @param recursed True if recursed (used to retry once)
   */
  void goToPose(
    const geometry_msgs::msg::PoseStamped & pose,
    rclcpp::Duration remaining_staging_duration,
    std::function<bool()> isPreempted,
    bool recursed = false);

protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  ActionClient::SharedPtr nav_to_pose_client_;
  std::string navigator_bt_xml_;
};

}  // namespace opennav_docking

#endif  // OPENNAV_DOCKING__NAVIGATOR_HPP_
