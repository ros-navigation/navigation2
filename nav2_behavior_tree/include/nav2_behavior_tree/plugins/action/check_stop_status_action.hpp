// Copyright (c) 2024 Angsa Robotics
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CHECK_STOP_STATUS_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CHECK_STOP_STATUS_ACTION_HPP_

#include <memory>
#include <string>

#include "nav2_ros_common/lifecycle_node.hpp"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/json_export.h"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_behavior_tree/bt_utils.hpp"
#include "nav2_behavior_tree/json_utils.hpp"

using namespace std::chrono_literals; // NOLINT

namespace nav2_behavior_tree
{

/**
 * @brief A nav2_behavior_tree::ActionNodeBase class that checks if the robot has been stopped for a specified duration
 */
class CheckStopStatus : public BT::ActionNodeBase
{
public:
  /**
    * @brief A constructor for nav2_behavior_tree::CheckStopStatus
    * @param xml_tag_name Name for the XML tag for this node
    * @param conf BT node configuration
    */
  CheckStopStatus(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    // Register JSON definitions for the types used in the ports
    BT::RegisterJsonDefinition<std::chrono::milliseconds>();

    return {
      BT::InputPort<double>(
        "velocity_threshold", 0.01,
        "Velocity threshold below which robot is considered stopped"),
      BT::InputPort<std::chrono::milliseconds>(
        "duration_stopped", 1000ms,
        "Duration (ms) the velocity must remain below the threshold"),
    };
  }

private:
  /**
   * @brief Override required by the a BT action. Cancel the action and set the path output
   */
  void halt() override {}
  /**
   * @brief Override required by the a BT action. Check if the robot has been stopped for the specified duration.
   * Return SUCCESS if stopped, FAILURE if not, and RUNNING if still within the duration threshold.
   */
  BT::NodeStatus tick() override;

  double velocity_threshold_;
  std::chrono::milliseconds duration_stopped_;
  rclcpp::Time stopped_stamp_;
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother_;
  nav2::LifecycleNode::SharedPtr node_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CHECK_STOP_STATUS_ACTION_HPP_
