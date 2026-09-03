// Copyright (c) 2019 Intel Corporation
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__RECOVERY_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__RECOVERY_NODE_HPP_

#include <limits>
#include <memory>
#include <string>

#include "behaviortree_cpp/control_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/bt_utils.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/tf2_factories.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{
/**
 * @brief The RecoveryNode has only two children and returns SUCCESS if and only if the first child
 * returns SUCCESS.
 *
 * - If the first child returns FAILURE, the second child will be executed.  After that the first
 * child is executed again if the second child returns SUCCESS.
 *
 * - If the first or second child returns RUNNING, this node returns RUNNING.
 *
 * - If the second child returns FAILURE, this control node will stop the loop and returns FAILURE.
 *
 * - Optional reset ports (both disabled by default):
 *   - reset_distance: resets retry_count_ to 0 once the robot has travelled this many metres
 *     since the last reset.  Useful in dynamic environments where obstacles move out of the way.
 *   - reset_time:     resets retry_count_ to 0 once this many seconds have elapsed since the last
 *     reset.  Useful for slow-moving or periodic obstructions.
 *
 * Usage in XML:
 * @code
 * <RecoveryNode number_of_retries="10" reset_distance="0.5">
 *     <!--Add tree components here-->
 * </RecoveryNode>
 * @endcode
 */
class RecoveryNode : public BT::ControlNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::RecoveryNode
   * @param name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  RecoveryNode(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief A destructor for nav2_behavior_tree::RecoveryNode
   */
  ~RecoveryNode() override = default;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("number_of_retries", 1, "Number of retries"),
      BT::InputPort<double>(
        "reset_distance", std::numeric_limits<double>::infinity(),
        "Reset the retry counter once the robot travels this distance (m). "
        "Disabled (infinity) by default."),
      BT::InputPort<double>(
        "reset_time", std::numeric_limits<double>::infinity(),
        "Reset the retry counter once this time has elapsed (s). "
        "Disabled (infinity) by default."),
      BT::InputPort<std::string>("global_frame", "map", "Global frame for TF lookups"),
      BT::InputPort<std::string>("robot_base_frame", "base_link", "Robot base frame"),
    };
  }

private:
  unsigned int current_child_idx_;
  unsigned int number_of_retries_;
  unsigned int retry_count_;

  // ---- reset_distance support ----
  nav2::LifecycleNode::SharedPtr node_;
  nav2::TransformBuffer::SharedPtr tf_;
  double transform_tolerance_;
  std::string global_frame_;
  std::string robot_base_frame_;
  geometry_msgs::msg::PoseStamped last_reset_pose_;
  double reset_distance_;

  // ---- reset_time support ----
  rclcpp::Time last_reset_time_;
  double reset_time_;

  // True from the first tick until a valid start pose has been captured
  bool first_tick_;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief The other (optional) override required by a BT action to reset node state
   */
  void halt() override;

  /**
   * @brief Check whether either the distance-travel or time-elapsed threshold has been crossed
   *        and, if so, reset retry_count_ and record the new baseline.
   */
  void checkAndResetCounterIfNeeded();
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__RECOVERY_NODE_HPP_
