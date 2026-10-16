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

#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/clock.hpp"
#include "nav2_ros_common/tf2_factories.hpp"
#include "behaviortree_cpp/control_node.h"

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
 * When reset_distance is positive, retries reset after the main child moves the robot that far.
 * Each main-child attempt starts a new reference; recovery motion does not count.
 *
 * Usage in XML:
 * @code
 * <RecoveryNode number_of_retries="1">
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
        "reset_distance", 0.0,
          "Main action displacement (m) to reset retries; non-positive disables"),
      BT::InputPort<std::string>("robot_base_frame", "Robot frame"),
      BT::InputPort<std::string>("global_frame", "odom", "Reference frame")
    };
  }

private:
  unsigned int current_child_idx_;
  unsigned int number_of_retries_;
  unsigned int retry_count_;
  double reset_distance_{0.0};
  double transform_staleness_threshold_{0.0};
  std::string robot_frame_, global_frame_;
  rclcpp::Clock::SharedPtr clock_;
  nav2::TransformBuffer::SharedPtr tf_;
  std::optional<geometry_msgs::msg::PoseStamped> reset_pose_;

  /**
   * @brief Initializes the execution
   */
  void initialize();

  /**
   * @brief Capture the reference pose before the main child executes if none is available.
   */
  void captureResetPose();
  /**
   * @brief Reset retries once the main child has made enough progress.
   */
  void checkRetryReset();

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief The other (optional) override required by a BT action to reset node state
   */
  void halt() override;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__RECOVERY_NODE_HPP_
