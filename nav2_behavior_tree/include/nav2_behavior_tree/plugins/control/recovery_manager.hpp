// Copyright (c) 2026 Nisarg Panchal
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__RECOVERY_MANAGER_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__RECOVERY_MANAGER_HPP_

#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "behaviortree_cpp/control_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/goals.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/tf2_factories.hpp"
#include "nav2_behavior_tree/bt_utils.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief Runs one recovery behavior each time navigation fails. Which one is picked from
 * a sequence per error code, configured with parameters instead of in the tree.
 *
 * The children are the available recovery behaviors, referred to by their name. Each
 * blackboard key in `error_code_names` gets its own group of sequences, named after the key:
 *
 * @code{.yaml}
 * recovery_manager:
 *   compute_path_error_code:
 *     default: [ClearGlobalCostmap, Wait, ClearGlobalCostmap]
 *     error_specific:
 *       start_occupied: [ClearGlobalCostmap, BackUp]
 *       goal_occupied: [none]
 *   follow_path_error_code:
 *     default: [ClearLocalCostmap, Wait, ClearLocalCostmap]
 *     error_specific:
 *       tf_error: [Wait]
 *   my_action_error_code:
 *     error_names: {MY_FAILURE: 950}
 *     error_specific:
 *       my_failure: [BackUp]
 * @endcode
 *
 * Errors are given by name, and `[none]` means that nothing can be done about them.
 * A group without a default uses all children in order. Custom error codes are named in
 * `error_names`, and those names only apply within their group.
 *
 * Every error code walks through its own sequence, one behavior per failure, even when that
 * behavior fails. Once the sequence runs out this node returns FAILURE. Sequences start over
 * when the goal changes, when a running recovery is halted, or once the robot has moved
 * `reset_distance` since the last recovery.
 *
 * Usage in XML:
 * @code
 * <RecoveryNode number_of_retries="-1">
 *   <!--navigation-->
 *   <RecoveryManager param_namespace="recovery_manager">
 *     <ClearEntireCostmap name="ClearLocalCostmap" service_name="..."/>
 *     <Wait wait_duration="5.0"/>
 *     <BackUp backup_dist="0.30" backup_speed="0.15"/>
 *   </RecoveryManager>
 * </RecoveryNode>
 * @endcode
 */
class RecoveryManager : public BT::ControlNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::RecoveryManager
   * @param name Name for the XML tag for this node
   * @param config BT node configuration
   */
  RecoveryManager(const std::string & name, const BT::NodeConfiguration & config);

  /**
   * @brief Starts the next recovery behavior or keeps ticking the running one
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Halts the running behavior
   */
  void halt() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "param_namespace", "recovery_manager", "Parameter namespace of the recovery sequences"),
      BT::InputPort<std::vector<std::string>>(
        "error_code_names", "compute_path_error_code;follow_path_error_code",
        "Blackboard keys of the error codes to recover from, in order of priority"),
      BT::InputPort<double>(
        "reset_distance", 0.5,
        "Distance the robot has to move after a recovery for the sequences to start over, "
        "zero to disable"),
      BT::InputPort<bool>(
        "wrap_around", false, "Start a sequence over instead of failing once it runs out"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination"),
      BT::InputPort<nav_msgs::msg::Goals>("goals", "Destinations"),
      BT::InputPort<std::string>("global_frame", "Global frame"),
      BT::InputPort<std::string>("robot_base_frame", "Robot base frame"),
    };
  }

private:
  // Indices of the children to run, in order
  using RecoverySequence = std::vector<std::size_t>;

  struct ErrorCodeGroup
  {
    std::string blackboard_key;
    // Uppercase error names by code: Nav2's, plus the custom ones from error_names
    std::unordered_map<uint16_t, std::string> error_names;
    RecoverySequence default_sequence;
    std::unordered_map<uint16_t, RecoverySequence> sequence_by_error_code;
  };

  void loadRecoverySequences();
  ErrorCodeGroup loadErrorCodeGroup(
    const std::string & blackboard_key, const std::string & param_namespace);
  std::map<std::string, rclcpp::ParameterValue> getParametersUnder(const std::string & prefix);
  std::optional<RecoverySequence> parseRecoverySequence(
    const std::string & param_name, const rclcpp::ParameterValue & param_value);

  bool selectNextRecoveryBehavior();
  void resetSequencesIfGoalChangedOrRobotMoved();
  void resetAllSequences(const std::string & reason);
  std::optional<geometry_msgs::msg::PoseStamped> getRobotPose();

  nav2::LifecycleNode::SharedPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("RecoveryManager")};
  nav2::TransformBuffer::SharedPtr tf_buffer_;
  std::string global_frame_;
  std::string robot_base_frame_;
  double transform_tolerance_{0.1};
  double reset_distance_{0.5};
  bool wrap_around_{false};

  bool sequences_loaded_{false};
  std::unordered_map<std::string, std::size_t> behavior_index_by_name_;
  std::vector<ErrorCodeGroup> error_code_groups_;
  std::unordered_map<uint16_t, std::size_t> next_behavior_index_by_error_code_;

  std::string error_description_;
  std::optional<std::size_t> running_behavior_index_;

  geometry_msgs::msg::PoseStamped last_goal_;
  nav_msgs::msg::Goals last_goals_;
  std::optional<geometry_msgs::msg::PoseStamped> pose_after_last_recovery_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__RECOVERY_MANAGER_HPP_
