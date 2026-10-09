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
 * @brief Runs one recovery behavior each time navigation fails. There is a sequence
 * configured per error code and it follows that.
 *
 * The children are the available recovery behaviors which are referred to by their name.
 * Unlike a Sequence or a RoundRobin, this node does not tick its children in the order they
 * are written in the XML. Which child runs, and when, is decided only by the configured
 * sequences: on each failure, the error code that was set picks its sequence and the next
 * behavior in it is run. Reordering the children in the XML has no effect on execution,
 * except for a group with no `default` sequence (see below).
 *
 * Each blackboard key in `error_code_names` gets its own group of sequences which is named
 * after the key:
 *
 * @code{.yaml}
 * bt_navigator:
 *   ros__parameters:
 *     recovery_manager:
 *       # compute_path_error_code is the blackboard key ComputePathToPose stores its error
 *       # code in
 *       compute_path_error_code:
 *         default: ["ClearGlobalCostmap", "Wait", "ClearGlobalCostmap"]
 *         error_specific:
 *           invalid_planner: ["none"]
 *           tf_error: ["Wait"]
 *           start_outside_map: ["none"]
 *           goal_outside_map: ["none"]
 *           start_occupied: ["ClearGlobalCostmap", "BackUp"]
 *           goal_occupied: ["none"]
 *       # follow_path_error_code is the blackboard key FollowPath stores its error code in
 *       follow_path_error_code:
 *         default: ["ClearLocalCostmap", "Wait", "ClearLocalCostmap"]
 *         error_specific:
 *           invalid_controller: ["none"]
 *           tf_error: ["Wait"]
 *           invalid_path: ["none"]
 *           failed_to_make_progress: ["ClearLocalCostmap", "BackUp", "Spin"]
 *           no_valid_control: ["ClearLocalCostmap", "BackUp"]
 *       # A custom action's error codes are named in error_names. Its blackboard key also
 *       # has to be added to the error_code_names port
 *       my_action_error_code:
 *         error_names: {MY_FAILURE: 950}
 *         default: ["Wait"]
 *         error_specific:
 *           my_failure: ["BackUp"]
 * @endcode
 *
 * Errors are given by name, and `[none]` means that nothing can be done about them.
 * A group without a default uses all children in their XML order; this is the only case where
 * the order of the children matters. Custom error codes are named in
 * `error_names`, and those names only apply within their group.
 *
 * Every error code of every group walks through its own sequence. Once the sequence runs out this node
 * returns FAILURE. Sequences start over when the goal changes, when a running recovery is
 * halted, or once the robot has moved `reset_distance` since the last recovery.
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
 *
 * A ready to use subtree is in nav2_bt_navigator/behavior_trees/subtrees/recovery_manager.xml.
 */
class RecoveryManager : public BT::ControlNode
{
public:
  /**
   * @brief Constructor for nav2_behavior_tree::RecoveryManager
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
   * @brief Halts the current running behavior
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
    // How far each error code has got through its sequence
    std::unordered_map<uint16_t, std::size_t> next_behavior_index_by_error_code;
  };

  /**
   * @brief Indexes the children nodes by name and loads a group of sequences for each key in
   * error_code_names. Warns about children that no sequence refers to
   * @throw BT::RuntimeError If two children share a name, or a sequence names an unknown child
   */
  void loadRecoverySequences();

  /**
   * @brief Loads the custom error names, default sequence and error specific sequences of
   * one error code group from the parameters
   * @param blackboard_key Blackboard key holding the error code which is also the name of the group
   * @param param_namespace Parameter namespace under which group is stored
   * @return ErrorCodeGroup The loaded group
   */
  ErrorCodeGroup loadErrorCodeGroup(
    const std::string & blackboard_key, const std::string & param_namespace);

  /**
   * @brief Gets the parameters under a prefix.
   * For e.g. the prefix "recovery_manager.follow_path_error_code" gets parameters like
   * "recovery_manager.follow_path_error_code.default" and
   * "recovery_manager.follow_path_error_code.error_specific.tf_error"
   * @param prefix Parameter name prefix, without the trailing dot
   * @return Parameter values by their full name
   */
  std::map<std::string, rclcpp::ParameterValue> getParametersUnder(const std::string & prefix);

  /**
   * @brief Declares a sequence parameter and turns its behavior names into child indices.
   * "none" entries are skipped
   * @param param_name Full name of the parameter
   * @param param_value Value of the parameter, expected to be a list of strings
   * @return RecoverySequence The child indices, empty for an empty list, or std::nullopt if
   * the value is not a list of strings
   * @throw BT::RuntimeError If a name is not one of the children
   */
  std::optional<RecoverySequence> parseRecoverySequence(
    const std::string & param_name, const rclcpp::ParameterValue & param_value);

  /**
   * @brief Picks the next behavior from its error specific sequence or else the group's
   * default sequence for the first error code that is set. Sets running_behavior_index_
   * @return bool False if there is nothing to run: no error code is set, the sequence is
   * empty, or it ran out and wrap_around is off
   */
  bool selectNextRecoveryBehavior();

  /**
   * @brief Starts all sequences over if the goal changed, or if the robot has moved at least
   * reset_distance since the last recovery
   */
  void resetSequencesIfGoalChangedOrRobotMoved();

  /**
   * @brief Starts all sequences over from their first behavior
   * @param reason Why they start over, for logging
   */
  void resetAllSequences(const std::string & reason);

  /**
   * @brief Gets the robot pose in the global frame
   * @return The robot pose, or std::nullopt if the transform is not available
   */
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
  std::unordered_map<std::string, std::size_t> child_node_index_by_name_;
  std::vector<ErrorCodeGroup> error_code_groups_;

  std::string error_description_;
  std::optional<std::size_t> running_behavior_index_;

  geometry_msgs::msg::PoseStamped last_goal_;
  nav_msgs::msg::Goals last_goals_;
  std::optional<geometry_msgs::msg::PoseStamped> pose_after_last_recovery_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__RECOVERY_MANAGER_HPP_
