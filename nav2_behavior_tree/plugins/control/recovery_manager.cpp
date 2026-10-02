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

#include <algorithm>
#include <cctype>
#include <limits>
#include <map>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/smooth_path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_behavior_tree/plugins/control/recovery_manager.hpp"

namespace nav2_behavior_tree
{

namespace
{

const std::vector<std::pair<uint16_t, std::string>> & errorCodeNames()
{
  using FollowPath = nav2_msgs::action::FollowPath::Result;
  using ComputePathToPose = nav2_msgs::action::ComputePathToPose::Result;
  using ComputePathThroughPoses = nav2_msgs::action::ComputePathThroughPoses::Result;
  using SmoothPath = nav2_msgs::action::SmoothPath::Result;
  static const std::vector<std::pair<uint16_t, std::string>> error_code_names = {
    {FollowPath::NONE, "NONE"},
    {FollowPath::GOAL_REJECTED, "GOAL_REJECTED"},
    {FollowPath::SEND_GOAL_FAILURE, "SEND_GOAL_FAILURE"},
    {FollowPath::UNKNOWN, "UNKNOWN"},
    {FollowPath::INVALID_CONTROLLER, "INVALID_CONTROLLER"},
    {FollowPath::TF_ERROR, "TF_ERROR"},
    {FollowPath::INVALID_PATH, "INVALID_PATH"},
    {FollowPath::PATIENCE_EXCEEDED, "PATIENCE_EXCEEDED"},
    {FollowPath::FAILED_TO_MAKE_PROGRESS, "FAILED_TO_MAKE_PROGRESS"},
    {FollowPath::NO_VALID_CONTROL, "NO_VALID_CONTROL"},
    {FollowPath::CONTROLLER_TIMED_OUT, "CONTROLLER_TIMED_OUT"},
    {FollowPath::TIMEOUT, "TIMEOUT"},
    {ComputePathToPose::UNKNOWN, "UNKNOWN"},
    {ComputePathToPose::INVALID_PLANNER, "INVALID_PLANNER"},
    {ComputePathToPose::TF_ERROR, "TF_ERROR"},
    {ComputePathToPose::START_OUTSIDE_MAP, "START_OUTSIDE_MAP"},
    {ComputePathToPose::GOAL_OUTSIDE_MAP, "GOAL_OUTSIDE_MAP"},
    {ComputePathToPose::START_OCCUPIED, "START_OCCUPIED"},
    {ComputePathToPose::GOAL_OCCUPIED, "GOAL_OCCUPIED"},
    {ComputePathToPose::TIMEOUT, "TIMEOUT"},
    {ComputePathToPose::NO_VALID_PATH, "NO_VALID_PATH"},
    {ComputePathThroughPoses::UNKNOWN, "UNKNOWN"},
    {ComputePathThroughPoses::INVALID_PLANNER, "INVALID_PLANNER"},
    {ComputePathThroughPoses::TF_ERROR, "TF_ERROR"},
    {ComputePathThroughPoses::START_OUTSIDE_MAP, "START_OUTSIDE_MAP"},
    {ComputePathThroughPoses::GOAL_OUTSIDE_MAP, "GOAL_OUTSIDE_MAP"},
    {ComputePathThroughPoses::START_OCCUPIED, "START_OCCUPIED"},
    {ComputePathThroughPoses::GOAL_OCCUPIED, "GOAL_OCCUPIED"},
    {ComputePathThroughPoses::TIMEOUT, "TIMEOUT"},
    {ComputePathThroughPoses::NO_VALID_PATH, "NO_VALID_PATH"},
    {ComputePathThroughPoses::NO_VIAPOINTS_GIVEN, "NO_VIAPOINTS_GIVEN"},
    {SmoothPath::UNKNOWN, "UNKNOWN"},
    {SmoothPath::INVALID_SMOOTHER, "INVALID_SMOOTHER"},
    {SmoothPath::TIMEOUT, "TIMEOUT"},
    {SmoothPath::SMOOTHED_PATH_IN_COLLISION, "SMOOTHED_PATH_IN_COLLISION"},
    {SmoothPath::FAILED_TO_SMOOTH_PATH, "FAILED_TO_SMOOTH_PATH"},
    {SmoothPath::INVALID_PATH, "INVALID_PATH"},
  };
  return error_code_names;
}

using CustomErrorCodes = std::unordered_map<std::string, uint16_t>;

std::string toUppercase(std::string text)
{
  std::transform(text.begin(), text.end(), text.begin(), ::toupper);
  return text;
}

// e.g. "105 (FAILED_TO_MAKE_PROGRESS)"
std::string describeErrorCode(
  const uint16_t error_code, const CustomErrorCodes & custom_error_codes = {})
{
  for (const auto & [name, code] : custom_error_codes) {
    if (code == error_code) {
      return std::to_string(error_code) + " (" + name + ")";
    }
  }
  std::string error_name = "UNKNOWN_ERROR_CODE";
  for (const auto & [code, name] : errorCodeNames()) {
    if (code == error_code) {
      error_name = name;
      break;
    }
  }
  return std::to_string(error_code) + " (" + error_name + ")";
}

// A name like "tf_error" matches the code of every action with that error, unless it is the
// name of a custom error code
std::vector<uint16_t> errorCodesForKey(
  const std::string & error_key, const CustomErrorCodes & custom_error_codes)
{
  const std::string uppercase_key = toUppercase(error_key);
  const auto custom_error_code = custom_error_codes.find(uppercase_key);
  if (custom_error_code != custom_error_codes.end()) {
    return {custom_error_code->second};
  }

  std::vector<uint16_t> matching_codes;
  for (const auto & [code, name] : errorCodeNames()) {
    if (name == uppercase_key) {
      matching_codes.push_back(code);
    }
  }
  return matching_codes;
}

bool startsWith(const std::string & text, const std::string & prefix)
{
  return text.rfind(prefix, 0) == 0;
}

}  // namespace

RecoveryManager::RecoveryManager(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ControlNode(name, config)
{
  getInput("reset_distance", reset_distance_);
  getInput("wrap_around", wrap_around_);

  node_ = config.blackboard->get<nav2::LifecycleNode::SharedPtr>("node");
  logger_ = node_->get_logger().get_child("RecoveryManager");

  if (reset_distance_ > 0.0) {
    tf_buffer_ = config.blackboard->get<nav2::TransformBuffer::SharedPtr>("tf_buffer");
    node_->get_parameter("transform_tolerance", transform_tolerance_);
    global_frame_ = BT::deconflictPortAndParamFrame<std::string>(node_, "global_frame", this);
    robot_base_frame_ = BT::deconflictPortAndParamFrame<std::string>(
      node_, "robot_base_frame", this);
  }
}

BT::NodeStatus RecoveryManager::tick()
{
  // Children are only added after construction, so the sequences are loaded on the first tick
  if (!sequences_loaded_) {
    loadRecoverySequences();
  }

  const bool starting_new_recovery = status() != BT::NodeStatus::RUNNING;
  if (starting_new_recovery) {
    resetSequencesIfGoalChangedOrRobotMoved();
    if (!selectNextRecoveryBehavior()) {
      return BT::NodeStatus::FAILURE;
    }
  }

  setStatus(BT::NodeStatus::RUNNING);

  TreeNode * behavior = children_nodes_[*running_behavior_index_];
  const BT::NodeStatus behavior_status = behavior->executeTick();

  if (behavior_status == BT::NodeStatus::RUNNING) {
    return BT::NodeStatus::RUNNING;
  }
  if (behavior_status == BT::NodeStatus::IDLE) {
    throw BT::LogicError("A child node must never return IDLE");
  }

  if (behavior_status != BT::NodeStatus::SUCCESS) {
    RCLCPP_WARN(
      logger_, "Recovery behavior %s failed for error code %s", behavior->name().c_str(),
      error_description_.c_str());
  }

  haltChild(*running_behavior_index_);
  running_behavior_index_.reset();

  // Taken after the behavior, so that its own motion (e.g. backing up) isn't counted as progress
  if (reset_distance_ > 0.0) {
    pose_after_last_recovery_ = getRobotPose();
  }

  // Even a failed behavior had its turn, so navigation is retried before the next one
  return BT::NodeStatus::SUCCESS;
}

void RecoveryManager::halt()
{
  // Halted mid recovery means navigation was cancelled or preempted
  const bool recovery_was_running =
    status() == BT::NodeStatus::RUNNING && running_behavior_index_.has_value();
  if (recovery_was_running) {
    resetAllSequences("recovery was halted");
  }

  ControlNode::halt();
  running_behavior_index_.reset();
}

void RecoveryManager::loadRecoverySequences()
{
  for (std::size_t behavior_index = 0; behavior_index < children_nodes_.size(); ++behavior_index) {
    const TreeNode * behavior = children_nodes_[behavior_index];
    if (!behavior_index_by_name_.emplace(behavior->name(), behavior_index).second) {
      throw BT::RuntimeError(
              "RecoveryManager: more than one recovery behavior is named '", behavior->name(),
              "'. Give each of them a unique name attribute.");
    }
  }

  std::string param_namespace;
  getInput("param_namespace", param_namespace);
  std::vector<std::string> error_code_names;
  getInput("error_code_names", error_code_names);

  for (const auto & blackboard_key : error_code_names) {
    error_code_groups_.push_back(loadErrorCodeGroup(blackboard_key, param_namespace));
  }

  sequences_loaded_ = true;
}

RecoveryManager::ErrorCodeGroup RecoveryManager::loadErrorCodeGroup(
  const std::string & blackboard_key, const std::string & param_namespace)
{
  ErrorCodeGroup group;
  group.blackboard_key = blackboard_key;
  group.name = blackboard_key;

  for (std::size_t behavior_index = 0; behavior_index < children_nodes_.size(); ++behavior_index) {
    group.default_sequence.push_back(behavior_index);
  }

  const std::string group_prefix =
    param_namespace.empty() ? group.name : param_namespace + "." + group.name;
  const std::string default_param_name = group_prefix + ".default";
  const std::string error_names_prefix = group_prefix + ".error_names.";
  const std::string error_specific_prefix = group_prefix + ".error_specific.";

  const auto parameters = getParametersUnder(group_prefix);

  // Custom error codes are named first, so that the error specific sequences can use the names
  for (const auto & [param_name, param_value] : parameters) {
    if (!startsWith(param_name, error_names_prefix)) {
      continue;
    }
    const int64_t error_code =
      param_value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER ?
      node_->declare_or_get_parameter(param_name, param_value.get<int64_t>()) : 0;
    if (error_code <= 0 || error_code > std::numeric_limits<uint16_t>::max()) {
      RCLCPP_WARN(
        logger_, "Ignoring parameter %s: not an error code from 1 to 65535", param_name.c_str());
      continue;
    }
    group.custom_error_codes[toUppercase(param_name.substr(error_names_prefix.size()))] =
      static_cast<uint16_t>(error_code);
  }

  for (const auto & [param_name, param_value] : parameters) {
    if (startsWith(param_name, error_names_prefix)) {
      continue;
    }

    if (param_name == default_param_name) {
      if (auto sequence = parseRecoverySequence(param_name, param_value)) {
        group.default_sequence = *sequence;
      }
      continue;
    }

    if (!startsWith(param_name, error_specific_prefix)) {
      RCLCPP_WARN(logger_, "Ignoring parameter %s: unknown parameter", param_name.c_str());
      continue;
    }

    const std::string error_key = param_name.substr(error_specific_prefix.size());
    const std::vector<uint16_t> error_codes =
      errorCodesForKey(error_key, group.custom_error_codes);
    if (error_codes.empty()) {
      RCLCPP_WARN(logger_, "Ignoring parameter %s: unknown error", param_name.c_str());
      continue;
    }

    if (auto sequence = parseRecoverySequence(param_name, param_value)) {
      for (const uint16_t error_code : error_codes) {
        group.sequence_by_error_code[error_code] = *sequence;
      }
    }
  }

  return group;
}

std::map<std::string, rclcpp::ParameterValue> RecoveryManager::getParametersUnder(
  const std::string & prefix)
{
  // Errors are named freely by the user, so their parameters can't be declared up front and
  // are looked up in the parameter file instead
  std::map<std::string, rclcpp::ParameterValue> parameters;
  const auto parameters_interface = node_->get_node_parameters_interface();
  for (const auto & [param_name, param_value] : parameters_interface->get_parameter_overrides()) {
    if (startsWith(param_name, prefix + ".")) {
      parameters[param_name] = param_value;
    }
  }

  const auto declared_parameters = node_->list_parameters(
    {prefix}, rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE);
  for (const auto & param_name : declared_parameters.names) {
    parameters[param_name] = node_->get_parameter(param_name).get_parameter_value();
  }
  return parameters;
}

std::optional<RecoveryManager::RecoverySequence> RecoveryManager::parseRecoverySequence(
  const std::string & param_name, const rclcpp::ParameterValue & param_value)
{
  // An empty list in the parameter file arrives as an unset parameter
  if (param_value.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
    return RecoverySequence{};
  }

  if (param_value.get_type() != rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
    RCLCPP_WARN(logger_, "Ignoring parameter %s: not a list of names", param_name.c_str());
    return std::nullopt;
  }

  const auto behavior_names = node_->declare_or_get_parameter(
    param_name, param_value.get<std::vector<std::string>>());

  RecoverySequence sequence;
  for (const auto & behavior_name : behavior_names) {
    if (behavior_name == "none") {
      continue;
    }
    auto behavior = behavior_index_by_name_.find(behavior_name);
    if (behavior == behavior_index_by_name_.end()) {
      throw BT::RuntimeError(
              "RecoveryManager: recovery behavior '", behavior_name, "' in parameter '",
              param_name, "' is not a child of '", name(), "'");
    }
    sequence.push_back(behavior->second);
  }
  return sequence;
}

bool RecoveryManager::selectNextRecoveryBehavior()
{
  const ErrorCodeGroup * failed_group = nullptr;
  error_code_being_recovered_ = 0;
  for (const auto & group : error_code_groups_) {
    uint16_t error_code = 0;
    if (config().blackboard->get(group.blackboard_key, error_code) && error_code != 0) {
      failed_group = &group;
      error_code_being_recovered_ = error_code;
      break;
    }
  }

  RecoverySequence sequence;
  if (failed_group != nullptr) {
    const auto specific_sequence =
      failed_group->sequence_by_error_code.find(error_code_being_recovered_);
    sequence = specific_sequence != failed_group->sequence_by_error_code.end() ?
      specific_sequence->second : failed_group->default_sequence;
  }

  error_description_ = failed_group != nullptr ?
    describeErrorCode(error_code_being_recovered_, failed_group->custom_error_codes) :
    describeErrorCode(error_code_being_recovered_);
  const std::string & error_description = error_description_;
  if (sequence.empty()) {
    RCLCPP_WARN(logger_, "No recovery configured for error code %s", error_description.c_str());
    return false;
  }

  std::size_t & next_behavior_index =
    next_behavior_index_by_error_code_[error_code_being_recovered_];
  if (next_behavior_index >= sequence.size()) {
    if (!wrap_around_) {
      RCLCPP_WARN(
        logger_, "Already executed all %zu recovery behaviors for error code %s",
        sequence.size(), error_description.c_str());
      return false;
    }
    next_behavior_index = 0;
  }

  running_behavior_index_ = sequence[next_behavior_index];
  ++next_behavior_index;

  RCLCPP_INFO(
    logger_, "Executing recovery behavior %s (%zu/%zu) for error code %s",
    children_nodes_[*running_behavior_index_]->name().c_str(), next_behavior_index,
    sequence.size(), error_description.c_str());
  return true;
}

void RecoveryManager::resetSequencesIfGoalChangedOrRobotMoved()
{
  geometry_msgs::msg::PoseStamped current_goal;
  nav_msgs::msg::Goals current_goals;
  BT::getInputOrBlackboard("goal", current_goal);
  BT::getInputOrBlackboard("goals", current_goals);

  if (current_goal != last_goal_ || current_goals != last_goals_) {
    last_goal_ = current_goal;
    last_goals_ = current_goals;
    resetAllSequences("goal changed");
    return;
  }

  if (reset_distance_ <= 0.0 || !pose_after_last_recovery_) {
    return;
  }

  const auto robot_pose = getRobotPose();
  if (!robot_pose) {
    return;
  }

  const double distance_moved = nav2_util::geometry_utils::euclidean_distance(
    pose_after_last_recovery_->pose, robot_pose->pose);
  if (distance_moved >= reset_distance_) {
    resetAllSequences("robot moved on since the last recovery");
  }
}

void RecoveryManager::resetAllSequences(const std::string & reason)
{
  if (!next_behavior_index_by_error_code_.empty()) {
    RCLCPP_INFO(logger_, "Starting all recovery sequences over: %s", reason.c_str());
  }
  next_behavior_index_by_error_code_.clear();
  pose_after_last_recovery_.reset();
}

std::optional<geometry_msgs::msg::PoseStamped> RecoveryManager::getRobotPose()
{
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!nav2_util::getCurrentPose(
      robot_pose, *tf_buffer_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    return std::nullopt;
  }
  return robot_pose;
}

}  // namespace nav2_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RecoveryManager>("RecoveryManager");
}
