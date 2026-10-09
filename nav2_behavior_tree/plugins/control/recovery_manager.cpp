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
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/smooth_path.hpp"
#include "nav2_behavior_tree/plugins/control/recovery_manager.hpp"

namespace nav2_behavior_tree
{

namespace
{

// The errors of some of Nav2's actions
const std::unordered_map<uint16_t, std::string> & builtinErrorNames()
{
  using FollowPath = nav2_msgs::action::FollowPath::Result;
  using ComputePathToPose = nav2_msgs::action::ComputePathToPose::Result;
  using ComputePathThroughPoses = nav2_msgs::action::ComputePathThroughPoses::Result;
  using SmoothPath = nav2_msgs::action::SmoothPath::Result;
  static const std::unordered_map<uint16_t, std::string> builtin_error_names = {
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
  return builtin_error_names;
}

std::string toUppercase(std::string text)
{
  std::transform(text.begin(), text.end(), text.begin(), ::toupper);
  return text;
}

}  // namespace

RecoveryManager::RecoveryManager(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ControlNode(name, config)
{
  getInput("wrap_around", wrap_around_);

  node_ = config.blackboard->get<nav2::LifecycleNode::SharedPtr>("node");
  logger_ = node_->get_logger().get_child("RecoveryManager");
}

BT::NodeStatus RecoveryManager::tick()
{
  // Children are added after construction, which means we can load the sequences on the first tick
  if (!sequences_loaded_) {
    loadRecoverySequences();
  }

  const bool starting_new_recovery = status() != BT::NodeStatus::RUNNING;
  if (starting_new_recovery) {
    resetSequencesIfGoalChanged();
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
  // Every recovery behavior is a child node of this node
  for (std::size_t child_node_index = 0; child_node_index < children_nodes_.size();
    ++child_node_index)
  {
    const TreeNode * child_node = children_nodes_[child_node_index];
    if (!child_node_index_by_name_.emplace(child_node->name(), child_node_index).second) {
      throw BT::RuntimeError(
              "RecoveryManager: more than one recovery behavior is named '", child_node->name(),
              "'. Give each of them a unique name attribute.");
    }
  }

  std::string param_namespace;
  getInput("param_namespace", param_namespace);
  // The same prefixes the server reports the error codes of, e.g. follow_path for
  // follow_path_error_code
  std::vector<std::string> error_code_name_prefixes;
  if (!node_->get_parameter("error_code_name_prefixes", error_code_name_prefixes)) {
    throw BT::RuntimeError(
            "RecoveryManager: parameter 'error_code_name_prefixes' is not declared on the node");
  }

  // Only the error codes that have recovery sequences configured are recovered from
  for (const auto & error_code_name_prefix : error_code_name_prefixes) {
    auto group = loadErrorCodeGroup(error_code_name_prefix + "_error_code", param_namespace);
    if (group) {
      error_code_groups_.push_back(std::move(*group));
    }
  }
  if (error_code_groups_.empty()) {
    RCLCPP_WARN(
      logger_, "No recovery sequences are configured under '%s' for any error code",
      param_namespace.c_str());
  }

  // A child that no sequence refers to can never run
  std::vector<bool> child_node_used(children_nodes_.size(), false);
  for (const auto & group : error_code_groups_) {
    for (const std::size_t behavior_index : group.default_sequence) {
      child_node_used[behavior_index] = true;
    }
    for (const auto & [error_code, sequence] : group.sequence_by_error_code) {
      for (const std::size_t behavior_index : sequence) {
        child_node_used[behavior_index] = true;
      }
    }
  }
  for (std::size_t child_node_index = 0; child_node_index < children_nodes_.size();
    ++child_node_index)
  {
    if (!child_node_used[child_node_index]) {
      RCLCPP_WARN(
        logger_, "Recovery behavior %s is not in any recovery sequence and will never run",
        children_nodes_[child_node_index]->name().c_str());
    }
  }

  sequences_loaded_ = true;
}

std::optional<RecoveryManager::ErrorCodeGroup> RecoveryManager::loadErrorCodeGroup(
  const std::string & blackboard_key, const std::string & param_namespace)
{
  // e.g. recovery_manager.follow_path_error_code
  const std::string group_prefix =
    param_namespace.empty() ? blackboard_key : param_namespace + "." + blackboard_key;
  const auto parameters_for_this_group = getParametersUnder(group_prefix);
  if (parameters_for_this_group.empty()) {
    return std::nullopt;
  }

  ErrorCodeGroup group;
  group.blackboard_key = blackboard_key;
  group.error_names = builtinErrorNames();

  // Every child is used in order without a default
  for (std::size_t behavior_index = 0; behavior_index < children_nodes_.size(); ++behavior_index) {
    group.default_sequence.push_back(behavior_index);
  }

  // for e.g. changes "recovery_manager.follow_path_error_code.error_specific.tf_error" to
  // "error_specific.tf_error"
  const auto strip_parameter_prefix = [&](const std::string & param_name) {
      return param_name.substr(group_prefix.size() + 1);
    };

  const std::string error_names = "error_names.";
  const std::string error_specific = "error_specific.";

  // 1. Custom error names first so that the error specific sequences can use them
  std::unordered_map<std::string, uint16_t> custom_error_codes;
  for (const auto & [param_name, param_value] : parameters_for_this_group) {
    const std::string key = strip_parameter_prefix(param_name);
    if (!key.starts_with(error_names)) {
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
    const std::string error_name = toUppercase(key.substr(error_names.size()));
    custom_error_codes[error_name] = static_cast<uint16_t>(error_code);
    group.error_names[static_cast<uint16_t>(error_code)] = error_name;
  }

  // 2. The default and error specific sequences
  for (const auto & [param_name, param_value] : parameters_for_this_group) {
    const std::string key = strip_parameter_prefix(param_name);
    if (key == "default") {
      if (auto sequence = parseRecoverySequence(param_name, param_value)) {
        group.default_sequence = *sequence;
      }
    } else if (key.starts_with(error_specific)) {
      // A custom name only means its own code. Otherwise a Nav2 name can stand for several
      // codes, e.g. TF_ERROR of the planner and of the controller.
      const std::string error_name = toUppercase(key.substr(error_specific.size()));
      std::vector<uint16_t> error_codes;
      if (custom_error_codes.contains(error_name)) {
        error_codes.push_back(custom_error_codes.at(error_name));
      } else {
        for (const auto & [code, name] : builtinErrorNames()) {
          if (name == error_name) {
            error_codes.push_back(code);
          }
        }
      }
      if (error_codes.empty()) {
        // Still parsed so that a misspelled error name doesn't hide an unknown behavior name
        parseRecoverySequence(param_name, param_value);
        RCLCPP_WARN(logger_, "Ignoring parameter %s: unknown error", param_name.c_str());
        continue;
      }
      if (auto sequence = parseRecoverySequence(param_name, param_value)) {
        for (const uint16_t error_code : error_codes) {
          group.sequence_by_error_code[error_code] = *sequence;
        }
      }
    } else if (!key.starts_with(error_names)) {
      RCLCPP_WARN(logger_, "Ignoring parameter %s: unknown parameter", param_name.c_str());
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
    if (param_name.starts_with(prefix + ".")) {
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
    auto behavior = child_node_index_by_name_.find(behavior_name);
    if (behavior == child_node_index_by_name_.end()) {
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
  // 1. The first error code that is set is the one to recover from first
  ErrorCodeGroup * group_with_error = nullptr;
  uint16_t error_code = 0;
  for (auto & group : error_code_groups_) {
    if (config().blackboard->get(group.blackboard_key, error_code) && error_code != 0) {
      group_with_error = &group;
      break;
    }
  }
  if (group_with_error == nullptr) {
    RCLCPP_WARN(logger_, "No recovery configured for error code 0 (NONE)");
    return false;
  }

  const auto error_name = group_with_error->error_names.find(error_code);
  error_description_ = std::to_string(error_code) + " (" +
    (error_name != group_with_error->error_names.end() ?
    error_name->second : "UNKNOWN_ERROR_CODE") + ")";

  // 2. Its error specific sequence else the default one
  const auto specific_sequence = group_with_error->sequence_by_error_code.find(error_code);
  const RecoverySequence & sequence =
    specific_sequence != group_with_error->sequence_by_error_code.end() ?
    specific_sequence->second : group_with_error->default_sequence;
  if (sequence.empty()) {
    RCLCPP_WARN(logger_, "No recovery configured for error code %s", error_description_.c_str());
    return false;
  }

  // 3. The next behavior of that sequence
  std::size_t & next_behavior_index =
    group_with_error->next_behavior_index_by_error_code[error_code];
  if (next_behavior_index >= sequence.size()) {
    if (!wrap_around_) {
      RCLCPP_WARN(
        logger_, "Already executed all %zu recovery behaviors for error code %s",
        sequence.size(), error_description_.c_str());
      return false;
    }
    next_behavior_index = 0;
  }

  running_behavior_index_ = sequence[next_behavior_index];
  ++next_behavior_index;

  RCLCPP_INFO(
    logger_, "Executing recovery behavior %s (%zu/%zu) for error code %s",
    children_nodes_[*running_behavior_index_]->name().c_str(), next_behavior_index,
    sequence.size(), error_description_.c_str());
  return true;
}

void RecoveryManager::resetSequencesIfGoalChanged()
{
  geometry_msgs::msg::PoseStamped current_goal;
  nav_msgs::msg::Goals current_goals;
  BT::getInputOrBlackboard("goal", current_goal);
  BT::getInputOrBlackboard("goals", current_goals);

  if (current_goal != last_goal_ || current_goals != last_goals_) {
    last_goal_ = current_goal;
    last_goals_ = current_goals;
    resetAllSequences("goal changed");
  }
}

void RecoveryManager::resetAllSequences(const std::string & reason)
{
  const bool any_sequence_started = std::any_of(
    error_code_groups_.begin(), error_code_groups_.end(), [](const ErrorCodeGroup & group) {
      return !group.next_behavior_index_by_error_code.empty();
    });
  if (any_sequence_started) {
    RCLCPP_INFO(logger_, "Starting all recovery sequences over: %s", reason.c_str());
  }
  for (auto & group : error_code_groups_) {
    group.next_behavior_index_by_error_code.clear();
  }
}

}  // namespace nav2_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RecoveryManager>("RecoveryManager");
}
