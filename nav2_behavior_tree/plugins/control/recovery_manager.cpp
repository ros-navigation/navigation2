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
#include <cstdlib>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "nav2_behavior_tree/plugins/control/recovery_manager.hpp"

namespace nav2_behavior_tree
{

RecoveryManager::RecoveryManager(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ControlNode(name, config)
{
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
    getInput("wrap_around", wrap_around_);
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

  const bool behavior_failed = behavior_status != BT::NodeStatus::SUCCESS;
  const bool abort_recovery = behavior_failed && abort_on_failure_[*running_behavior_index_];
  if (behavior_failed) {
    RCLCPP_WARN(
      logger_, "Recovery behavior %s failed for %s%s", behavior->name().c_str(),
      error_description_.c_str(), abort_recovery ? ", aborting recovery" : "");
  }

  ControlNode::haltChildren();
  running_behavior_index_.reset();

  if (abort_recovery) {
    return BT::NodeStatus::FAILURE;
  }

  // Any other failed behavior still had its turn, so navigation is retried before the next one
  return BT::NodeStatus::SUCCESS;
}

void RecoveryManager::halt()
{
  // A finished recovery is not halted by its parent, so this means the recovery was
  // preempted, or the tree was halted at the end of the navigation
  resetAllSequences("recovery manager was halted");

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

  // A failure of behaviors with abort_on_failure fails the recovery, the failures of the others
  // are ignored
  const std::string abort_on_failure_param =
    param_namespace.empty() ? "abort_on_failure" : param_namespace + ".abort_on_failure";
  abort_on_failure_.assign(children_nodes_.size(), false);
  if (auto behaviors = parseRecoverySequence(
      abort_on_failure_param, getParameterValue(abort_on_failure_param)))
  {
    for (const std::size_t behavior_index : *behaviors) {
      abort_on_failure_[behavior_index] = true;
    }
  }
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

  const std::string error_specific = "error_specific.";
  for (const auto & [param_name, param_value] : parameters_for_this_group) {
    // e.g. "error_specific.105" of "recovery_manager.follow_path_error_code.error_specific.105"
    const std::string key = param_name.substr(group_prefix.size() + 1);
    if (key == "default") {
      if (auto sequence = parseRecoverySequence(param_name, param_value)) {
        group.default_sequence = *sequence;
      }
    } else if (key.starts_with(error_specific)) {
      // Parsed even for an invalid error code so that it doesn't hide an unknown behavior name
      auto sequence = parseRecoverySequence(param_name, param_value);
      // Text that isn't a number gives 0, which is not an error code either
      const auto error_code =
        static_cast<uint16_t>(std::strtoul(key.substr(error_specific.size()).c_str(), nullptr, 10));
      if (error_code == 0) {
        RCLCPP_WARN(logger_, "Ignoring parameter %s: not an error code", param_name.c_str());
      } else if (sequence) {
        group.sequence_by_error_code[error_code] = *sequence;
      }
    } else {
      RCLCPP_WARN(logger_, "Ignoring parameter %s: unknown parameter", param_name.c_str());
    }
  }

  return group;
}

std::map<std::string, rclcpp::ParameterValue> RecoveryManager::getParametersUnder(
  const std::string & prefix)
{
  std::map<std::string, rclcpp::ParameterValue> parameters;
  const auto declared_parameters = node_->list_parameters(
    {prefix}, rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE);
  for (const auto & param_name : declared_parameters.names) {
    parameters[param_name] = node_->get_parameter(param_name).get_parameter_value();
  }
  return parameters;
}

rclcpp::ParameterValue RecoveryManager::getParameterValue(const std::string & param_name)
{
  return node_->has_parameter(param_name) ?
         node_->get_parameter(param_name).get_parameter_value() : rclcpp::ParameterValue();
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
  // 1. The first error code that is set and has recovery behaviors configured, from its error
  // specific sequence else the default one
  ErrorCodeGroup * group_with_error = nullptr;
  const RecoverySequence * recovery_sequence = nullptr;
  uint16_t error_code = 0;
  for (auto & group : error_code_groups_) {
    if (!config().blackboard->get(group.blackboard_key, error_code) || error_code == 0) {
      continue;
    }
    const auto specific_sequence = group.sequence_by_error_code.find(error_code);
    recovery_sequence = specific_sequence != group.sequence_by_error_code.end() ?
      &specific_sequence->second : &group.default_sequence;
    if (!recovery_sequence->empty()) {
      group_with_error = &group;
      break;
    }
    RCLCPP_WARN(
      logger_, "No recovery configured for %s %u", group.blackboard_key.c_str(), error_code);
  }
  if (group_with_error == nullptr) {
    RCLCPP_WARN(logger_, "No recovery configured for any error code that is set");
    return false;
  }

  error_description_ = group_with_error->blackboard_key + " " + std::to_string(error_code);
  const RecoverySequence & sequence = *recovery_sequence;

  // 2. The next behavior of that sequence
  std::size_t & next_behavior_index =
    group_with_error->next_behavior_index_by_error_code[error_code];
  if (next_behavior_index >= sequence.size()) {
    if (!wrap_around_) {
      RCLCPP_WARN(
        logger_, "Already executed all %zu recovery behaviors for %s",
        sequence.size(), error_description_.c_str());
      return false;
    }
    next_behavior_index = 0;
  }

  running_behavior_index_ = sequence[next_behavior_index];
  ++next_behavior_index;

  RCLCPP_INFO(
    logger_, "Executing recovery behavior %s (%zu/%zu) for %s",
    children_nodes_[*running_behavior_index_]->name().c_str(), next_behavior_index,
    sequence.size(), error_description_.c_str());
  return true;
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
