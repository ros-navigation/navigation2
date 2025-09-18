// Copyright (c) 2020 Sarthak Mittal
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

#ifndef NAV2_BEHAVIOR_TREE__BT_ACTION_SERVER_IMPL_HPP_
#define NAV2_BEHAVIOR_TREE__BT_ACTION_SERVER_IMPL_HPP_

#include <chrono>
#include <exception>
#include <fstream>
#include <limits>
#include <memory>
#include <set>
#include <utility>
#include <string>
#include <vector>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_behavior_tree/bt_action_server.hpp"
#include "nav2_ros_common/node_utils.hpp"
#include "rcl_action/action_server.h"
#include "nav2_ros_common/lifecycle_node.hpp"

namespace nav2_behavior_tree
{

template<class ActionT, class NodeT>
BtActionServer<ActionT, NodeT>::BtActionServer(
  const typename NodeT::WeakPtr & parent,
  const std::string & action_name,
  const std::vector<std::string> & plugin_lib_names,
  const std::string & default_bt_xml_filename,
  OnGoalReceivedCallback on_goal_received_callback,
  OnLoopCallback on_loop_callback,
  OnPreemptCallback on_preempt_callback,
  OnCompletionCallback on_completion_callback,
  const std::vector<std::string> & search_directories)
: action_name_(action_name),
  default_bt_xml_filename_or_id_(default_bt_xml_filename),
  search_directories_(search_directories),
  plugin_lib_names_(plugin_lib_names),
  node_(parent),
  on_goal_received_callback_(on_goal_received_callback),
  on_loop_callback_(on_loop_callback),
  on_preempt_callback_(on_preempt_callback),
  on_completion_callback_(on_completion_callback),
  internal_error_code_(0),
  internal_error_msg_()
{
  auto node = node_.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  std::vector<std::string> default_error_code_name_prefixes = {
    "assisted_teleop",
    "backup",
    "compute_path",
    "dock_robot",
    "drive_on_heading",
    "follow_object",
    "follow_path",
    "nav_thru_poses",
    "nav_to_pose",
    "spin",
    "undock_robot",
    "wait",
  };

  if (node->has_parameter("error_code_names")) {
    throw std::runtime_error("parameter 'error_code_names' has been replaced by "
      " 'error_code_name_prefixes' and MUST be removed.\n"
      " Please review migration guide and update your configuration.");
  }

  // Declare and get error code name prefixes parameter
  error_code_name_prefixes_ = node->declare_or_get_parameter(
    "error_code_name_prefixes",
    default_error_code_name_prefixes);

  // Provide informative logging about error code prefixes
  std::string error_code_name_prefixes_str;
  for (const auto & error_code_name_prefix : error_code_name_prefixes_) {
    error_code_name_prefixes_str += " " + error_code_name_prefix;
  }

  if (error_code_name_prefixes_ == default_error_code_name_prefixes) {
    RCLCPP_WARN_STREAM(
      logger_, "error_code_name_prefixes parameters were not set. Using default values of:"
        << error_code_name_prefixes_str + "\n"
        << "Make sure these match your BT and there are not other sources of error codes you"
        << "reported to your application");
  } else {
    RCLCPP_INFO_STREAM(logger_, "Error_code parameters were set to:"
      << error_code_name_prefixes_str);
  }
}

template<class ActionT, class NodeT>
BtActionServer<ActionT, NodeT>::~BtActionServer()
{}

template<class ActionT, class NodeT>
bool BtActionServer<ActionT, NodeT>::on_configure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Name client node after action name
  std::string client_node_name = action_name_;
  std::replace(client_node_name.begin(), client_node_name.end(), '/', '_');
  // Use suffix '_rclcpp_node' to keep parameter file consistency #1773

  auto new_arguments = node->get_node_options().arguments();
  nav2::replaceOrAddArgument(new_arguments, "-r", "__node", std::string("__node:=") +
    std::string(node->get_name()) + "_" + client_node_name + "_rclcpp_node");
  auto options = node->get_node_options();
  options = options.arguments(new_arguments);

  // Support for handling the topic-based goal pose from rviz
  client_node_ = std::make_shared<nav2::LifecycleNode>("_", options);
  client_node_->configure();
  client_node_->activate();

  // Declare parameters for common client node applications to share with BT nodes
  // Declare if not declared in case being used an external application, then copying
  // all of the main node's parameters to the client for BT nodes to obtain
  nav2::declare_parameter_if_not_declared(
    node, "global_frame", rclcpp::ParameterValue(std::string("map")));
  nav2::declare_parameter_if_not_declared(
    node, "robot_base_frame", rclcpp::ParameterValue(std::string("base_link")));
  nav2::declare_parameter_if_not_declared(
    node, "transform_tolerance", rclcpp::ParameterValue(0.1));
  rclcpp::copy_all_parameter_values(node, client_node_);

  // Could be using a user rclcpp::Node, so need to use the Nav2 factory to create the subscription
  // to convert nav2::LifecycleNode, rclcpp::Node or rclcpp_lifecycle::LifecycleNode
  action_server_ = nav2::interfaces::create_action_server<ActionT>(
    node, action_name_, std::bind(&BtActionServer<ActionT, NodeT>::executeCallback, this),
    nullptr, std::chrono::milliseconds(500), false);

  // Get parameters for BT timeouts
  bt_loop_duration_ = std::chrono::milliseconds(
    node->declare_or_get_parameter("bt_loop_duration", 10));

  default_server_timeout_ = std::chrono::milliseconds(
    node->declare_or_get_parameter("default_server_timeout", 20));

  wait_for_service_timeout_ = std::chrono::milliseconds(
    node->declare_or_get_parameter("wait_for_service_timeout", 1000));

  always_reload_bt_ = node->declare_or_get_parameter(
    "always_reload_bt_xml", false);

  // Get error code id names to grab off of the blackboard
  error_code_name_prefixes_ = node->get_parameter("error_code_name_prefixes").as_string_array();

  // Create the class that registers our custom nodes and executes the BT
  bt_ = std::make_unique<nav2_behavior_tree::BehaviorTreeEngine>(plugin_lib_names_, client_node_);

  // Create the blackboard that will be shared by all of the nodes in the tree
  blackboard_ = BT::Blackboard::create();

  // Put items on the blackboard
  blackboard_->set<nav2::LifecycleNode::SharedPtr>("node", client_node_);  // NOLINT
  blackboard_->set<std::chrono::milliseconds>("server_timeout", default_server_timeout_);  // NOLINT
  blackboard_->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_);  // NOLINT
  blackboard_->set<std::chrono::milliseconds>(
    "wait_for_service_timeout",
    wait_for_service_timeout_);
  blackboard_->set<uint64_t>("run_id", run_id_);  // NOLINT
  return true;
}

template<class ActionT, class NodeT>
bool BtActionServer<ActionT, NodeT>::on_activate()
{
  resetInternalError();
  if (!loadBehaviorTree(default_bt_xml_filename_or_id_)) {
    RCLCPP_ERROR(logger_, "Error loading BT: %s", default_bt_xml_filename_or_id_.c_str());
    return false;
  }
  action_server_->activate();
  return true;
}

template<class ActionT, class NodeT>
bool BtActionServer<ActionT, NodeT>::on_deactivate()
{
  action_server_->deactivate();
  return true;
}

template<class ActionT, class NodeT>
bool BtActionServer<ActionT, NodeT>::on_cleanup()
{
  client_node_->deactivate();
  client_node_->cleanup();
  client_node_.reset();
  action_server_.reset();
  topic_logger_.reset();
  plugin_lib_names_.clear();
  current_bt_file_or_id_.clear();
  blackboard_.reset();
  bt_->haltAllActions(tree_);
  bt_->resetGrootMonitor();
  bt_.reset();
  return true;
}

template<class ActionT, class NodeT>
void BtActionServer<ActionT, NodeT>::setGrootMonitoring(
  const bool enable,
  const unsigned server_port)
{
  enable_groot_monitoring_ = enable;
  groot_server_port_ = server_port;
}

template<class ActionT, class NodeT>
bool BtActionServer<ActionT, NodeT>::loadBehaviorTree(const std::string & bt_xml_filename_or_id)
{
  namespace fs = std::filesystem;

  // Empty argument is default for backward compatibility
  auto file_or_id =
    bt_xml_filename_or_id.empty() ? default_bt_xml_filename_or_id_ : bt_xml_filename_or_id;

  // Use previous BT if it is the existing one and always reload flag is not set to true
  if (!always_reload_bt_ && current_bt_file_or_id_ == file_or_id) {
    RCLCPP_DEBUG(logger_, "BT will not be reloaded as the given xml or ID is already loaded");
    return true;
  }

  // Reset any existing Groot2 monitoring
  bt_->resetGrootMonitor();

  bool is_bt_id = false;
  if ((file_or_id.length() < 4) ||
    file_or_id.substr(file_or_id.length() - 4) != ".xml")
  {
    is_bt_id = true;
  }

  std::set<std::string> registered_ids;
  std::string main_id;
  auto register_all_bt_files = [&](const std::string & skip_file = "") {
      for (const auto & directory : search_directories_) {
        for (const auto & entry : fs::directory_iterator(directory)) {
          if (entry.path().extension() != ".xml") {
            continue;
          }
          if (!skip_file.empty() && entry.path().string() == skip_file) {
            continue;
          }

          auto id = bt_->extractBehaviorTreeID(entry.path().string());
          if (id.empty()) {
            RCLCPP_ERROR(logger_, "Skipping BT file %s (missing ID)", entry.path().c_str());
            continue;
          }
          if (registered_ids.count(id)) {
            RCLCPP_WARN(logger_, "Skipping conflicting BT file %s (duplicate ID %s)",
              entry.path().c_str(), id.c_str());
            continue;
          }

          RCLCPP_DEBUG(logger_, "Registering Tree from File: %s", entry.path().string().c_str());
          bt_->registerTreeFromFile(entry.path().string());
          registered_ids.insert(id);
        }
      }
    };

  try {
    if (!is_bt_id) {
      // file_or_id is a filename: register it first
      std::string main_file = file_or_id;
      main_id = bt_->extractBehaviorTreeID(main_file);
      if (main_id.empty()) {
        RCLCPP_ERROR(logger_, "Failed to extract ID from %s", main_file.c_str());
        return false;
      }
      RCLCPP_DEBUG(logger_, "Registering Tree from File: %s", main_file.c_str());
      bt_->registerTreeFromFile(main_file);
      registered_ids.insert(main_id);

      // When a filename is specified, it must be register first
      // and treat it as the "main" tree to execute.
      // This ensures the requested tree is always available
      // and prioritized, even if other files in the directory have duplicate IDs.
      // The lambda then skips this main file to avoid
      // re-registering it or logging a duplicate warning.
      // In contrast, when an ID is specified, it's unknown which file is "main"
      // so all files are registered and conflicts are handled in the lambda.
      register_all_bt_files(main_file);
    } else {
      // file_or_id is an ID: register all files, skipping conflicts
      main_id = file_or_id;
      register_all_bt_files();
    }
  } catch (const std::exception & e) {
    setInternalError(ActionT::Result::FAILED_TO_LOAD_BEHAVIOR_TREE,
      "Exception registering behavior trees: " + std::string(e.what()));
    return false;
  }

  // Create the tree with the specified ID
  try {
    tree_ = bt_->createTree(main_id, blackboard_);
    RCLCPP_INFO(logger_, "Created BT from ID: %s", main_id.c_str());

    for (auto & subtree : tree_.subtrees) {
      auto & blackboard = subtree->blackboard;
      blackboard->set("node", client_node_);
      blackboard->set<std::chrono::milliseconds>("server_timeout", default_server_timeout_);
      blackboard->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_);
      blackboard->set<std::chrono::milliseconds>(
          "wait_for_service_timeout", wait_for_service_timeout_);
    }
  } catch (const std::exception & e) {
    setInternalError(ActionT::Result::FAILED_TO_LOAD_BEHAVIOR_TREE,
      std::string("Exception when creating BT tree from ID: ") + e.what());
    return false;
  }

  // Optional logging and monitoring
  topic_logger_ = std::make_unique<RosTopicLogger>(client_node_, tree_);
  current_bt_file_or_id_ = file_or_id;

  if (enable_groot_monitoring_) {
    bt_->addGrootMonitoring(&tree_, groot_server_port_);
    RCLCPP_DEBUG(
      logger_, "Enabling Groot2 monitoring for %s: %d",
      action_name_.c_str(), groot_server_port_);
  }

  return true;
}

template<class ActionT, class NodeT>
void BtActionServer<ActionT, NodeT>::executeCallback()
{
  if (!on_goal_received_callback_(action_server_->get_current_goal())) {
    // Give server an opportunity to populate the result message
    // if the goal is not accepted
    auto result = std::make_shared<typename ActionT::Result>();
    populateErrorCode(result);
    action_server_->terminate_current(result);
    cleanErrorCodes();
    return;
  }

  run_id_++;
  blackboard_->set<uint64_t>("run_id", run_id_);

  auto is_canceling = [&]() {
      if (action_server_ == nullptr) {
        RCLCPP_DEBUG(logger_, "Action server unavailable. Canceling.");
        return true;
      }
      if (!action_server_->is_server_active()) {
        RCLCPP_DEBUG(logger_, "Action server is inactive. Canceling.");
        return true;
      }
      return action_server_->is_cancel_requested();
    };

  auto on_loop = [&]() {
      if (action_server_->is_preempt_requested() && on_preempt_callback_) {
        on_preempt_callback_(action_server_->get_pending_goal());
      }
      topic_logger_->flush();
      on_loop_callback_();
    };

  // Execute the BT that was previously created in the configure step
  nav2_behavior_tree::BtStatus rc = bt_->run(&tree_, on_loop, is_canceling, bt_loop_duration_);

  // Make sure that the Bt is not in a running state from a previous execution
  // note: if all the ControlNodes are implemented correctly, this is not needed.
  bt_->haltAllActions(tree_);

  // Give server an opportunity to populate the result message or simple give
  // an indication that the action is complete.
  auto result = std::make_shared<typename ActionT::Result>();

  populateErrorCode(result);

  on_completion_callback_(result, rc);

  switch (rc) {
    case nav2_behavior_tree::BtStatus::SUCCEEDED:
      action_server_->succeeded_current(result);
      RCLCPP_INFO(logger_, "Goal succeeded");
      break;

    case nav2_behavior_tree::BtStatus::FAILED:
      action_server_->terminate_current(result);
      RCLCPP_ERROR(logger_, "Goal failed error_code:%d error_msg:'%s'", result->error_code,
        result->error_msg.c_str());
      break;

    case nav2_behavior_tree::BtStatus::CANCELED:
      action_server_->terminate_all(result);
      RCLCPP_INFO(logger_, "Goal canceled");
      break;
  }

  cleanErrorCodes();
}

template<class ActionT, class NodeT>
void BtActionServer<ActionT, NodeT>::setInternalError(
  uint16_t error_code,
  const std::string & error_msg)
{
  internal_error_code_ = error_code;
  internal_error_msg_ = error_msg;
  RCLCPP_ERROR(logger_, "Setting internal error error_code:%d, error_msg:%s",
    internal_error_code_, internal_error_msg_.c_str());
}

template<class ActionT, class NodeT>
void BtActionServer<ActionT, NodeT>::resetInternalError(void)
{
  internal_error_code_ = ActionT::Result::NONE;
  internal_error_msg_ = "";
}

template<class ActionT, class NodeT>
bool BtActionServer<ActionT, NodeT>::populateInternalError(
  typename std::shared_ptr<typename ActionT::Result> result)
{
  if (internal_error_code_ != ActionT::Result::NONE) {
    result->error_code = internal_error_code_;
    result->error_msg = internal_error_msg_;
    return true;
  }
  return false;
}

template<class ActionT, class NodeT>
void BtActionServer<ActionT, NodeT>::populateErrorCode(
  typename std::shared_ptr<typename ActionT::Result> result)
{
  int highest_priority_error_code = std::numeric_limits<int>::max();
  std::string highest_priority_error_msg = "";
  std::string name;

  if (internal_error_code_ != 0) {
    highest_priority_error_code = internal_error_code_;
    highest_priority_error_msg = internal_error_msg_;
  }

  for (const auto & error_code_name_prefix : error_code_name_prefixes_) {
    try {
      name = error_code_name_prefix + "_error_code";
      int current_error_code = blackboard_->get<int>(name);
      if (current_error_code != 0 && current_error_code < highest_priority_error_code) {
        highest_priority_error_code = current_error_code;
        name = error_code_name_prefix + "_error_msg";
        highest_priority_error_msg = blackboard_->get<std::string>(name);
      }
    } catch (...) {
      RCLCPP_DEBUG(
        logger_,
        "Failed to get error code name: %s from blackboard",
        name.c_str());
    }
  }

  if (highest_priority_error_code != std::numeric_limits<int>::max()) {
    result->error_code = highest_priority_error_code;
    result->error_msg = highest_priority_error_msg;
  }
}

template<class ActionT, class NodeT>
void BtActionServer<ActionT, NodeT>::cleanErrorCodes()
{
  std::string name;
  for (const auto & error_code_name_prefix : error_code_name_prefixes_) {
    name = error_code_name_prefix + "_error_code";
    blackboard_->set<unsigned short>(name, 0);  //NOLINT
    name = error_code_name_prefix + "_error_msg";
    blackboard_->set<std::string>(name, "");
  }
  resetInternalError();
}

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__BT_ACTION_SERVER_IMPL_HPP_
