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

#ifndef NAV2_BEHAVIOR_TREE__BT_ACTION_SERVER_HPP_
#define NAV2_BEHAVIOR_TREE__BT_ACTION_SERVER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <set>
#include <exception>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/behavior_tree_engine.hpp"
#include "nav2_behavior_tree/ros_topic_logger.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"

namespace nav2_behavior_tree
{
/**
 * @class nav2_behavior_tree::BtActionServer
 * @brief An action server that uses behavior tree to execute an action
 */
template<class ActionT>
class BtActionServer
{
public:
  using ActionServer = nav2_util::SimpleActionServer<ActionT>;

  typedef std::function<bool (typename ActionT::Goal::ConstSharedPtr)> OnGoalReceivedCallback;
  typedef std::function<void ()> OnLoopCallback;

  /**
   * @brief A constructor for nav2_behavior_tree::BtActionServer class
   */
  explicit BtActionServer(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & action_name,
    OnGoalReceivedCallback on_goal_received_callback,
    OnLoopCallback on_loop_callback)
  : action_name_(action_name),
    node_(parent),
    on_goal_received_callback_(on_goal_received_callback),
    on_loop_callback_(on_loop_callback)
  {
    auto node = node_.lock();
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    RCLCPP_INFO(logger_, "Creating");

    const std::vector<std::string> plugin_libs = {
      "nav2_compute_path_to_pose_action_bt_node",
      "nav2_follow_path_action_bt_node",
      "nav2_back_up_action_bt_node",
      "nav2_spin_action_bt_node",
      "nav2_wait_action_bt_node",
      "nav2_clear_costmap_service_bt_node",
      "nav2_is_stuck_condition_bt_node",
      "nav2_goal_reached_condition_bt_node",
      "nav2_initial_pose_received_condition_bt_node",
      "nav2_goal_updated_condition_bt_node",
      "nav2_reinitialize_global_localization_service_bt_node",
      "nav2_rate_controller_bt_node",
      "nav2_distance_controller_bt_node",
      "nav2_speed_controller_bt_node",
      "nav2_truncate_path_action_bt_node",
      "nav2_goal_updater_node_bt_node",
      "nav2_recovery_node_bt_node",
      "nav2_pipeline_sequence_bt_node",
      "nav2_round_robin_node_bt_node",
      "nav2_transform_available_condition_bt_node",
      "nav2_time_expired_condition_bt_node",
      "nav2_distance_traveled_condition_bt_node"
    };

    // Declare this node's parameters
    node->declare_parameter("default_bt_xml_filename");
    node->declare_parameter("plugin_lib_names", plugin_libs);
    node->declare_parameter("enable_groot_monitoring", true);
    node->declare_parameter("groot_zmq_publisher_port", 1666);
    node->declare_parameter("groot_zmq_server_port", 1667);
  }

  /**
   * @brief A destructor for nav2_behavior_tree::BtActionServer class
   */
  ~BtActionServer() {}

  /**
   * @brief Configures member variables
   *
   * Initializes action server for, builds behavior tree from xml file,
   * and calls user-defined onConfigure.
   * @return true on SUCCESS and false on FAILURE
   */
  bool on_configure(const std::shared_ptr<tf2_ros::Buffer> & tf)
  {
    RCLCPP_INFO(logger_, "Configuring");

    auto node = node_.lock();
    if (!node) {
      throw std::runtime_error{"Failed to lock node"};
    }

    // use suffix '_rclcpp_node' to keep parameter file consistency #1773
    auto options = rclcpp::NodeOptions().arguments(
      {"--ros-args",
        "-r", std::string("__node:=") + node->get_name() + "_rclcpp_node",
        "--"});
    // Support for handling the topic-based goal pose from rviz
    client_node_ = std::make_shared<rclcpp::Node>("_", options);

    // TF buffer
    tf_ = tf;

    action_server_ = std::make_shared<ActionServer>(
      node->get_node_base_interface(),
      node->get_node_clock_interface(),
      node->get_node_logging_interface(),
      node->get_node_waitables_interface(),
      action_name_, std::bind(&BtActionServer<ActionT>::executeCallback, this));

    // Get the libraries to pull plugins from
    plugin_lib_names_ = node->get_parameter("plugin_lib_names").as_string_array();

    // Create the class that registers our custom nodes and executes the BT
    bt_ = std::make_unique<nav2_behavior_tree::BehaviorTreeEngine>(plugin_lib_names_);

    // Create the blackboard that will be shared by all of the nodes in the tree
    blackboard_ = BT::Blackboard::create();

    // Put items on the blackboard
    blackboard_->set<rclcpp::Node::SharedPtr>("node", client_node_);  // NOLINT
    blackboard_->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_);  // NOLINT
    blackboard_->set<std::chrono::milliseconds>("server_timeout", std::chrono::milliseconds(10));  // NOLINT

    // Get the BT filename to use from the node parameter
    node->get_parameter("default_bt_xml_filename", default_bt_xml_filename_);

    // Get parameter for monitoring with Groot via ZMQ Publisher
    node->get_parameter("enable_groot_monitoring", enable_groot_monitoring_);
    node->get_parameter("groot_zmq_publisher_port", groot_zmq_publisher_port_);
    node->get_parameter("groot_zmq_server_port", groot_zmq_server_port_);

    return true;
  }

  /**
   * @brief Activates action server
   * @return true on SUCCESS and false on FAILURE
   */
  bool on_activate()
  {
    RCLCPP_INFO(logger_, "Activating");
    if (!loadBehaviorTree(default_bt_xml_filename_)) {
      RCLCPP_ERROR(logger_, "Error loading XML file: %s", default_bt_xml_filename_.c_str());
      return false;
    }
    action_server_->activate();
    return true;
  }

  /**
   * @brief Deactivates action server
   * @return true on SUCCESS and false on FAILURE
   */
  bool on_deactivate()
  {
    RCLCPP_INFO(logger_, "Deactivating");
    action_server_->deactivate();
    return true;
  }

  /**
   * @brief Resets member variables
   * @return true on SUCCESS and false on FAILURE
   */
  bool on_cleanup()
  {
    RCLCPP_INFO(logger_, "Cleaning up");

    client_node_.reset();

    action_server_.reset();
    topic_logger_.reset();
    plugin_lib_names_.clear();
    current_bt_xml_filename_.clear();
    blackboard_.reset();
    bt_->haltAllActions(tree_.rootNode());
    bt_->resetGrootMonitor();
    bt_.reset();

    RCLCPP_INFO(logger_, "Completed Cleaning up");
    return true;
  }

  /**
   * @brief Called when in shutdown state
   * @return true on SUCCESS and false on FAILURE
   */
  bool on_shutdown()
  {
    RCLCPP_INFO(logger_, "Shutting down");
    return true;
  }

  /**
   * @brief Replace current BT with another one
   * @param bt_xml_filename The file containing the new BT, uses default filename if empty
   * @return true if the resulting BT correspond to the one in bt_xml_filename. false
   * if something went wrong, and previous BT is maintained
   */
  bool loadBehaviorTree(const std::string & bt_xml_filename)
  {
    // Empty filename is default for backward compatibility
    auto filename = bt_xml_filename.empty() ? default_bt_xml_filename_ : bt_xml_filename;

    // Use previous BT if it is the existing one
    if (current_bt_xml_filename_ == filename) {
      RCLCPP_DEBUG(logger_, "BT will not be reloaded as the given xml is already loaded");
      return true;
    }

    // if a new tree is created, than the ZMQ Publisher must be destroyed
    bt_->resetGrootMonitor();

    // Read the input BT XML from the specified file into a string
    std::ifstream xml_file(filename);

    if (!xml_file.good()) {
      RCLCPP_ERROR(logger_, "Couldn't open input XML file: %s", filename.c_str());
      return false;
    }

    auto xml_string = std::string(
      std::istreambuf_iterator<char>(xml_file),
      std::istreambuf_iterator<char>());

    // Create the Behavior Tree from the XML input
    tree_ = bt_->createTreeFromText(xml_string, blackboard_);
    topic_logger_ = std::make_unique<RosTopicLogger>(client_node_, tree_);

    current_bt_xml_filename_ = filename;

    // Enable monitoring with Groot
    if (enable_groot_monitoring_) {
      // optionally add max_msg_per_second = 25 (default) here
      try {
        bt_->addGrootMonitoring(&tree_, groot_zmq_publisher_port_, groot_zmq_server_port_);
      } catch (const std::logic_error & e) {
        RCLCPP_ERROR(logger_, "ZMQ already enabled, Error: %s", e.what());
      }
    }
    return true;
  }

  /**
   * @brief Getter function for BT Blackboard
   * @return shared pointer to current BT blackboard
   */
  BT::Blackboard::Ptr getBlackboard() const
  {
    return blackboard_;
  }

  /**
   * @brief Getter function for action server
   * @return shared pointer to action server
   */
  std::shared_ptr<ActionServer> getActionServer() const
  {
    return action_server_;
  }

protected:
  /**
   * @brief Action server callback
   */
  void executeCallback()
  {
    if (!on_goal_received_callback_(action_server_->get_current_goal())) {
      action_server_->terminate_current();
      return;
    }

    auto is_canceling = [this]() {
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
        if (action_server_->is_preempt_requested()) {
          RCLCPP_INFO(logger_, "Received goal preemption request");
          action_server_->accept_pending_goal();
          on_goal_received_callback_(action_server_->get_current_goal());
        }
        topic_logger_->flush();
        on_loop_callback_();
      };

    // Execute the BT that was previously created in the configure step
    nav2_behavior_tree::BtStatus rc = bt_->run(&tree_, on_loop, is_canceling);

    // Make sure that the Bt is not in a running state from a previous execution
    // note: if all the ControlNodes are implemented correctly, this is not needed.
    bt_->haltAllActions(tree_.rootNode());

    switch (rc) {
      case nav2_behavior_tree::BtStatus::SUCCEEDED:
        RCLCPP_INFO(logger_, "Goal succeeded");
        action_server_->succeeded_current();
        break;

      case nav2_behavior_tree::BtStatus::FAILED:
        RCLCPP_ERROR(logger_, "Goal failed");
        action_server_->terminate_current();
        break;

      case nav2_behavior_tree::BtStatus::CANCELED:
        RCLCPP_INFO(logger_, "Goal canceled");
        action_server_->terminate_all();
        break;
    }
  }

  // Action name
  std::string action_name_;

  // Our action server implements the template action
  std::shared_ptr<ActionServer> action_server_;

  // Behavior Tree to be executed when goal is received
  BT::Tree tree_;

  // The blackboard shared by all of the nodes in the tree
  BT::Blackboard::Ptr blackboard_;

  // The XML fiñe that cointains the Behavior Tree to create
  std::string current_bt_xml_filename_;
  std::string default_bt_xml_filename_;

  // The wrapper class for the BT functionality
  std::unique_ptr<nav2_behavior_tree::BehaviorTreeEngine> bt_;

  // Libraries to pull plugins (BT Nodes) from
  std::vector<std::string> plugin_lib_names_;

  // A regular, non-spinning ROS node that we can use for calls to the action client
  rclcpp::Node::SharedPtr client_node_;

  // Spinning transform that can be used by the BT nodes
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // Parent node
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

  // Clock
  rclcpp::Clock::SharedPtr clock_;

  // Logger
  rclcpp::Logger logger_{rclcpp::get_logger("BtActionServer")};

  // To publish BT logs
  std::unique_ptr<RosTopicLogger> topic_logger_;

  // Parameters for Groot monitoring
  bool enable_groot_monitoring_;
  int groot_zmq_publisher_port_;
  int groot_zmq_server_port_;

  // User-provided callbacks
  OnGoalReceivedCallback on_goal_received_callback_;
  OnLoopCallback on_loop_callback_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__BT_ACTION_SERVER_HPP_
