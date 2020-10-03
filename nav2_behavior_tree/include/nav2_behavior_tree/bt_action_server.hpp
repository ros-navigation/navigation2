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
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"

namespace nav2_behavior_tree
{
/**
 * @class nav2_behavior_tree::BtActionServer
 * @brief An action server that uses behavior tree to execute an action
 */
template<class ActionT>
class BtActionServer : public nav2_util::LifecycleNode
{
public:
  using ActionServer = nav2_util::SimpleActionServer<ActionT>;

  /**
   * @brief A constructor for nav2_behavior_tree::BtActionServer class
   */
  explicit BtActionServer(
    const std::string & action_name,
    const std::string & node_name,
    const std::string & namespace_ = "",
    bool use_rclcpp_node = false,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : nav2_util::LifecycleNode(node_name, namespace_, use_rclcpp_node, options),
    action_name_(action_name)
  {
    RCLCPP_INFO(get_logger(), "Creating");

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
    declare_parameter("default_bt_xml_filename");
    declare_parameter("plugin_lib_names", plugin_libs);
    declare_parameter("enable_groot_monitoring", true);
    declare_parameter("groot_zmq_publisher_port", 1666);
    declare_parameter("groot_zmq_server_port", 1667);
  }

  /**
   * @brief A destructor for nav2_behavior_tree::BtActionServer class
   */
  ~BtActionServer() {}

  // Derived classes can override any of the following methods to hook into the
  // processing for different lifecycle states

  virtual nav2_util::CallbackReturn on_configure()
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  virtual nav2_util::CallbackReturn on_activate()
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  virtual nav2_util::CallbackReturn on_deactivate()
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  virtual nav2_util::CallbackReturn on_cleanup()
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  virtual nav2_util::CallbackReturn on_shutdown()
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  // Derived classes can override any of the following methods to hook into the
  // processing for the action: on_goal_received, on_loop, and is_cancelling

  // Could check if goal is valid and put values
  // on the blackboard which depend on the received goal
  virtual bool on_goal_received() = 0;

  // Could define execution that happens on one iteration through the BT
  // Preempt current goal when new goal is received
  // Publish feedback
  virtual void on_loop() = 0;

  // Decides if cancel has been requested for the current goal
  virtual bool is_canceling() = 0;

protected:
  /**
   * @brief Configures member variables
   *
   * Initializes action server for, builds behavior tree from xml file,
   * and calls user-defined onConfigure.
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & /*state*/) override
  {
    RCLCPP_INFO(get_logger(), "Configuring");

    // use suffix '_rclcpp_node' to keep parameter file consistency #1773
    auto options = rclcpp::NodeOptions().arguments(
      {"--ros-args",
        "-r", std::string("__node:=") + get_name() + "_rclcpp_node",
        "--"});
    // Support for handling the topic-based goal pose from rviz
    client_node_ = std::make_shared<rclcpp::Node>("_", options);

    tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(), get_node_timers_interface());
    tf_->setCreateTimerInterface(timer_interface);
    tf_->setUsingDedicatedThread(true);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_, this, false);

    action_server_ = std::make_unique<ActionServer>(
      get_node_base_interface(),
      get_node_clock_interface(),
      get_node_logging_interface(),
      get_node_waitables_interface(),
      action_name_, std::bind(&BtActionServer<ActionT>::executeCallback, this));

    // Get the libraries to pull plugins from
    plugin_lib_names_ = get_parameter("plugin_lib_names").as_string_array();

    // Create the class that registers our custom nodes and executes the BT
    bt_ = std::make_unique<nav2_behavior_tree::BehaviorTreeEngine>(plugin_lib_names_);

    // Create the blackboard that will be shared by all of the nodes in the tree
    blackboard_ = BT::Blackboard::create();

    // Put items on the blackboard
    blackboard_->set<rclcpp::Node::SharedPtr>("node", client_node_);  // NOLINT
    blackboard_->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_);  // NOLINT
    blackboard_->set<std::chrono::milliseconds>("server_timeout", std::chrono::milliseconds(10));  // NOLINT
    blackboard_->set<bool>("initial_pose_received", false);  // NOLINT
    blackboard_->set<int>("number_recoveries", 0);  // NOLINT

    // Get the BT filename to use from the node parameter
    get_parameter("default_bt_xml_filename", default_bt_xml_filename_);

    return on_configure();
  }

  /**
   * @brief Activates action server
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & /*state*/) override
  {
    RCLCPP_INFO(get_logger(), "Activating");

    if (!loadBehaviorTree(default_bt_xml_filename_)) {
      RCLCPP_ERROR(get_logger(), "Error loading XML file: %s", default_bt_xml_filename_.c_str());
      return nav2_util::CallbackReturn::FAILURE;
    }

    action_server_->activate();

    // create bond connection
    createBond();

    return on_activate();
  }

  /**
   * @brief Deactivates action server
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*state*/) override
  {
    RCLCPP_INFO(get_logger(), "Deactivating");

    action_server_->deactivate();

    // destroy bond connection
    destroyBond();

    return on_deactivate();
  }

  /**
   * @brief Resets member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & /*state*/) override
  {
    RCLCPP_INFO(get_logger(), "Cleaning up");

    client_node_.reset();

    // Reset the listener before the buffer
    tf_listener_.reset();
    tf_.reset();

    action_server_.reset();
    plugin_lib_names_.clear();
    current_bt_xml_filename_.clear();
    blackboard_.reset();
    bt_->haltAllActions(tree_.rootNode());
    bt_->resetGrootMonitor();
    bt_.reset();

    RCLCPP_INFO(get_logger(), "Completed Cleaning up");
    return on_cleanup();
  }

  /**
   * @brief Called when in shutdown state
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & /*state*/) override
  {
    RCLCPP_INFO(get_logger(), "Shutting down");
    return on_shutdown();
  }

  /**
   * @brief Action server callback
   */
  void executeCallback()
  {
    if (!on_goal_received()) {
      action_server_->terminate_current();
      return;
    }

    // Execute the BT that was previously created in the configure step
    nav2_behavior_tree::BtStatus rc = bt_->run(
      &tree_,
      std::bind(&BtActionServer<ActionT>::on_loop, this),
      std::bind(&BtActionServer<ActionT>::is_canceling, this));

    // Make sure that the Bt is not in a running state from a previous execution
    // note: if all the ControlNodes are implemented correctly, this is not needed.
    bt_->haltAllActions(tree_.rootNode());

    switch (rc) {
      case nav2_behavior_tree::BtStatus::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Goal succeeded");
        action_server_->succeeded_current();
        break;

      case nav2_behavior_tree::BtStatus::FAILED:
        RCLCPP_ERROR(get_logger(), "Goal failed");
        action_server_->terminate_current();
        break;

      case nav2_behavior_tree::BtStatus::CANCELED:
        RCLCPP_INFO(get_logger(), "Goal canceled");
        action_server_->terminate_all();
        break;
    }
  }

  /**
   * @brief Replace current BT with another one
   * @param bt_xml_filename The file containing the new BT
   * @return true if the resulting BT correspond to the one in bt_xml_filename. false
   * if something went wrong, and previous BT is maintained
   */
  bool loadBehaviorTree(const std::string & bt_xml_filename)
  {
    // Use previous BT if it is the existing one
    if (current_bt_xml_filename_ == bt_xml_filename) {
      RCLCPP_DEBUG(get_logger(), "BT will not be reloaded as the given xml is already loaded");
      return true;
    }

    // if a new tree is created, than the ZMQ Publisher must be destroyed
    bt_->resetGrootMonitor();

    // Read the input BT XML from the specified file into a string
    std::ifstream xml_file(bt_xml_filename);

    if (!xml_file.good()) {
      RCLCPP_ERROR(get_logger(), "Couldn't open input XML file: %s", bt_xml_filename.c_str());
      return false;
    }

    auto xml_string = std::string(
      std::istreambuf_iterator<char>(xml_file),
      std::istreambuf_iterator<char>());

    // Create the Behavior Tree from the XML input
    tree_ = bt_->createTreeFromText(xml_string, blackboard_);
    current_bt_xml_filename_ = bt_xml_filename;

    // get parameter for monitoring with Groot via ZMQ Publisher
    if (get_parameter("enable_groot_monitoring").as_bool()) {
      uint16_t zmq_publisher_port = get_parameter("groot_zmq_publisher_port").as_int();
      uint16_t zmq_server_port = get_parameter("groot_zmq_server_port").as_int();
      // optionally add max_msg_per_second = 25 (default) here
      try {
        bt_->addGrootMonitoring(&tree_, zmq_publisher_port, zmq_server_port);
      } catch (const std::logic_error & e) {
        RCLCPP_ERROR(get_logger(), "ZMQ already enabled, Error: %s", e.what());
      }
    }
    return true;
  }

  // Action name
  std::string action_name_;

  // Our action server implements the template action
  std::unique_ptr<ActionServer> action_server_;

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
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__BT_ACTION_SERVER_HPP_
