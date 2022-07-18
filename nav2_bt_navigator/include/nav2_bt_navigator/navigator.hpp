// Copyright (c) 2021 Samsung Research America
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

#ifndef NAV2_BT_NAVIGATOR__NAVIGATOR_HPP_
#define NAV2_BT_NAVIGATOR__NAVIGATOR_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "nav2_util/odometry_utils.hpp"
#include "tf2_ros/buffer.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav2_behavior_tree/bt_action_server.hpp"
#include "nav2_core/navigator_base.hpp"

namespace nav2_bt_navigator
{

/**
 * @class Navigator
 * @brief Navigator interface that acts as a base class for all BT-based Navigator action's plugins
 */
template<class ActionT>
class Navigator : public nav2_core::NavigatorBase
{
public:
  using Ptr = std::shared_ptr<nav2_bt_navigator::Navigator<ActionT>>;

  /**
   * @brief A Navigator constructor
   */
  Navigator() : NavigatorBase() {}

  /**
   * @brief Virtual destructor
   */
  virtual ~Navigator() = default;

  bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
    const std::vector<std::string> &plugin_lib_names,
    const nav2_core::FeedbackUtils & feedback_utils,
    nav2_core::NavigatorMuxer * plugin_muxer,
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) override
  {
    auto node = parent_node.lock();
    RCLCPP_INFO(node->get_logger(), "Configuring %s", getName().c_str());
    
    logger_ = node->get_logger();
    clock_ = node->get_clock();
    feedback_utils_ = feedback_utils;
    plugin_muxer_ = plugin_muxer;
    odom_smoother_ = odom_smoother;

    // Create the Behavior Tree Action Server for this navigator
    bt_action_server_ = std::make_unique<nav2_behavior_tree::BtActionServer<ActionT>>(
      parent_node.lock(),
      getName(),
      plugin_lib_names,
      default_bt_xml_filename_,
      std::bind(&Navigator::onGoalReceived, this, std::placeholders::_1),
      std::bind(&Navigator::onLoop, this),
      std::bind(&Navigator::onPreempt, this, std::placeholders::_1),
      std::bind(&Navigator::onCompletion, this, std::placeholders::_1, std::placeholders::_2));

    bool ok = true;
    if (!bt_action_server_->on_configure()) {
      ok = false;
    }

    // get the default behavior tree for this navigator
    default_bt_xml_filename_ = getDefaultBTFilepath(parent_node);

    BT::Blackboard::Ptr blackboard = bt_action_server_->getBlackboard();
    blackboard->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", feedback_utils.tf);  // NOLINT
    blackboard->set<bool>("initial_pose_received", false);  // NOLINT
    blackboard->set<int>("number_recoveries", 0);  // NOLINT
    return onConfigure(parent_node) && ok;
  }

  /**
   * @brief Activation of the navigator's backend BT and actions
   * @return bool If successful
   */
  bool activate() override
  {
    bt_action_server_->on_activate();
    return onActivate();
  }

  /**
   * @brief Deactivation of the navigator's backend BT and actions
   * @return bool If successful
   */
  bool deactivate() override
  {
    bt_action_server_->on_deactivate();
    return onDeactivate();
  }

  /**
   * @brief Cleanup a navigator
   * @return bool If successful
   */
  bool cleanup() override
  {
    bt_action_server_.reset();
    return onCleanup();
  }

protected:
  /**
   * @brief Get the action name of this navigator to expose
   * @return string Name of action to expose
   */
  virtual std::string getName() = 0;

  virtual std::string getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) = 0;
  
  /**
   * @brief An intermediate goal reception function to mux navigators.
   */
  bool onGoalReceived(typename ActionT::Goal::ConstSharedPtr goal)
  {
    if (plugin_muxer_->isNavigating()) {
      RCLCPP_ERROR(
        logger_,
        "Requested navigation from %s while another navigator is processing,"
        " rejecting request.", getName().c_str());
      return false;
    }

    bool goal_accepted = goalReceived(goal);

    if (goal_accepted) {
      plugin_muxer_->startNavigating(getName());
    }

    return goal_accepted;
  }

  /**
   * @brief An intermediate completion function to mux navigators
   */
  void onCompletion(
    typename ActionT::Result::SharedPtr result,
    const nav2_behavior_tree::BtStatus final_bt_status)
  {
    plugin_muxer_->stopNavigating(getName());
    goalCompleted(result, final_bt_status);
  }

  /**
   * @brief A callback to be called when a new goal is received by the BT action server
   * Can be used to check if goal is valid and put values on
   * the blackboard which depend on the received goal
   */
  virtual bool goalReceived(typename ActionT::Goal::ConstSharedPtr goal) = 0;

  /**
   * @brief A callback that defines execution that happens on one iteration through the BT
   * Can be used to publish action feedback
   */
  virtual void onLoop() = 0;

  /**
   * @brief A callback that is called when a preempt is requested
   */
  virtual void onPreempt(typename ActionT::Goal::ConstSharedPtr goal) = 0;

  /**
   * @brief A callback that is called when a the action is completed; Can fill in
   * action result message or indicate that this action is done.
   */
  virtual void goalCompleted(
    typename ActionT::Result::SharedPtr result,
    const nav2_behavior_tree::BtStatus final_bt_status) = 0;

  virtual bool onConfigure(rclcpp_lifecycle::LifecycleNode::WeakPtr /*node*/) 
  {
    return true;
  }

  virtual bool onActivate()
  {
    return true;
  }

  virtual bool onDeactivate()
  {
    return true;
  }

  virtual bool onCleanup()
  {
    return true;
  }

  rclcpp::Logger logger_{rclcpp::get_logger("Navigator")};
  rclcpp::Clock::SharedPtr clock_;
  nav2_core::FeedbackUtils feedback_utils_;
  nav2_core::NavigatorMuxer * plugin_muxer_;

  // Odometry smoother object
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother_;

  std::string default_bt_xml_filename_;
  std::unique_ptr<nav2_behavior_tree::BtActionServer<ActionT>> bt_action_server_;
};

}  // namespace nav2_bt_navigator

#endif  // NAV2_BT_NAVIGATOR__NAVIGATOR_HPP_
