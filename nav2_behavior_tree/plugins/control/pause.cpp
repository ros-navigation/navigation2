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

#include <thread>

// ROS includes
#include "nav2_behavior_tree/plugins/control/pause.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

// Other includes
#include "behaviortree_cpp/bt_factory.h"

namespace nav2_behavior_tree
{

using namespace std::placeholders;

Pause::Pause(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ControlNode(xml_tag_name, conf)
{
  node_ = this->config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  state_ = UNPAUSED;

  // Create a separate cb group with a separate executor to spin
  cb_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);

  executor_ =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  executor_->add_callback_group(cb_group_, node_->get_node_base_interface());

  std::string pause_service_name;
  getInput("pause_service_name", pause_service_name);
  pause_srv_ = node_->create_service<Trigger>(
    pause_service_name,
    std::bind(&Pause::pause_service_callback, this, _1, _2),
    rclcpp::ServicesQoS(), cb_group_);

  std::string resume_service_name;
  getInput("resume_service_name", resume_service_name);
  resume_srv_ = node_->create_service<Trigger>(
    resume_service_name,
    std::bind(&Pause::resume_service_callback, this, _1, _2),
    rclcpp::ServicesQoS(), cb_group_);

  spinner_thread_ = std::make_unique<std::thread>(
    [&]() {
      executor_->spin();
    });
  spinner_thread_->detach();
}

Pause::~Pause()
{
  RCLCPP_DEBUG(node_->get_logger(), "Shutting down Pause BT node");
  executor_->cancel();
}

BT::NodeStatus Pause::tick()
{
  unsigned int children_count = children_nodes_.size();
  if (children_count < 1 || children_count > 4) {
    throw BT::LogicError(
            "PauseNode must have at least one and at most four children "
            "(currently has " + std::to_string(children_count) + ")");
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  if (status() == BT::NodeStatus::IDLE) {
    state_ = UNPAUSED;
  }
  if (state_ == PAUSE_REQUESTED) {
    resetChildren();
    state_ = ON_PAUSE;
    RCLCPP_INFO(node_->get_logger(), "Switched to state: ON_PAUSE");
  }
  if (state_ == RESUME_REQUESTED) {
    resetChildren();
    state_ = ON_RESUME;
    RCLCPP_INFO(node_->get_logger(), "Switched to state: ON_RESUME");
  }
  setStatus(BT::NodeStatus::RUNNING);

  if (state_ == ON_PAUSE) {
    if (children_count < 3) {
      // Event not handled, go directly to PAUSED state
      RCLCPP_INFO(node_->get_logger(), "Switched to state: PAUSED");
      state_ = PAUSED;
    } else {
      // Tick ON_PAUSE child
      const BT::NodeStatus child_status = children_nodes_[2]->executeTick();
      switch (child_status) {
        case BT::NodeStatus::RUNNING:
          break;
        case BT::NodeStatus::SUCCESS:
          RCLCPP_INFO(node_->get_logger(), "Switched to state: PAUSED");
          state_ = PAUSED;
          break;
        case BT::NodeStatus::SKIPPED:
          // Same as SUCCESS
          RCLCPP_INFO(node_->get_logger(), "Switched to state: PAUSED");
          state_ = PAUSED;
          break;
        case BT::NodeStatus::FAILURE:
          RCLCPP_ERROR(node_->get_logger(), "ON_PAUSE child returned FAILURE");
          setStatus(BT::NodeStatus::FAILURE);
          break;
        default:
          throw BT::LogicError("A child node must never return IDLE");
      }
    }
  }

  if (state_ == ON_RESUME) {
    if (children_count < 4) {
      // Event not handled, go directly to UNPAUSED state
      RCLCPP_INFO(node_->get_logger(), "Switched to state: UNPAUSED");
      state_ = UNPAUSED;
    } else {
      // Tick ON_RESUME child
      const BT::NodeStatus child_status = children_nodes_[3]->executeTick();
      switch (child_status) {
        case BT::NodeStatus::RUNNING:
          break;
        case BT::NodeStatus::SUCCESS:
          RCLCPP_INFO(node_->get_logger(), "Switched to state: UNPAUSED");
          state_ = UNPAUSED;
          break;
        case BT::NodeStatus::SKIPPED:
          // Same as SUCCESS
          RCLCPP_INFO(node_->get_logger(), "Switched to state: UNPAUSED");
          state_ = UNPAUSED;
          break;
        case BT::NodeStatus::FAILURE:
          RCLCPP_ERROR(node_->get_logger(), "ON_RESUME child returned FAILURE");
          setStatus(BT::NodeStatus::FAILURE);
          break;
        default:
          throw BT::LogicError("A child node must never return IDLE");
      }
    }
  }

  if (state_ == PAUSED) {
    if (children_count < 2) {
      // State not handled, do nothing until unpaused
    } else {
      // Tick PAUSED child
      const BT::NodeStatus child_status = children_nodes_[1]->executeTick();
      switch (child_status) {
        case BT::NodeStatus::RUNNING:
          break;
        case BT::NodeStatus::SUCCESS:
          break;
        case BT::NodeStatus::SKIPPED:
          // Same as SUCCESS
          break;
        case BT::NodeStatus::FAILURE:
          RCLCPP_ERROR(node_->get_logger(), "PAUSED child returned FAILURE");
          setStatus(BT::NodeStatus::FAILURE);
          break;
        default:
          throw BT::LogicError("A child node must never return IDLE");
      }
    }
  }

  if (state_ == UNPAUSED) {
    // Tick UNPAUSED child
    const BT::NodeStatus child_status = children_nodes_[0]->executeTick();
    switch (child_status) {
      case BT::NodeStatus::RUNNING:
        break;
      case BT::NodeStatus::SUCCESS:
        setStatus(BT::NodeStatus::SUCCESS);
        break;
      case BT::NodeStatus::SKIPPED:
        // Same as SUCCESS
        setStatus(BT::NodeStatus::SUCCESS);
        break;
      case BT::NodeStatus::FAILURE:
        RCLCPP_ERROR(node_->get_logger(), "UNPAUSED child returned FAILURE");
        setStatus(BT::NodeStatus::FAILURE);
        break;
      default:
        throw BT::LogicError("A child node must never return IDLE");
    }
  }

  return status();
}

void Pause::pause_service_callback(
  const std::shared_ptr<Trigger::Request>/*request*/,
  std::shared_ptr<Trigger::Response> response)
{
  if (status() == BT::NodeStatus::IDLE) {
    std::string warn_msg = "Pause BT node has not been ticked yet";
    response->success = false;
    response->message = warn_msg;
    RCLCPP_ERROR(node_->get_logger(), "%s", warn_msg.c_str());
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  if (state_ != PAUSED) {
    RCLCPP_INFO(node_->get_logger(), "PAUSE_REQUESTED");
    response->success = true;
    state_ = PAUSE_REQUESTED;
    return;
  }

  std::string warn_message = "Pause BT node already in state PAUSED";
  RCLCPP_WARN(node_->get_logger(), "%s", warn_message.c_str());
  response->success = false;
  response->message = warn_message;
}

void Pause::resume_service_callback(
  const std::shared_ptr<Trigger::Request>/*request*/,
  std::shared_ptr<Trigger::Response> response)
{
  if (status() == BT::NodeStatus::IDLE) {
    std::string warn_msg = "Pause BT node has not been ticked yet";
    response->success = false;
    response->message = warn_msg;
    RCLCPP_ERROR(node_->get_logger(), "%s", warn_msg.c_str());
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  if (state_ == PAUSED) {
    RCLCPP_INFO(node_->get_logger(), "RESUME_REQUESTED");
    response->success = true;
    state_ = RESUME_REQUESTED;
    return;
  }

  std::string warn_message = "You can't resume a node that is not paused";
  RCLCPP_WARN(node_->get_logger(), "%s", warn_message.c_str());
  response->success = false;
  response->message = warn_message;
}

void Pause::halt()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  state_ = UNPAUSED;
  ControlNode::halt();
}

}  // namespace nav2_behavior_tree

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::Pause>(
        name, config);
    };

  factory.registerBuilder<nav2_behavior_tree::Pause>("Pause", builder);
}
