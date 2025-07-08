// Copyright (c) 2025 Intel Corporation
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

// Other includes
#include <functional>

// ROS includes
#include "nav2_behavior_tree/plugins/control/pause_resume_controller.hpp"

// Other includes
#include "behaviortree_cpp/bt_factory.h"

namespace nav2_behavior_tree
{

using namespace std::placeholders;

PauseResumeController::PauseResumeController(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ControlNode(xml_tag_name, conf)
{
  auto node = this->config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  state_ = RESUMED;

  // Create a separate cb group with a separate executor to spin
  cb_group_ = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);

  executor_ =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  executor_->add_callback_group(cb_group_, node->get_node_base_interface());

  std::string pause_service_name;
  getInput("pause_service_name", pause_service_name);
  pause_srv_ = std::make_shared<nav2::ServiceServer<Trigger>>(
    pause_service_name,
    node,
    std::bind(&PauseResumeController::pauseServiceCallback, this, _1, _2, _3),
    cb_group_);

  std::string resume_service_name;
  getInput("resume_service_name", resume_service_name);
  resume_srv_ = std::make_shared<nav2::ServiceServer<Trigger>>(
    resume_service_name,
    node,
    std::bind(&PauseResumeController::resumeServiceCallback, this, _1, _2, _3),
    cb_group_);
}

BT::NodeStatus PauseResumeController::tick()
{
  unsigned int children_count = children_nodes_.size();
  if (children_count < 1 || children_count > 4) {
    throw std::runtime_error(
            "PauseNode must have at least one and at most four children "
            "(currently has " + std::to_string(children_count) + ")");
  }

  if (status() == BT::NodeStatus::IDLE) {
    state_ = RESUMED;
  }
  setStatus(BT::NodeStatus::RUNNING);

  executor_->spin_some();

  // If pause / resume requested, reset children and switch to transient child
  if (state_ == PAUSE_REQUESTED || state_ == RESUME_REQUESTED) {
    resetChildren();
    switchToNextState();
  }

  // Return RUNNING and do nothing if specific child is not used
  const uint16_t child_idx = child_indices.at(state_);
  if (children_nodes_.size() <= child_idx) {
    switchToNextState();
    return BT::NodeStatus::RUNNING;
  }

  // If child is used, tick it
  const BT::NodeStatus child_status =
    children_nodes_[child_indices.at(state_)]->executeTick();

  switch (child_status) {
    case BT::NodeStatus::RUNNING:
      return BT::NodeStatus::RUNNING;
    case BT::NodeStatus::SUCCESS:
    case BT::NodeStatus::SKIPPED:
      if (state_ == RESUMED) {
        // Resumed child returned SUCCESS, we're done
        return BT::NodeStatus::SUCCESS;
      }
      switchToNextState();
      // If any branch other than RESUMED returned SUCCESS, keep ticking
      return BT::NodeStatus::RUNNING;
    case BT::NodeStatus::FAILURE:
      RCLCPP_ERROR(
        logger_, "%s child returned FAILURE", state_names.at(state_).c_str());
      return BT::NodeStatus::FAILURE;
    default:
      throw std::runtime_error("A child node must never return IDLE");
  }
}

void PauseResumeController::switchToNextState()
{
  static const std::map<state_t, state_t> next_states = {
    {PAUSE_REQUESTED, ON_PAUSE},
    {ON_PAUSE, PAUSED},
    {RESUME_REQUESTED, ON_RESUME},
    {ON_RESUME, RESUMED}
  };

  if (state_ == PAUSED || state_ == RESUMED) {
    // No next state, do nothing
    return;
  }

  state_ = next_states.at(state_);
  RCLCPP_INFO(logger_, "Switched to state: %s", state_names.at(state_).c_str());
}

void PauseResumeController::pauseServiceCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<Trigger::Request>/*request*/,
  std::shared_ptr<Trigger::Response> response)
{
  if (status() == BT::NodeStatus::IDLE) {
    std::string warn_msg = "PauseResumeController BT node has not been ticked yet";
    response->success = false;
    response->message = warn_msg;
    RCLCPP_ERROR(logger_, "%s", warn_msg.c_str());
    return;
  }

  if (state_ != PAUSED) {
    RCLCPP_INFO(logger_, "Received pause request");
    response->success = true;
    state_ = PAUSE_REQUESTED;
    return;
  }

  std::string warn_message = "PauseResumeController BT node already in state PAUSED";
  RCLCPP_WARN(logger_, "%s", warn_message.c_str());
  response->success = false;
  response->message = warn_message;
}

void PauseResumeController::resumeServiceCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<Trigger::Request>/*request*/,
  std::shared_ptr<Trigger::Response> response)
{
  if (status() == BT::NodeStatus::IDLE) {
    std::string warn_msg = "PauseResumeController BT node has not been ticked yet";
    response->success = false;
    response->message = warn_msg;
    RCLCPP_ERROR(logger_, "%s", warn_msg.c_str());
    return;
  }

  if (state_ == PAUSED) {
    RCLCPP_INFO(logger_, "Received resume request");
    response->success = true;
    state_ = RESUME_REQUESTED;
    return;
  }

  std::string warn_message = "PauseResumeController BT node not in state PAUSED";
  RCLCPP_WARN(logger_, "%s", warn_message.c_str());
  response->success = false;
  response->message = warn_message;
}

void PauseResumeController::halt()
{
  state_ = RESUMED;
  ControlNode::halt();
}

}  // namespace nav2_behavior_tree

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::PauseResumeController>(
        name, config);
    };

  factory.registerBuilder<nav2_behavior_tree::PauseResumeController>(
    "PauseResumeController", builder);
}
