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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__PAUSE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__PAUSE_HPP_

// Other includes
#include <string>
#include <memory>
#include <mutex>

// ROS includes
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/control_node.h"

// Interface definitions
#include "std_srvs/srv/trigger.hpp"


namespace nav2_behavior_tree
{

using Trigger = std_srvs::srv::Trigger;

enum state_t {UNPAUSED, PAUSED, PAUSE_REQUESTED, ON_PAUSE, RESUME_REQUESTED, ON_RESUME};

/* @brief Controlled through service calls to pause and resume the execution of the tree
 * It has one mandatory child for the UNPAUSED, and three optional for the PAUSED state,
 * the ON_PAUSE event and the ON_RESUME event.
 * It has two input ports:
 * - pause_service_name: name of the service to pause
 * - resume_service_name: name of the service to resume
 */
class Pause : public BT::ControlNode
{
public:
  //! @brief Constructor
  Pause(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  //! @brief Destructor
  ~Pause();

  //! @brief Reset state and go to Idle
  void halt() override;

  //! @brief Return a NodeStatus according to the children's status
  BT::NodeStatus tick() override;

  //! @brief Declare ports
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "pause_service_name",
        "Name of the service to pause"),
      BT::InputPort<std::string>(
        "resume_service_name",
        "Name of the service to resume"),
    };
  }

private:
  //! @brief Service callback to pause
  void pause_service_callback(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response);

  //! @brief Service callback to resume
  void resume_service_callback(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::unique_ptr<std::thread> spinner_thread_;
  rclcpp::Service<Trigger>::SharedPtr pause_srv_;
  rclcpp::Service<Trigger>::SharedPtr resume_srv_;
  state_t state_;
  std::mutex state_mutex_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__PAUSE_HPP_
