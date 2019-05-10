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

#include "nav2_lifecycle_manager/lifecycle_manager.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

using lifecycle_msgs::msg::Transition;
using nav2_util::LifecycleServiceClient;

namespace nav2_lifecycle_manager
{

LifecycleManager::LifecycleManager()
: Node("lifecycle_manager")
{
  RCLCPP_INFO(get_logger(), "Creating");

  // The default set of node names for the nav2 stack
  std::vector<std::string> default_node_names{"map_server", "amcl", "world_model", "dwb_controller",
    "navfn_planner", "bt_navigator"};

  // The list of names is parameterized, allowing this module to be used with a different set of nodes
  declare_parameter("node_names", rclcpp::ParameterValue(default_node_names));
  declare_parameter("autostart", rclcpp::ParameterValue(false));

  get_parameter("node_names", node_names_);
  get_parameter("autostart", autostart_);

  startup_srv_ = create_service<std_srvs::srv::Empty>("lifecycle_manager/startup",
      std::bind(&LifecycleManager::startupCallback, this, _1, _2, _3));

  shutdown_srv_ = create_service<std_srvs::srv::Empty>("lifecycle_manager/shutdown",
      std::bind(&LifecycleManager::shutdownCallback, this, _1, _2, _3));

  pause_srv_ = create_service<std_srvs::srv::Empty>("lifecycle_manager/pause",
      std::bind(&LifecycleManager::pauseCallback, this, _1, _2, _3));

  resume_srv_ = create_service<std_srvs::srv::Empty>("lifecycle_manager/resume",
      std::bind(&LifecycleManager::resumeCallback, this, _1, _2, _3));

  service_client_node_ = std::make_shared<rclcpp::Node>("lifecycle_manager_service_client");

  if (autostart_) {
    startup();
  }
}

LifecycleManager::~LifecycleManager()
{
  RCLCPP_INFO(get_logger(), "Destroying");
}

void
LifecycleManager::startupCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::Empty::Request>/*request*/,
  std::shared_ptr<std_srvs::srv::Empty::Response>/*response*/)
{
  startup();
}

void
LifecycleManager::shutdownCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::Empty::Request>/*request*/,
  std::shared_ptr<std_srvs::srv::Empty::Response>/*response*/)
{
  shutdown();
}

void
LifecycleManager::pauseCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::Empty::Request>/*request*/,
  std::shared_ptr<std_srvs::srv::Empty::Response>/*response*/)
{
  pause();
}

void
LifecycleManager::resumeCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::Empty::Request>/*request*/,
  std::shared_ptr<std_srvs::srv::Empty::Response>/*response*/)
{
  resume();
}

void
LifecycleManager::createLifecycleServiceClients()
{
  message("Creating and initializing lifecycle service clients");
  for (auto & node_name : node_names_) {
    node_map_[node_name] =
      std::make_shared<LifecycleServiceClient>(node_name, service_client_node_);
  }
}

void
LifecycleManager::destroyLifecycleServiceClients()
{
  message("Destroying lifecycle service clients");
  for (auto & kv : node_map_) {
    kv.second.reset();
  }
}

void
LifecycleManager::changeStateForAllNodes(std::uint8_t transition)
{
  for (const auto & kv : node_map_) {
    if (!kv.second->change_state(transition)) {
      RCLCPP_ERROR(get_logger(), "Failed to change state for node: %s", kv.first.c_str());
      return;
    }
  }
}

bool
LifecycleManager::bringupNode(const std::string & node_name)
{
  message(std::string("Configuring and activating ") + node_name);
  if (!node_map_[node_name]->change_state(Transition::TRANSITION_CONFIGURE)) {
    RCLCPP_ERROR(get_logger(), "Failed to configure node: %s", node_name.c_str());
    return false;
  }

  auto rc = node_map_[node_name]->change_state(Transition::TRANSITION_ACTIVATE);
  if (!rc) {
    RCLCPP_ERROR(get_logger(), "Failed to activate node: %s", node_name.c_str());
    return false;
  }

  return true;
}

void
LifecycleManager::shutdownAllNodes()
{
  message("Deactivate, cleanup, and shutdown nodes");
  changeStateForAllNodes(Transition::TRANSITION_DEACTIVATE);
  changeStateForAllNodes(Transition::TRANSITION_CLEANUP);
  changeStateForAllNodes(Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
}

void
LifecycleManager::startup()
{
  message("Starting the system bringup...");
  createLifecycleServiceClients();
  for (auto & node_name : node_names_) {
    if (!bringupNode(node_name)) {
      RCLCPP_ERROR(get_logger(), "Failed to bring up node: %s, aboring bringup", node_name.c_str());
      return;
    }
  }
  message("The system is active");
}

void
LifecycleManager::shutdown()
{
  message("Shutting down the system...");
  shutdownAllNodes();
  destroyLifecycleServiceClients();
  message("The system has been sucessfully shut down");
}

void
LifecycleManager::pause()
{
  message("Pausing the system...");
  for (const auto & kv : node_map_) {
    if (!kv.second->change_state(Transition::TRANSITION_DEACTIVATE) ||
      !kv.second->change_state(Transition::TRANSITION_CLEANUP))
    {
      RCLCPP_ERROR(get_logger(), "Failed to change state for node: %s", kv.first.c_str());
      return;
    }
  }
  message("The system has been paused");
}

void
LifecycleManager::resume()
{
  message("Resuming the system...");
  for (auto & node_name : node_names_) {
    if (!bringupNode(node_name)) {
      RCLCPP_ERROR(get_logger(), "Failed to resume node: %s, aborting", node_name.c_str());
      return;
    }
  }
  message("The system has been resumed");
}

// TODO(mjeronimo): This is used to emphasize the major events during system bring-up and
// shutdown so that the messgaes can be easily seen among the log output. We should replace
// this with a ROS2-supported way of highlighting console output, if possible.

#define ANSI_COLOR_RESET    "\x1b[0m"
#define ANSI_COLOR_BLUE     "\x1b[34m"

void
LifecycleManager::message(const std::string & msg)
{
  RCLCPP_INFO(get_logger(), ANSI_COLOR_BLUE "\33[1m%s\33[0m" ANSI_COLOR_RESET, msg.c_str());
}

}  // namespace nav2_lifecycle_manager
