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

#include "nav2_controller/nav2_controller.hpp"

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#define ANSI_COLOR_RED      "\x1b[31m"
#define ANSI_COLOR_YELLOW   "\x1b[33m"
#define ANSI_COLOR_BLUE     "\x1b[34m"
#define ANSI_COLOR_RESET    "\x1b[0m"

using namespace std::chrono_literals;
using namespace std::placeholders;

using lifecycle_msgs::msg::Transition;
using nav2_util::LifecycleServiceClient;

namespace nav2_controller
{

Nav2Controller::Nav2Controller()
: Node("nav2_controller"),
  node_names_{"map_server", "amcl", "world_model", "dwb_controller", "navfn_planner",
    "bt_navigator"}
{
  RCLCPP_INFO(get_logger(), "Creating");

  startup_srv_ = create_service<std_srvs::srv::Empty>("nav2_controller/startup",
      std::bind(&Nav2Controller::startupCallback, this, _1, _2, _3));

  shutdown_srv_ = create_service<std_srvs::srv::Empty>("nav2_controller/shutdown",
      std::bind(&Nav2Controller::shutdownCallback, this, _1, _2, _3));

  pause_srv_ = create_service<std_srvs::srv::Empty>("nav2_controller/pause",
      std::bind(&Nav2Controller::pauseCallback, this, _1, _2, _3));

  resume_srv_ = create_service<std_srvs::srv::Empty>("nav2_controller/resume",
      std::bind(&Nav2Controller::resumeCallback, this, _1, _2, _3));

  service_client_node_ = std::make_shared<rclcpp::Node>("nav2_controller_service_client_node");
}

Nav2Controller::~Nav2Controller()
{
  RCLCPP_INFO(get_logger(), "Destroying");
}

void
Nav2Controller::startupCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::Empty::Request>/*req*/,
  std::shared_ptr<std_srvs::srv::Empty::Response>/*res*/)
{
  startup();
}

void
Nav2Controller::shutdownCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::Empty::Request>/*req*/,
  std::shared_ptr<std_srvs::srv::Empty::Response>/*res*/)
{
  shutdown();
}

void
Nav2Controller::pauseCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::Empty::Request>/*req*/,
  std::shared_ptr<std_srvs::srv::Empty::Response>/*res*/)
{
  pause();
}

void
Nav2Controller::resumeCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::Empty::Request>/*req*/,
  std::shared_ptr<std_srvs::srv::Empty::Response>/*res*/)
{
  resume();
}

void
Nav2Controller::createLifecycleServiceClients()
{
  message("Creating and initializing lifecycle service clients");
  for (auto & node_name : node_names_) {
    node_map_[node_name] =
      std::make_shared<LifecycleServiceClient>(node_name, service_client_node_);
  }
}

void
Nav2Controller::destroyLifecycleServiceClients()
{
  message("Destroying lifecycle service clients");
  for (auto & kv : node_map_) {
    kv.second.reset();
  }
}

void
Nav2Controller::changeStateForAllNodes(std::uint8_t transition)
{
  for (const auto & kv : node_map_) {
    if (!kv.second->change_state(transition)) {
      std::string error_msg = std::string("Failed to change state for ") + kv.first.c_str();
      error(error_msg);
      return;
    }
  }
}

bool
Nav2Controller::bringupNode(const std::string & node_name)
{
  message(std::string("Configuring and activating ") + node_name);
  if (!node_map_[node_name]->change_state(Transition::TRANSITION_CONFIGURE)) {
    std::string error_msg = std::string("Failed to configure ") + node_name;
    error(error_msg);
    return false;
  }

  auto rc = node_map_[node_name]->change_state(Transition::TRANSITION_ACTIVATE);
  if (!rc) {
    std::string error_msg = std::string("Failed to activate ") + node_name;
    error(error_msg);
    return false;
  }

  return true;
}

void
Nav2Controller::shutdownAllNodes()
{
  message("Deactivate, cleanup, and shutdown nodes");
  changeStateForAllNodes(Transition::TRANSITION_DEACTIVATE);
  changeStateForAllNodes(Transition::TRANSITION_CLEANUP);
  changeStateForAllNodes(Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
}

void
Nav2Controller::startup()
{
  message("Starting the system bringup...");
  createLifecycleServiceClients();
  for (auto & node_name : node_names_) {
    if (!bringupNode(node_name)) {
      std::string error_msg =
        std::string("Failed to bring up node: ") + node_name + ", aborting bringup";
      error(error_msg);
    }
  }
  message("The system is active");
}

void
Nav2Controller::shutdown()
{
  message("Shutting down the system...");
  shutdownAllNodes();
  destroyLifecycleServiceClients();
  message("The system has been sucessfully shut down");
}

void
Nav2Controller::pause()
{
  message("Pausing the system...");
  for (const auto & kv : node_map_) {
    if (!kv.second->change_state(Transition::TRANSITION_DEACTIVATE) ||
      !kv.second->change_state(Transition::TRANSITION_CLEANUP))
    {
      std::string error_msg = std::string("Failed to change state for ") + kv.first.c_str();
      error(error_msg);
      return;
    }
  }
  message("The system has been paused");
}

void
Nav2Controller::resume()
{
  message("Resuming the system...");
  for (auto & node_name : node_names_) {
    if (!bringupNode(node_name)) {
      std::string error_msg = std::string("Failed to resume node: ") + node_name + ", aborting";
      error(error_msg);
    }
  }
  message("The system has been resumed");
}

void
Nav2Controller::message(const std::string & msg)
{
  RCLCPP_INFO(get_logger(), ANSI_COLOR_BLUE "\33[1m%s\33[0m" ANSI_COLOR_RESET, msg.c_str());
}

void
Nav2Controller::error(const std::string & error_msg)
{
  RCLCPP_INFO(get_logger(), ANSI_COLOR_RED "\33[1m%s\33[0m" ANSI_COLOR_RESET, error_msg.c_str());
}

}  // namespace nav2_controller
