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
using lifecycle_msgs::msg::State;
using nav2_util::LifecycleServiceClient;

namespace nav2_lifecycle_manager
{

LifecycleManager::LifecycleManager()
: Node("lifecycle_manager")
{
  RCLCPP_INFO(get_logger(), "Creating");

  // The list of names is parameterized, allowing this module to be used with a different set
  // of nodes
  declare_parameter("node_names");
  declare_parameter("autostart", rclcpp::ParameterValue(false));
  declare_parameter("bond_timeout_ms", 100);

  node_names_ = get_parameter("node_names").as_string_array();
  get_parameter("autostart", autostart_);
  int bond_timeout_int;
  get_parameter("bond_timeout_ms", bond_timeout_int);
  bond_timeout_ = std::chrono::milliseconds(bond_timeout_int);

  manager_srv_ = create_service<ManageLifecycleNodes>(
    get_name() + std::string("/manage_nodes"),
    std::bind(&LifecycleManager::managerCallback, this, _1, _2, _3));

  is_active_srv_ = create_service<std_srvs::srv::Trigger>(
    get_name() + std::string("/is_active"),
    std::bind(&LifecycleManager::isActiveCallback, this, _1, _2, _3));

  auto options = rclcpp::NodeOptions().arguments(
    {"--ros-args", "-r", std::string("__node:=") + get_name() + "_service_client", "--"});
  service_client_node_ = std::make_shared<rclcpp::Node>("_", options);

  transition_state_map_[Transition::TRANSITION_CONFIGURE] = State::PRIMARY_STATE_INACTIVE;
  transition_state_map_[Transition::TRANSITION_CLEANUP] = State::PRIMARY_STATE_UNCONFIGURED;
  transition_state_map_[Transition::TRANSITION_ACTIVATE] = State::PRIMARY_STATE_ACTIVE;
  transition_state_map_[Transition::TRANSITION_DEACTIVATE] = State::PRIMARY_STATE_INACTIVE;
  transition_state_map_[Transition::TRANSITION_UNCONFIGURED_SHUTDOWN] =
    State::PRIMARY_STATE_FINALIZED;

  transition_label_map_[Transition::TRANSITION_CONFIGURE] = std::string("Configuring ");
  transition_label_map_[Transition::TRANSITION_CLEANUP] = std::string("Cleaning up ");
  transition_label_map_[Transition::TRANSITION_ACTIVATE] = std::string("Activating ");
  transition_label_map_[Transition::TRANSITION_DEACTIVATE] = std::string("Deactivating ");
  transition_label_map_[Transition::TRANSITION_UNCONFIGURED_SHUTDOWN] =
    std::string("Shutting down ");

  createLifecycleServiceClients();

  if (autostart_) {
    startup();
  }
}

LifecycleManager::~LifecycleManager()
{
  RCLCPP_INFO(get_logger(), "Destroying");
}

void
LifecycleManager::managerCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<ManageLifecycleNodes::Request> request,
  std::shared_ptr<ManageLifecycleNodes::Response> response)
{
  switch (request->command) {
    case ManageLifecycleNodes::Request::STARTUP:
      response->success = startup();
      break;
    case ManageLifecycleNodes::Request::RESET:
      response->success = reset();
      break;
    case ManageLifecycleNodes::Request::SHUTDOWN:
      response->success = shutdown();
      break;
    case ManageLifecycleNodes::Request::PAUSE:
      response->success = pause();
      break;
    case ManageLifecycleNodes::Request::RESUME:
      response->success = resume();
      break;
  }
}

void
LifecycleManager::isActiveCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  response->success = system_active_;
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

bool
LifecycleManager::changeStateForNode(const std::string & node_name, std::uint8_t transition)
{
  message(transition_label_map_[transition] + node_name);
  if (!node_map_[node_name]->change_state(transition) ||
    !(node_map_[node_name]->get_state() == transition_state_map_[transition]))
  {
    RCLCPP_ERROR(get_logger(), "Failed to change state for node: %s", node_name.c_str());
    return false;
  }

  return true;
}

bool
LifecycleManager::changeStateForAllNodes(std::uint8_t transition)
{
  if (transition == Transition::TRANSITION_CONFIGURE ||
    transition == Transition::TRANSITION_ACTIVATE)
  {
    for (auto & node_name : node_names_) {
      if (!changeStateForNode(node_name, transition)) {
        return false;
      }
    }
  } else {
    std::vector<std::string>::reverse_iterator rit;
    for (rit = node_names_.rbegin(); rit != node_names_.rend(); ++rit) {
      if (!changeStateForNode(*rit, transition)) {
        return false;
      }
    }
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

bool
LifecycleManager::startup()
{
  message("Starting managed nodes bringup...");
  if (!changeStateForAllNodes(Transition::TRANSITION_CONFIGURE) ||
    !changeStateForAllNodes(Transition::TRANSITION_ACTIVATE))
  {
    RCLCPP_ERROR(get_logger(), "Failed to bring up all requested nodes. Aborting bringup.");
    return false;
  }
  message("Managed nodes are active");
  system_active_ = true;
  createBondTimer();
  return true;
}

bool
LifecycleManager::shutdown()
{
  message("Shutting down managed nodes...");
  shutdownAllNodes();
  destroyLifecycleServiceClients();
  message("Managed nodes have been shut down");
  system_active_ = false;
  destroyBondConnections();
  return true;
}

bool
LifecycleManager::reset()
{
  message("Resetting managed nodes...");
  // Should transition in reverse order
  if (!changeStateForAllNodes(Transition::TRANSITION_DEACTIVATE) ||
    !changeStateForAllNodes(Transition::TRANSITION_CLEANUP))
  {
    RCLCPP_ERROR(get_logger(), "Failed to reset nodes: aborting reset");
    return false;
  }
  message("Managed nodes have been reset");
  system_active_ = false;
  destroyBondConnections();

  return true;
}

bool
LifecycleManager::pause()
{
  message("Pausing managed nodes...");
  if (!changeStateForAllNodes(Transition::TRANSITION_DEACTIVATE)) {
    RCLCPP_ERROR(get_logger(), "Failed to pause nodes: aborting pause");
    return false;
  }
  message("Managed nodes have been paused");
  system_active_ = false;
  return true;
}

bool
LifecycleManager::resume()
{
  message("Resuming managed nodes...");
  if (!changeStateForAllNodes(Transition::TRANSITION_ACTIVATE)) {
    RCLCPP_ERROR(get_logger(), "Failed to resume nodes: aborting resume");
    return false;
  }
  message("Managed nodes are active");
  system_active_ = true;
  return true;
}

void
LifecycleManager::createBondTimer()
{
  if (bond_timeout_.count() <= 0) {
    return;
  }

  message("Creating bond timer...");

  bond_timer_ = this->create_wall_timer(
    100ms,
    std::bind(&LifecycleManager::checkBondConnections, this));
}

void
LifecycleManager::destroyBondConnections()
{
  bond_timer_.reset();

  if (bond_map_.empty()) {
    return;
  }

  message("Terminating bond connections...");

  for (auto & node_name : node_names_) {
    bond_map_[node_name]->breakBond();
  }

  bond_map_.clear();
}

void
LifecycleManager::createBondConnections()
{
  const double timeout_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(bond_timeout_).count();
  const double timeout_s = timeout_ns / 1e9;

  for (auto & node_name : node_names_) {
    bond_map_[node_name] =
      std::make_shared<bond::Bond>("bond", node_name, service_client_node_);
    auto & node_bond = bond_map_[node_name];
    node_bond->setHeartbeatTimeout(timeout_s);
    node_bond->start();

    RCLCPP_INFO(get_logger(), "Server %s trying to connect to bond....", node_name.c_str());
    node_bond->waitUntilFormed(rclcpp::Duration(timeout_ns));
    if (node_bond->isBroken()) {
      RCLCPP_ERROR(
        get_logger(),
        "Server %s was unable to be reached after %0.2fs by bond. "
        "This server may be misconfigured.",
        node_name.c_str(), timeout_s, node_name.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Server %s connected to bond!", node_name.c_str());
    }
  }
}

void
LifecycleManager::checkBondConnections()
{
  if (!system_active_) {
    return;
  }

  // populate the bond connections
  if (bond_map_.empty()) {
    createBondConnections();
  }

  for (auto & node_name : node_names_) {
    if (bond_map_[node_name]->isBroken()) {
      message(
        std::string(
        "Have not received a heartbeat from " + node_name + "."));

      // if down, destroy
      RCLCPP_ERROR(
        get_logger(),
        "CRITICAL FAILURE: SERVER %s IS DOWN after not receiving a heartbeat for %i ms."
        " Shutting down related nodes.",
        node_name.c_str(), static_cast<int>(bond_timeout_.count()));
      reset();
      return;
    }
  }
}

#define ANSI_COLOR_RESET    "\x1b[0m"
#define ANSI_COLOR_BLUE     "\x1b[34m"

void
LifecycleManager::message(const std::string & msg)
{
  RCLCPP_INFO(get_logger(), ANSI_COLOR_BLUE "\33[1m%s\33[0m" ANSI_COLOR_RESET, msg.c_str());
}

}  // namespace nav2_lifecycle_manager
