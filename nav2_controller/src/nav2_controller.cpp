// Copyright 2016 Open Source Robotics Foundation, Inc.
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
#include <thread>

#include "rclcpp/rclcpp.hpp"

#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"

using namespace std::chrono_literals;
using namespace std::placeholders;

using lifecycle_msgs::msg::Transition;
using nav2_util::LifecycleServiceClient;

namespace nav2_controller
{

static void
message(const char * msg)
{
  fprintf(stderr, ANSI_COLOR_BLUE "\33[1m%s\33[0m" ANSI_COLOR_RESET "\n", msg);
}

Nav2Controller::Nav2Controller()
: Node("nav2_controller")
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

  client_ = std::make_shared<rclcpp::Node>("nav2_controller_lifecycle_client_node");
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
  RCLCPP_INFO(get_logger(), "startupCallback");
  startup();
}

void
Nav2Controller::shutdownCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::Empty::Request>/*req*/,
  std::shared_ptr<std_srvs::srv::Empty::Response>/*res*/)
{
  RCLCPP_INFO(get_logger(), "shutdownCallback");
  shutdown();
}

void
Nav2Controller::pauseCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::Empty::Request>/*req*/,
  std::shared_ptr<std_srvs::srv::Empty::Response>/*res*/)
{
  RCLCPP_INFO(get_logger(), "pauseCallback");
  pause();
}

void
Nav2Controller::resumeCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::Empty::Request>/*req*/,
  std::shared_ptr<std_srvs::srv::Empty::Response>/*res*/)
{
  RCLCPP_INFO(get_logger(), "resumeCallback");
  resume();
}

void
Nav2Controller::createLifecycleServiceClients()
{
  message("Creating and initializing lifecycle service clients");

  node_map["amcl"] = std::make_shared<LifecycleServiceClient>("amcl", client_);
  node_map["map_server"] = std::make_shared<LifecycleServiceClient>("map_server", client_);
  node_map["dwb_controller"] = std::make_shared<LifecycleServiceClient>("dwb_controller", client_);
  node_map["navfn_planner"] = std::make_shared<LifecycleServiceClient>("navfn_planner", client_);
  node_map["bt_navigator"] = std::make_shared<LifecycleServiceClient>("bt_navigator", client_);
  node_map["world_model"] = std::make_shared<LifecycleServiceClient>("world_model", client_);
}

void
Nav2Controller::destroyLifecycleServiceClients()
{
  message("Destroying lifecycle service clients");
  for (auto & kv : node_map) {
    kv.second.reset();
  }
}

void
Nav2Controller::changeStateForAllNodes(std::uint8_t transition)
{
  for (const auto & kv : node_map) {
    if (!kv.second->change_state(transition)) {
      fprintf(stderr,
        ANSI_COLOR_RED "Failed to change state for %s\n" ANSI_COLOR_RESET, kv.first.c_str());
      return;
    }
  }
}

void
Nav2Controller::activateMapServer()
{
  message("Configuring and activating the MapServer");
  node_map["map_server"]->change_state(Transition::TRANSITION_CONFIGURE);
  node_map["map_server"]->change_state(Transition::TRANSITION_ACTIVATE);
}

void
Nav2Controller::activateLocalizer()
{
  message("Configuring and activating AMCL");
  node_map["amcl"]->change_state(Transition::TRANSITION_CONFIGURE);
  node_map["amcl"]->change_state(Transition::TRANSITION_ACTIVATE);
}

void
Nav2Controller::activateWorldModel()
{
  message("Configuring and activating the world model");

  node_map["world_model"]->change_state(Transition::TRANSITION_CONFIGURE);
  node_map["world_model"]->change_state(Transition::TRANSITION_ACTIVATE);
}

void
Nav2Controller::activateLocalPlanner()
{
  message("Configuring and activating DWB");

  node_map["dwb_controller"]->change_state(Transition::TRANSITION_CONFIGURE);
  node_map["dwb_controller"]->change_state(Transition::TRANSITION_ACTIVATE);
}

void
Nav2Controller::activateRemainingNodes()
{
  message("Configuring and activating the global planner");

  node_map["navfn_planner"]->change_state(Transition::TRANSITION_CONFIGURE);
  node_map["navfn_planner"]->change_state(Transition::TRANSITION_ACTIVATE);

  message("Configuring and activating the navigator");

  node_map["bt_navigator"]->change_state(Transition::TRANSITION_CONFIGURE);
  node_map["bt_navigator"]->change_state(Transition::TRANSITION_ACTIVATE);
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
  activateMapServer();
  activateLocalizer();
  activateWorldModel();
  activateLocalPlanner();
  activateRemainingNodes();
  message("The system is active");
}

void
Nav2Controller::shutdown()
{
  message("Shutting down the system...");
  shutdownAllNodes();
  destroyLifecycleServiceClients();
  message("The system has been sucessfully shut down");
  std::this_thread::sleep_for(2s);
}

void
Nav2Controller::pause()
{
  message("Pausing the system...");
  for (const auto & kv : node_map) {
    if (!kv.second->change_state(Transition::TRANSITION_DEACTIVATE)) {
      fprintf(stderr,
        ANSI_COLOR_RED "Failed to change state for %s\n" ANSI_COLOR_RESET, kv.first.c_str());
      return;
    }
    if (!kv.second->change_state(Transition::TRANSITION_CLEANUP)) {
      fprintf(stderr,
        ANSI_COLOR_RED "Failed to change state for %s\n" ANSI_COLOR_RESET, kv.first.c_str());
      return;
    }
  }
  message("The system has been paused");
}

void
Nav2Controller::resume()
{
  message("Resuming the system...");
  activateMapServer();
  activateLocalizer();
  activateWorldModel();
  activateLocalPlanner();
  activateRemainingNodes();
  message("The system has been resumed");
}

}  // namespace nav2_controller
