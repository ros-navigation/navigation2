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
#include <condition_variable>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"

#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"

using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace lifecycle_msgs::msg;

namespace nav2_controller
{

static void
message(const char * msg)
{
  fprintf(stderr, ANSI_COLOR_BLUE "\33[1m%s\33[0m" ANSI_COLOR_BLUE "\n" ANSI_COLOR_RESET, msg);
}

Nav2Controller::Nav2Controller()
: Node("nav2_controller")
{
  RCLCPP_INFO(get_logger(), "Creating");

  pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "amcl_pose", std::bind(&Nav2Controller::onPoseReceived, this, std::placeholders::_1));

  cb_grp_ = create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);

  startup_srv_ = create_service<std_srvs::srv::Empty>("startup",
      std::bind(&Nav2Controller::startupCallback, this, _1, _2, _3),
      rmw_qos_profile_services_default, cb_grp_);

  shutdown_srv_ = create_service<std_srvs::srv::Empty>("shutdown",
      std::bind(&Nav2Controller::shutdownCallback, this, _1, _2, _3),
      rmw_qos_profile_services_default, cb_grp_);

  pause_srv_ = create_service<std_srvs::srv::Empty>("pause",
      std::bind(&Nav2Controller::pauseCallback, this, _1, _2, _3),
      rmw_qos_profile_services_default, cb_grp_);

  resume_srv_ = create_service<std_srvs::srv::Empty>("resume",
      std::bind(&Nav2Controller::resumeCallback, this, _1, _2, _3),
      rmw_qos_profile_services_default, cb_grp_);
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
Nav2Controller::onPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr /*msg*/)
{
  std::unique_lock<std::mutex> lock(pose_mutex_);
  if (!pose_received_) {
    RCLCPP_INFO(get_logger(), "Received the initial pose");
  }
  pose_received_ = true;
}

void
Nav2Controller::createLifecycleServiceClients()
{
  message("Creating and initializing lifecycle service clients");
  rclcpp::Node::SharedPtr node = shared_from_this();

  node_map["amcl"] = std::make_shared<nav2_lifecycle::LifecycleServiceClient>(node, "amcl");
  node_map["map_server"] = std::make_shared<nav2_lifecycle::LifecycleServiceClient>(node, "map_server");
  node_map["dwb_controller"] = std::make_shared<nav2_lifecycle::LifecycleServiceClient>(node, "dwb_controller");
  node_map["navfn_planner"] = std::make_shared<nav2_lifecycle::LifecycleServiceClient>(node, "navfn_planner");
  node_map["simple_navigator"] = std::make_shared<nav2_lifecycle::LifecycleServiceClient>(node, "simple_navigator");
  node_map["world_model"] = std::make_shared<nav2_lifecycle::LifecycleServiceClient>(node, "world_model");
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
    if (!kv.second->changeState(transition)) {
      fprintf(stderr, ANSI_COLOR_RED "Failed to change state for %s\n" ANSI_COLOR_RESET, kv.first.c_str());
      return;
    }
  }
}

void
Nav2Controller::activateMapServer()
{
  message("Configuring and activating the MapServer");
  node_map["map_server"]->changeState(Transition::TRANSITION_CONFIGURE);
  node_map["map_server"]->changeState(Transition::TRANSITION_ACTIVATE);
}

void
Nav2Controller::activateLocalizer()
{
  message("Configuring and activating AMCL");
  node_map["amcl"]->changeState(Transition::TRANSITION_CONFIGURE);
  node_map["amcl"]->changeState(Transition::TRANSITION_ACTIVATE);
}

void
Nav2Controller::waitForInitialPose()
{
  message("Waiting for first AMCL pose");
  do {
    std::unique_lock<std::mutex> lock(pose_mutex_);
    if (cv_pose_.wait_for(lock, std::chrono::milliseconds(10),
      [&] {return pose_received_ == true;}))
    {
      message("Received initial pose from AMCL");
      pose_received_ = false;
      return;
    }
  } while (true);
}

void
Nav2Controller::activateWorldModel()
{
  message("Configuring and activating the world model");

  node_map["world_model"]->changeState(Transition::TRANSITION_CONFIGURE);
  node_map["world_model"]->changeState(Transition::TRANSITION_ACTIVATE);
}

void
Nav2Controller::activateLocalPlanner()
{
  message("Configuring and activating DWB");

  node_map["dwb_controller"]->changeState(Transition::TRANSITION_CONFIGURE);
  node_map["dwb_controller"]->changeState(Transition::TRANSITION_ACTIVATE);
}

void
Nav2Controller::activateRemainingNodes()
{
  message("Configuring and activating the remaining nodes");

  node_map["navfn_planner"]->changeState(Transition::TRANSITION_CONFIGURE);
  node_map["navfn_planner"]->changeState(Transition::TRANSITION_ACTIVATE);

  node_map["simple_navigator"]->changeState(Transition::TRANSITION_CONFIGURE);
  node_map["simple_navigator"]->changeState(Transition::TRANSITION_ACTIVATE);
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
  waitForInitialPose();
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
    if (!kv.second->changeState(Transition::TRANSITION_DEACTIVATE)) {
      fprintf(stderr, ANSI_COLOR_RED "Failed to change state for %s\n" ANSI_COLOR_RESET, kv.first.c_str());
      return;
    }
    if (!kv.second->changeState(Transition::TRANSITION_CLEANUP)) {
      fprintf(stderr, ANSI_COLOR_RED "Failed to change state for %s\n" ANSI_COLOR_RESET, kv.first.c_str());
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
  waitForInitialPose();
  activateWorldModel();
  activateLocalPlanner();
  activateRemainingNodes();
  message("The system has been resumed");
}

}  // namespace nav2_controller
