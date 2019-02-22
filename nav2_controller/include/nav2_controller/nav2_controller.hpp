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

#ifndef NAV2_CONTROLLER__NAV2_CONTROLLER_HPP_
#define NAV2_CONTROLLER__NAV2_CONTROLLER_HPP_

#include <condition_variable>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_lifecycle/lifecycle_service_client.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

namespace nav2_controller
{

class Nav2Controller: public rclcpp::Node
{
public:
  Nav2Controller();
  ~Nav2Controller();

  void startup();
  void shutdown();
  void pause();
  void resume();

protected:
  rclcpp::callback_group::CallbackGroup::SharedPtr cb_grp_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr startup_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr shutdown_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr pause_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr resume_srv_;

  void startupCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);

  void shutdownCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);

  void pauseCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);

  void resumeCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);

  // Support functions for bring-up
  void createLifecycleServiceClients();
  void activateMapServer();
  void activateLocalizer();
  void activateWorldModel();
  void activateLocalPlanner();
  void activateRemainingNodes();
  void waitForInitialPose();

  // Support functions for shutdown
  void shutdownAllNodes();
  void destroyLifecycleServiceClients();

  // For each node in the map, transition to the new target state
  void changeStateForAllNodes(std::uint8_t transition);

  // A subscriber to listen for the first AMCL pose
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  void onPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  // A map of all nodes to be controlled
  std::map<std::string, std::shared_ptr<nav2_lifecycle::LifecycleServiceClient>> node_map;

  // Coordinate the receipt of the first AMCL pose
  bool pose_received_;
  std::mutex pose_mutex_;
  std::condition_variable cv_pose_;
};

}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__NAV2_CONTROLLER_HPP_
