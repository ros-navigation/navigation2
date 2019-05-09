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

#ifndef NAV2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_
#define NAV2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "nav2_util/lifecycle_service_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

namespace nav2_lifecycle_manager
{

class LifecycleManager : public rclcpp::Node
{
public:
  LifecycleManager();
  ~LifecycleManager();

protected:
  // The ROS node to use when calling lifecycle services
  rclcpp::Node::SharedPtr service_client_node_;

  // The services provided by this node
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

  // Support functions for the service calls
  void startup();
  void shutdown();
  void pause();
  void resume();

  // Support functions for bring-up
  void createLifecycleServiceClients();
  bool bringupNode(const std::string & node_name);

  // Support functions for shutdown
  void shutdownAllNodes();
  void destroyLifecycleServiceClients();

  // For each node in the map, transition to the new target state
  void changeStateForAllNodes(std::uint8_t transition);

  // Convenience function to highlight the output on the console
  void message(const std::string & msg);

  // A map of all nodes to be controlled
  std::map<std::string, std::shared_ptr<nav2_util::LifecycleServiceClient>> node_map_;

  // The names of the nodes to be managed, in the order of desired bring-up
  std::vector<std::string> node_names_;
};

}  // namespace nav2_lifecycle_manager

#endif  // NAV2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_
