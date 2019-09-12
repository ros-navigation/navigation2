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
#include <unordered_map>
#include <vector>

#include "nav2_util/lifecycle_service_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"

namespace nav2_lifecycle_manager
{

using nav2_msgs::srv::ManageLifecycleNodes;

class LifecycleManager : public rclcpp::Node
{
public:
  LifecycleManager();
  ~LifecycleManager();

protected:
  // The ROS node to use when calling lifecycle services
  rclcpp::Node::SharedPtr service_client_node_;

  // The services provided by this node
  rclcpp::Service<ManageLifecycleNodes>::SharedPtr manager_srv_;

  void managerCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ManageLifecycleNodes::Request> request,
    std::shared_ptr<ManageLifecycleNodes::Response> response);

  // Support functions for the service calls
  bool startup();
  bool shutdown();
  bool reset();
  bool pause();
  bool resume();

  // Support function for creating service clients
  void createLifecycleServiceClients();

  // Support functions for shutdown
  void shutdownAllNodes();
  void destroyLifecycleServiceClients();

  // For a node, transition to the new target state
  bool changeStateForNode(const std::string & node_name, std::uint8_t transition);

  // For each node in the map, transition to the new target state
  bool changeStateForAllNodes(std::uint8_t transition, bool reverse_order = false);

  // Convenience function to highlight the output on the console
  void message(const std::string & msg);

  // A map of all nodes to be controlled
  std::map<std::string, std::shared_ptr<nav2_util::LifecycleServiceClient>> node_map_;

  std::map<std::uint8_t, std::string> transition_label_map_;

  // A map of the expected transitions to primary states
  std::unordered_map<std::uint8_t, std::uint8_t> transition_state_map_;

  // The names of the nodes to be managed, in the order of desired bring-up
  std::vector<std::string> node_names_;

  // Whether to automatically start up the system
  bool autostart_;
};

}  // namespace nav2_lifecycle_manager

#endif  // NAV2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_
