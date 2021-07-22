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
#include <thread>
#include <unordered_map>
#include <vector>

#include "nav2_util/lifecycle_service_client.hpp"
#include "nav2_util/node_thread.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "bondcpp/bond.hpp"

namespace nav2_lifecycle_manager
{

using nav2_msgs::srv::ManageLifecycleNodes;
/**
 * @class nav2_lifecycle_manager::LifecycleManager
 * @brief Implements service interface to transition the lifecycle nodes of
 * Nav2 stack. It receives transition request and then uses lifecycle
 * interface to change lifecycle node's state.
 */
class LifecycleManager : public rclcpp::Node
{
public:
  /**
   * @brief A constructor for nav2_lifecycle_manager::LifecycleManager
   */
  LifecycleManager();
  /**
   * @brief A destructor for nav2_lifecycle_manager::LifecycleManager
   */
  ~LifecycleManager();

protected:
  // Callback group used by services and timers
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::unique_ptr<nav2_util::NodeThread> service_thread_;

  // The services provided by this node
  rclcpp::Service<ManageLifecycleNodes>::SharedPtr manager_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr is_active_srv_;
  /**
   * @brief Lifecycle node manager callback function
   * @param request_header Header of the service request
   * @param request Service request
   * @param reponse Service response
   */
  void managerCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ManageLifecycleNodes::Request> request,
    std::shared_ptr<ManageLifecycleNodes::Response> response);
  /**
   * @brief Trigger callback function checks if the managed nodes are in active
   * state.
   * @param request_header Header of the request
   * @param request Service request
   * @param reponse Service response
   */
  void isActiveCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // Support functions for the service calls
  /**
   * @brief Start up managed nodes.
   * @return true or false
   */
  bool startup();
  /**
   * @brief Deactivate, clean up and shut down all the managed nodes.
   * @return true or false
   */
  bool shutdown();
  /**
   * @brief Reset all the managed nodes.
   * @return true or false
   */
  bool reset();
  /**
   * @brief Pause all the managed nodes.
   * @return true or false
   */
  bool pause();
  /**
   * @brief Resume all the managed nodes.
   * @return true or false
   */
  bool resume();

  // Support function for creating service clients
  /**
   * @brief Support function for creating service clients
   */
  void createLifecycleServiceClients();

  // Support functions for shutdown
  /**
   * @brief Support function for shutdown
   */
  void shutdownAllNodes();
  /**
   * @brief Destroy all the lifecycle service clients.
   */
  void destroyLifecycleServiceClients();

  // Support function for creating bond timer
  /**
   * @brief Support function for creating bond timer
   */
  void createBondTimer();

  // Support function for creating bond connection
  /**
   * @brief Support function for creating bond connections
   */
  bool createBondConnection(const std::string & node_name);

  // Support function for killing bond connections
  /**
   * @brief Support function for killing bond connections
   */
  void destroyBondTimer();

  // Support function for checking on bond connections
  /**
   * @ brief Support function for checking on bond connections
   * will take down system if there's something non-responsive
   */
  void checkBondConnections();

  /**
   * @brief For a node, transition to the new target state
   */
  bool changeStateForNode(
    const std::string & node_name,
    std::uint8_t transition);

  /**
   * @brief For each node in the map, transition to the new target state
   */
  bool changeStateForAllNodes(std::uint8_t transition);

  // Convenience function to highlight the output on the console
  /**
   * @brief Helper function to highlight the output on the console
   */
  void message(const std::string & msg);

  // Timer thread to look at bond connections
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr bond_timer_;
  std::chrono::milliseconds bond_timeout_;

  // A map of all nodes to check bond connection
  std::map<std::string, std::shared_ptr<bond::Bond>> bond_map_;

  // A map of all nodes to be controlled
  std::map<std::string, std::shared_ptr<nav2_util::LifecycleServiceClient>> node_map_;

  std::map<std::uint8_t, std::string> transition_label_map_;

  // A map of the expected transitions to primary states
  std::unordered_map<std::uint8_t, std::uint8_t> transition_state_map_;

  // The names of the nodes to be managed, in the order of desired bring-up
  std::vector<std::string> node_names_;

  // Whether to automatically start up the system
  bool autostart_;

  bool system_active_{false};
};

}  // namespace nav2_lifecycle_manager

#endif  // NAV2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_
