// Copyright (c) 2024 Open Navigation LLC
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

#ifndef OPENNAV_DOCKING__DOCK_DATABASE_HPP_
#define OPENNAV_DOCKING__DOCK_DATABASE_HPP_

#include <vector>
#include <memory>
#include <string>
#include <mutex>
#include <unordered_map>

#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_ros_common/service_server.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/node_utils.hpp"
#include "nav2_ros_common/simple_action_server.hpp"
#include "opennav_docking/utils.hpp"
#include "opennav_docking/types.hpp"
#include "opennav_docking/controller_base.hpp"
#include "nav2_msgs/srv/reload_dock_database.hpp"
#include "nav2_ros_common/tf2_factories.hpp"

namespace opennav_docking
{

using ControllerMap = std::unordered_map<std::string, ControllerBase::Ptr>;

/**
 * @class opennav_docking::DockDatabase
 * @brief An object to contain docks, dock plugins and docking controller plugins
 */
class DockDatabase
{
public:
  /**
   * @brief A constructor for opennav_docking::DockDatabase
   */
  explicit DockDatabase(std::mutex & mutex);

  /**
   * @brief A setup function to populate database
   * @param parent Weakptr to the node to use to get interances and parameters
   * @param tf TF buffer
   * @return If successful
   */
  bool initialize(
    const nav2::LifecycleNode::WeakPtr & parent, nav2::TransformBuffer::SharedPtr tf);

  /**
   * @brief A destructor for opennav_docking::DockDatabase
   */
  ~DockDatabase();

  /**
   * @brief An activation method
   */
  void activate();

  /**
   * @brief An deactivation method
   */
  void deactivate();

  /**
   * @brief Find a dock instance & plugin in the databases from ID
   * @param dock_id Id of dock to find
   * @return Dock pointer
   */
  Dock * findDock(const std::string & dock_id);

  /**
   * @brief Find a dock plugin to use for a given type
   * @param type Dock type to find plugin for
   * @return Dock plugin pointer
   */
  ChargingDock::Ptr findDockPlugin(const std::string & type);

  /**
   * @brief Find the loaded controller plugin
   * @param requested_controller_name Controller name
   * @param type Dock type whose controller is used when requested is empty
   * @return Controller plugin pointer
   * @throw DockNotValid if the controller is not loaded
   */
  ControllerBase::Ptr findController(
    const std::string & requested_controller_name, const std::string & type) const;

  /**
   * @brief Get the number of docks in the database
   * @return unsigned int Number of dock instances in the database
   */
  unsigned int instance_size() const;

  /**
   * @brief Get the number of dock types in the database
   * @return unsigned int Number of dock types in the database
   */
  unsigned int plugin_size() const;

  /**
   * @brief Get the number of loaded controller plugins
   * @return unsigned int Number of controller plugins including the default
   */
  unsigned int controller_size() const;

protected:
  /**
   * @brief Get the controller to use
   * @param requested_controller_name Controller name requested
   * @param type Dock type whose controller used when requested is empty
   * @return Controller name: requested one, else dock type's, else the default
   */
  std::string getControllerName(
    const std::string & requested_controller_name, const std::string & type) const;

  /**
   * @brief Populate database of dock type plugins
   * @param Node Node to get values from
   * @param tf TF buffer
   * @return bool If successful
   */
  bool getDockPlugins(
    const nav2::LifecycleNode::SharedPtr & node, nav2::TransformBuffer::SharedPtr tf);

  /**
   * @brief Populate database of dock instances
   * @param Node Node to get values from
   */
  bool getDockInstances(const nav2::LifecycleNode::SharedPtr & node);

  /**
   * @brief Create default graceful `controller` and the plugins listed
   * in the `controllers` parameter
   * @param node Node to get values from
   * @param tf TF buffer
   * @return bool If every controller is created and configured
   */
  bool getControllerPlugins(
    const nav2::LifecycleNode::SharedPtr & node, nav2::TransformBuffer::SharedPtr tf);

  /**
   * @brief Check every controller name in the database against the loaded controllers.
   *
   * @param docks Dock instances to check, against the already-loaded dock plugins
   * @return True if every dock controller and every non-empty dock type controller is loaded
   */
  bool validateControllersExist(const DockMap & docks) const;

  /**
   * @brief Give every dock without a controller its type's controller, else the default
   * @param docks Dock instances to update
   */
  void assignControllers(DockMap & docks) const;

  /**
   * @brief Find a dock instance in the database from ID
   * @param dock_id Id of dock to find
   * @return Dock pointer
   */
  Dock * findDockInstance(const std::string & dock_id);

  /**
   * @brief Service request to reload database of docks
   * @param request Service request
   * @param response Service response
   */
  void reloadDbCb(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::ReloadDockDatabase::Request> request,
    std::shared_ptr<nav2_msgs::srv::ReloadDockDatabase::Response> response);

  nav2::LifecycleNode::WeakPtr node_;
  std::mutex & mutex_;  // Don't reload database while actively docking
  DockPluginMap dock_plugins_;
  std::unordered_map<std::string, std::string> type_controllers_;
  DockMap dock_instances_;
  pluginlib::ClassLoader<opennav_docking_core::ChargingDock> dock_loader_;
  // Declared before the map so that it is destructed after controllers
  pluginlib::ClassLoader<ControllerBase> controller_loader_;
  ControllerMap controllers_;
  nav2::ServiceServer<nav2_msgs::srv::ReloadDockDatabase>::SharedPtr reload_db_service_;
};

}  // namespace opennav_docking

#endif  // OPENNAV_DOCKING__DOCK_DATABASE_HPP_
