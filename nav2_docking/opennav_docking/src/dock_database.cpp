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

#include <algorithm>
#include <string>
#include <vector>

#include "opennav_docking/dock_database.hpp"
#include "nav2_ros_common/tf2_factories.hpp"

namespace opennav_docking
{

DockDatabase::DockDatabase(std::mutex & mutex)
: mutex_(mutex),
  dock_loader_("opennav_docking_core", "opennav_docking_core::ChargingDock"),
  controller_loader_("opennav_docking", "opennav_docking::ControllerBase")
{}

DockDatabase::~DockDatabase()
{
  dock_instances_.clear();
  dock_plugins_.clear();
  for (auto & controller : controllers_) {
    controller.second->cleanup();
  }
  controllers_.clear();
  type_controllers_.clear();
  reload_db_service_.reset();
}

bool DockDatabase::initialize(
  const nav2::LifecycleNode::WeakPtr & parent,
  nav2::TransformBuffer::SharedPtr tf)
{
  node_ = parent;
  auto node = node_.lock();

  if (!getDockPlugins(node, tf)) {
    RCLCPP_ERROR(
      node->get_logger(),
      "An error occurred while getting the dock plugins!");
    return false;
  }

  if (!getControllerPlugins(node, tf)) {
    RCLCPP_ERROR(
      node->get_logger(),
      "An error occurred while getting the controller plugins!");
    return false;
  }

  if (!getDockInstances(node)) {
    RCLCPP_ERROR(
      node->get_logger(),
      "An error occurred while getting the dock instances!");
    return false;
  }

  assignControllers(dock_instances_);
  if (!validateControllersExist(dock_instances_)) {
    return false;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Docking Server has %u dock types and %u dock instances available.",
    this->plugin_size(), this->instance_size());

  reload_db_service_ = node->create_service<nav2_msgs::srv::ReloadDockDatabase>(
    "~/reload_database",
    std::bind(
      &DockDatabase::reloadDbCb, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  return true;
}

void DockDatabase::activate()
{
  DockPluginMap::iterator it;
  for (it = dock_plugins_.begin(); it != dock_plugins_.end(); ++it) {
    it->second->activate();
  }
  for (auto & controller : controllers_) {
    controller.second->activate();
  }
}

void DockDatabase::deactivate()
{
  DockPluginMap::iterator it;
  for (it = dock_plugins_.begin(); it != dock_plugins_.end(); ++it) {
    it->second->deactivate();
  }
  for (auto & controller : controllers_) {
    controller.second->deactivate();
  }
}

bool DockDatabase::validateControllersExist(const DockMap & docks) const
{
  auto node = node_.lock();
  bool valid = true;

  auto isControllerLoaded = [this](const std::string & name) {
      return controllers_.find(name) != controllers_.end();
    };

  for (const auto & entry : type_controllers_) {
    const std::string & name = entry.second;
    // An empty name selects the server's default graceful controller
    if (!name.empty() && !isControllerLoaded(name)) {
      RCLCPP_ERROR(
        node->get_logger(),
        "Dock plugin '%s' names controller '%s', which is not loaded.",
        entry.first.c_str(), name.c_str());
      valid = false;
    }
  }

  for (const auto & entry : docks) {
    const std::string & name = entry.second.controller_name;
    if (!isControllerLoaded(name)) {
      RCLCPP_ERROR(
        node->get_logger(),
        "Dock '%s' names controller '%s', which is not loaded.",
        entry.first.c_str(), name.c_str());
      valid = false;
    }
  }

  return valid;
}

void DockDatabase::reloadDbCb(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<nav2_msgs::srv::ReloadDockDatabase::Request> request,
  std::shared_ptr<nav2_msgs::srv::ReloadDockDatabase::Response> response)
{
  if (!mutex_.try_lock()) {
    RCLCPP_ERROR(node_.lock()->get_logger(), "Cannot reload database while docking!");
    response->success = false;
    return;
  }

  auto node = node_.lock();
  DockMap dock_instances;
  if (utils::parseDockFile(request->filepath, node, dock_instances)) {
    assignControllers(dock_instances);
    if (validateControllersExist(dock_instances)) {
      dock_instances_ = dock_instances;
      response->success = true;
      RCLCPP_INFO(
        node->get_logger(),
        "Dock database reloaded from file %s.", request->filepath.c_str());
      mutex_.unlock();
      return;
    }
  }
  response->success = false;
  mutex_.unlock();
}

Dock * DockDatabase::findDock(const std::string & dock_id)
{
  Dock * dock_instance = findDockInstance(dock_id);
  ChargingDock::Ptr dock_plugin{nullptr};

  if (dock_instance) {
    dock_plugin = findDockPlugin(dock_instance->type);
    if (dock_plugin) {
      // Populate the plugin shared pointer
      dock_instance->plugin = dock_plugin;
      return dock_instance;
    }
    throw opennav_docking_core::DockNotValid("Dock requested has no valid plugin!");
  }
  throw opennav_docking_core::DockNotInDB("Dock ID requested is not in database!");
}

Dock * DockDatabase::findDockInstance(const std::string & dock_id)
{
  auto it = dock_instances_.find(dock_id);
  if (it != dock_instances_.end()) {
    return &(it->second);
  }
  return nullptr;
}

ChargingDock::Ptr DockDatabase::findDockPlugin(const std::string & type)
{
  // If only one dock plugin and type not set, use the default dock
  if (type.empty() && dock_plugins_.size() == 1) {
    return dock_plugins_.begin()->second;
  }

  auto it = dock_plugins_.find(type);
  if (it != dock_plugins_.end()) {
    return it->second;
  }
  return nullptr;
}

ControllerBase::Ptr DockDatabase::findController(
  const std::string & requested_controller_name, const std::string & type) const
{
  const std::string name = getControllerName(requested_controller_name, type);
  auto it = controllers_.find(name);
  if (it == controllers_.end()) {
    auto node = node_.lock();
    if (node) {
      std::string available;
      for (const auto & controller : controllers_) {
        available += controller.first + " ";
      }
      RCLCPP_ERROR(
        node->get_logger(), "Dock requested controller '%s', which does not exist. "
        "Available controllers are: %s.", name.c_str(), available.c_str());
    }
    throw opennav_docking_core::DockNotValid("Controller '" + name + "' is not loaded");
  }
  return it->second;
}

std::string DockDatabase::getControllerName(
  const std::string & requested_controller_name, const std::string & type) const
{
  if (!requested_controller_name.empty()) {
    return requested_controller_name;
  }

  // If only one dock type and type not set, use that type's controller
  auto it = (type.empty() && type_controllers_.size() == 1) ?
    type_controllers_.begin() : type_controllers_.find(type);
  if (it != type_controllers_.end() && !it->second.empty()) {
    return it->second;
  }

  return kDefaultControllerId;
}

void DockDatabase::assignControllers(DockMap & docks) const
{
  for (auto & entry : docks) {
    Dock & dock = entry.second;
    dock.controller_name = getControllerName(dock.controller_name, dock.type);
  }
}

bool DockDatabase::getDockPlugins(
  const nav2::LifecycleNode::SharedPtr & node,
  nav2::TransformBuffer::SharedPtr tf)
{
  std::vector<std::string> docks_plugins;
  try {
    docks_plugins = node->declare_or_get_parameter<std::vector<std::string>>("dock_plugins");
  } catch (...) {
    RCLCPP_ERROR(node->get_logger(), "Charging dock plugins not given!");
    return false;
  }

  if (docks_plugins.size() < 1u) {
    RCLCPP_ERROR(node->get_logger(), "Charging dock plugins empty! Must provide 1.");
    return false;
  }

  for (size_t i = 0; i != docks_plugins.size(); i++) {
    try {
      std::string plugin_type = nav2::get_plugin_type_param(
        node, docks_plugins[i]);
      opennav_docking_core::ChargingDock::Ptr dock =
        dock_loader_.createUniqueInstance(plugin_type);
      RCLCPP_INFO(
        node->get_logger(), "Created charging dock plugin %s of type %s",
        docks_plugins[i].c_str(), plugin_type.c_str());
      dock->configure(node, docks_plugins[i], tf);
      // Type-level controller selection. An individual dock instance may override this;
      // when both are empty the server's default controller is used.
      type_controllers_[docks_plugins[i]] = node->declare_or_get_parameter(
        docks_plugins[i] + ".controller", std::string(kDefaultControllerId));
      dock_plugins_.insert({docks_plugins[i], dock});
    } catch (const std::exception & ex) {
      RCLCPP_FATAL(
        node->get_logger(), "Failed to create Charging Dock plugin. Exception: %s",
        ex.what());
      return false;
    }
  }

  return true;
}

bool DockDatabase::getControllerPlugins(
  const nav2::LifecycleNode::SharedPtr & node,
  nav2::TransformBuffer::SharedPtr tf)
{
  std::vector<std::string> controller_ids =
    node->declare_or_get_parameter("controllers", std::vector<std::string>{});

  if (std::find(controller_ids.begin(), controller_ids.end(), kDefaultControllerId) !=
    controller_ids.end())
  {
    RCLCPP_ERROR(
      node->get_logger(), "Controller name '%s' is reserved. Please rename it in `controllers`.",
      kDefaultControllerId);
    return false;
  }

  // The default controller is always a graceful controller named `controller`
  const std::string graceful_type = "opennav_docking::GracefulController";
  nav2::declare_parameter_if_not_declared(
    node, std::string(kDefaultControllerId) + ".plugin", rclcpp::ParameterValue(graceful_type));
  if (nav2::get_plugin_type_param(node, kDefaultControllerId) != graceful_type) {
    RCLCPP_ERROR(
      node->get_logger(), "Controller '%s' is reserved for the default %s.",
      kDefaultControllerId, graceful_type.c_str());
    return false;
  }
  controller_ids.push_back(kDefaultControllerId);

  std::string available;
  for (const auto & controller_id : controller_ids) {
    if (controllers_.find(controller_id) != controllers_.end()) {
      RCLCPP_FATAL(
        node->get_logger(), "Controller %s is listed more than once in `controllers`.",
        controller_id.c_str());
      return false;
    }

    try {
      std::string plugin_type = nav2::get_plugin_type_param(node, controller_id);
      ControllerBase::Ptr controller = controller_loader_.createSharedInstance(plugin_type);
      RCLCPP_INFO(
        node->get_logger(), "Created controller plugin %s of type %s",
        controller_id.c_str(), plugin_type.c_str());
      controller->configure(node, controller_id, tf);
      controllers_.insert({controller_id, controller});
    } catch (const std::exception & ex) {
      RCLCPP_FATAL(
        node->get_logger(), "Failed to create controller plugin %s. Exception: %s",
        controller_id.c_str(), ex.what());
      return false;
    }
    available += controller_id + " ";
  }

  RCLCPP_INFO(
    node->get_logger(), "Docking server has %s controllers available.", available.c_str());

  return true;
}

bool DockDatabase::getDockInstances(const nav2::LifecycleNode::SharedPtr & node)
{
  // Attempt to obtain docks from separate file
  std::string dock_filepath;
  try {
    dock_filepath = node->declare_or_get_parameter<std::string>("dock_database");
    RCLCPP_INFO(
      node->get_logger(), "Loading dock from database file  %s.", dock_filepath.c_str());
    try {
      return utils::parseDockFile(dock_filepath, node, dock_instances_);
    } catch (YAML::BadConversion & e) {
      RCLCPP_ERROR(
        node->get_logger(),
        "Dock database (%s) is malformed: %s.", dock_filepath.c_str(), e.what());
      return false;
    }
    return true;
  } catch (...) {
    // pass
  }

  // Attempt to obtain docks from parameter file
  std::vector<std::string> docks_param;
  try {
    docks_param = node->declare_or_get_parameter<std::vector<std::string>>("docks");
    RCLCPP_INFO(node->get_logger(), "Loading docks from parameter file.");
    return utils::parseDockParams(docks_param, node, dock_instances_);
  } catch (...) {
    // pass
  }

  RCLCPP_WARN(
    node->get_logger(),
    "Dock database filepath nor dock parameters set. "
    "Docking actions can only be executed specifying the dock pose via the action request. "
    "Or update the dock database via the reload_database service.");
  return true;
}

unsigned int DockDatabase::plugin_size() const
{
  return dock_plugins_.size();
}

unsigned int DockDatabase::instance_size() const
{
  return dock_instances_.size();
}

unsigned int DockDatabase::controller_size() const
{
  return controllers_.size();
}

}  // namespace opennav_docking
