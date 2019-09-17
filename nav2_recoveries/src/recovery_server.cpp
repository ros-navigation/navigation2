// Copyright (c) 2018 Samsung Research America
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
// limitations under the License. Reserved.

#include "nav2_recoveries/recovery_server.hpp"

namespace recovery_server
{

RecoveryServer::RecoveryServer()
: nav2_util::LifecycleNode("nav2_recoveries", "", true),
  plugin_loader_("nav2_core", "nav2_core::Recovery")
{

}


RecoveryServer::~RecoveryServer()
{
  return;
}

nav2_util::CallbackReturn
RecoveryServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");


  auto tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(),
    get_node_timers_interface());
  tf_->setCreateTimerInterface(timer_interface);
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_);

  declare_parameter("costmap_topic",
    rclcpp::ParameterValue(std::string("local_costmap/costmap_raw")));
  declare_parameter("footprint_topic",
    rclcpp::ParameterValue(std::string("local_costmap/published_footprint")));

  std::vector<std::string> plugin_names{std::string("default_spin"),
    std::string("default_backup")};
  std::vector<std::string> plugin_types{std::string("nav2_recoveries/Spin"),
    std::string("nav2_recoveries/Backup")};

  declare_parameter("plugin_names",
    rclcpp::ParameterValue(plugin_names));
  declare_parameter("plugin_types",
    rclcpp::ParameterValue(plugin_types));

  plugin_names_ = get_parameter("plugin_names").as_string_array();
  plugin_types_ = get_parameter("plugin_types").as_string_array();

  loadRecoveryPlugins();

  return nav2_util::CallbackReturn::SUCCESS;
}


void
RecoveryServer::loadRecoveryPlugins()
{
  auto node = shared_from_this();
  for (uint i = 0; i != plugin_names_.size(); i++) {
    try {
      nav2_core::Recovery::Ptr recovery =
        plugin_loader_.createUniqueInstance(plugin_types_[i]);
      RCLCPP_INFO(get_logger(), "Created recovery plugin %s of type %s",
        plugin_names_[i].c_str(), plugin_types_[i].c_str());
      recovery->configure(node,
        plugin_loader_.getName(plugin_names_[i]), tf_);
      recoveries_.push_back(recovery);
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(get_logger(), "Failed to create recovery %s of type %s."
        " Exception: %s", plugin_names_[i].c_str(), plugin_types_[i].c_str(),
        ex.what());
      exit(-1);
    }
  }
}

nav2_util::CallbackReturn
RecoveryServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  std::vector<nav2_core::Recovery::Ptr>::iterator iter;
  for (iter = recoveries_.begin(); iter != recoveries_.end(); ++ iter) {
    (*iter)->activate();
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
RecoveryServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  
  std::vector<nav2_core::Recovery::Ptr>::iterator iter;
  for (iter = recoveries_.begin(); iter != recoveries_.end(); ++ iter) {
    (*iter)->deactivate();
  }
  
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
RecoveryServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  std::vector<nav2_core::Recovery::Ptr>::iterator iter;
  for (iter = recoveries_.begin(); iter != recoveries_.end(); ++ iter) {
    (*iter)->cleanup();
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
RecoveryServer::on_error(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_FATAL(get_logger(), "Lifecycle node entered error state");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
RecoveryServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

}  // end namespace recovery_server
