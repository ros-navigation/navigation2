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

#include <memory>
#include <string>
#include <vector>
#include <utility>
#include "nav2_recoveries/recovery_server.hpp"

namespace recovery_server
{

RecoveryServer::RecoveryServer()
: nav2_util::LifecycleNode("nav2_recoveries", "", true),
  plugin_loader_("nav2_core", "nav2_core::Recovery")
{
  declare_parameter(
    "costmap_topic",
    rclcpp::ParameterValue(std::string("local_costmap/costmap_raw")));
  declare_parameter(
    "footprint_topic",
    rclcpp::ParameterValue(std::string("local_costmap/published_footprint")));
  declare_parameter("cycle_frequency", rclcpp::ParameterValue(10.0));

  std::vector<std::string> plugin_names{std::string("spin"),
    std::string("back_up"), std::string("wait")};
  std::vector<std::string> plugin_types{std::string("nav2_recoveries/Spin"),
    std::string("nav2_recoveries/BackUp"),
    std::string("nav2_recoveries/Wait")};

  declare_parameter(
    "plugin_names",
    rclcpp::ParameterValue(plugin_names));
  declare_parameter(
    "plugin_types",
    rclcpp::ParameterValue(plugin_types));
  declare_parameter(
    "odom_frame",
    rclcpp::ParameterValue("odom"));
  declare_parameter(
    "robot_base_frame",
    rclcpp::ParameterValue("base_link"));
}


RecoveryServer::~RecoveryServer()
{
}

nav2_util::CallbackReturn
RecoveryServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface());
  tf_->setCreateTimerInterface(timer_interface);
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

  std::string costmap_topic, footprint_topic;
  this->get_parameter("costmap_topic", costmap_topic);
  this->get_parameter("footprint_topic", footprint_topic);
  costmap_sub_ = std::make_unique<nav2_costmap_2d::CostmapSubscriber>(
    shared_from_this(), costmap_topic);
  footprint_sub_ = std::make_unique<nav2_costmap_2d::FootprintSubscriber>(
    shared_from_this(), footprint_topic, 1.0);

  std::string odom_frame, robot_base_frame;
  this->get_parameter("odom_frame", odom_frame);
  this->get_parameter("robot_base_frame", robot_base_frame);
  collision_checker_ = std::make_shared<nav2_costmap_2d::CollisionChecker>(
    *costmap_sub_, *footprint_sub_, *tf_, this->get_name(), odom_frame, robot_base_frame);

  this->get_parameter("plugin_names", plugin_names_);
  this->get_parameter("plugin_types", plugin_types_);

  loadRecoveryPlugins();

  return nav2_util::CallbackReturn::SUCCESS;
}


void
RecoveryServer::loadRecoveryPlugins()
{
  auto node = shared_from_this();

  for (size_t i = 0; i != plugin_names_.size(); i++) {
    try {
      RCLCPP_INFO(
        get_logger(), "Creating recovery plugin %s of type %s",
        plugin_names_[i].c_str(), plugin_types_[i].c_str());
      recoveries_.push_back(plugin_loader_.createUniqueInstance(plugin_types_[i]));
      recoveries_.back()->configure(node, plugin_names_[i], tf_, collision_checker_);
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(), "Failed to create recovery %s of type %s."
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
  std::vector<pluginlib::UniquePtr<nav2_core::Recovery>>::iterator iter;
  for (iter = recoveries_.begin(); iter != recoveries_.end(); ++iter) {
    (*iter)->activate();
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
RecoveryServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  std::vector<pluginlib::UniquePtr<nav2_core::Recovery>>::iterator iter;
  for (iter = recoveries_.begin(); iter != recoveries_.end(); ++iter) {
    (*iter)->deactivate();
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
RecoveryServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  std::vector<pluginlib::UniquePtr<nav2_core::Recovery>>::iterator iter;
  for (iter = recoveries_.begin(); iter != recoveries_.end(); ++iter) {
    (*iter)->cleanup();
  }

  recoveries_.clear();
  transform_listener_.reset();
  tf_.reset();
  footprint_sub_.reset();
  costmap_sub_.reset();
  collision_checker_.reset();

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
