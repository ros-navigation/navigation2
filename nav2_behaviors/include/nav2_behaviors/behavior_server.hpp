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


#include <chrono>
#include <string>
#include <memory>
#include <vector>

#include "nav2_util/lifecycle_node.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_core/behavior.hpp"

#ifndef NAV2_BEHAVIORS__BEHAVIOR_SERVER_HPP_
#define NAV2_BEHAVIORS__BEHAVIOR_SERVER_HPP_

namespace behavior_server
{

/**
 * @class behavior_server::BehaviorServer
 * @brief An server hosting a map of behavior plugins
 */
class BehaviorServer : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief A constructor for behavior_server::BehaviorServer
   * @param options Additional options to control creation of the node.
   */
  explicit BehaviorServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~BehaviorServer();

  /**
   * @brief Loads behavior plugins from parameter file
   * @return bool if successfully loaded the plugins
   */
  bool loadBehaviorPlugins();

protected:
  /**
   * @brief Configure lifecycle server
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Activate lifecycle server
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Deactivate lifecycle server
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Cleanup lifecycle server
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Shutdown lifecycle server
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

  // Plugins
  pluginlib::ClassLoader<nav2_core::Behavior> plugin_loader_;
  std::vector<pluginlib::UniquePtr<nav2_core::Behavior>> behaviors_;
  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> behavior_ids_;
  std::vector<std::string> behavior_types_;

  // Utilities
  std::unique_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  std::unique_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;
  std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker_;
};

}  // namespace behavior_server

#endif  // NAV2_BEHAVIORS__BEHAVIOR_SERVER_HPP_
