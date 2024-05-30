// Copyright (c) 2021 RoboTech Vision
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

#ifndef NAV2_SMOOTHER__NAV2_SMOOTHER_HPP_
#define NAV2_SMOOTHER__NAV2_SMOOTHER_HPP_

#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "nav2_core/smoother.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/costmap_topic_collision_checker.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "nav2_msgs/action/smooth_path.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"

namespace nav2_smoother
{

/**
 * @class nav2_smoother::SmootherServer
 * @brief This class hosts variety of plugins of different algorithms to
 * smooth or refine a path from the exposed SmoothPath action server.
 */
class SmootherServer : public nav2_util::LifecycleNode
{
public:
  using SmootherMap = std::unordered_map<std::string, nav2_core::Smoother::Ptr>;

  /**
   * @brief A constructor for nav2_smoother::SmootherServer
   * @param options Additional options to control creation of the node.
   */
  explicit SmootherServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief Destructor for nav2_smoother::SmootherServer
   */
  ~SmootherServer();

protected:
  /**
   * @brief Configures smoother parameters and member variables
   *
   * Configures smoother plugin and costmap; Initialize odom subscriber,
   * velocity publisher and smooth path action server.
   * @param state LifeCycle Node's state
   * @return Success or Failure
   * @throw pluginlib::PluginlibException When failed to initialize smoother
   * plugin
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Loads smoother plugins from parameter file
   * @return bool if successfully loaded the plugins
   */
  bool loadSmootherPlugins();

  /**
   * @brief Activates member variables
   *
   * Activates smoother, costmap, velocity publisher and smooth path action
   * server
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Deactivates member variables
   *
   * Deactivates smooth path action server, smoother, costmap and velocity
   * publisher. Before calling deactivate state, velocity is being set to zero.
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Calls clean up states and resets member variables.
   *
   * Smoother and costmap clean up state is called, and resets rest of the
   * variables
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Called when in Shutdown state
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  using Action = nav2_msgs::action::SmoothPath;
  using ActionResult = Action::Result;
  using ActionServer = nav2_util::SimpleActionServer<Action>;

  /**
   * @brief SmoothPath action server callback. Handles action server updates and
   * spins server until goal is reached
   *
   * Provides global path to smoother received from action client. Local
   * section of the path is optimized using smoother.
   * @throw nav2_core::PlannerException
   */
  void smoothPlan();

  /**
   * @brief Find the valid smoother ID name for the given request
   *
   * @param c_name The requested smoother name
   * @param name Reference to the name to use for control if any valid available
   * @return bool Whether it found a valid smoother to use
   */
  bool findSmootherId(const std::string & c_name, std::string & name);

  /**
   * @brief Validate that the path contains a meaningful path for smoothing
   * @param path current path
   * return bool if the path is valid
   */
  bool validate(const nav_msgs::msg::Path & path);

  // Our action server implements the SmoothPath action
  std::unique_ptr<ActionServer> action_server_;

  // Transforms
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

  // Publishers and subscribers
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;

  // Smoother Plugins
  pluginlib::ClassLoader<nav2_core::Smoother> lp_loader_;
  SmootherMap smoothers_;
  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> smoother_ids_;
  std::vector<std::string> smoother_types_;
  std::string smoother_ids_concat_, current_smoother_;

  // Utilities
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;
  std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker_;
};

}  // namespace nav2_smoother

#endif  // NAV2_SMOOTHER__NAV2_SMOOTHER_HPP_
