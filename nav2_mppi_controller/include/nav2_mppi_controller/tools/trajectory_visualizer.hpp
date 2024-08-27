// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#ifndef NAV2_MPPI_CONTROLLER__TOOLS__TRAJECTORY_VISUALIZER_HPP_
#define NAV2_MPPI_CONTROLLER__TOOLS__TRAJECTORY_VISUALIZER_HPP_

#include <memory>
#include <string>

// xtensor creates warnings that needs to be ignored as we are building with -Werror
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
#pragma GCC diagnostic ignored "-Wstringop-overflow"
#include <xtensor/xtensor.hpp>
#pragma GCC diagnostic pop

#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav2_mppi_controller/tools/parameters_handler.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"
#include "nav2_mppi_controller/models/trajectories.hpp"

namespace mppi
{

/**
 * @class mppi::TrajectoryVisualizer
 * @brief Visualizes trajectories for debugging
 */
class TrajectoryVisualizer
{
public:
  /**
    * @brief Constructor for mppi::TrajectoryVisualizer
    */
  TrajectoryVisualizer() = default;

  /**
    * @brief Configure trajectory visualizer
    * @param parent WeakPtr to node
    * @param name Name of plugin
    * @param frame_id Frame to publish trajectories in
    * @param dynamic_parameter_handler Parameter handler object
    */
  void on_configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
    const std::string & frame_id, ParametersHandler * parameters_handler);

  /**
    * @brief Cleanup object on shutdown
    */
  void on_cleanup();

  /**
    * @brief Activate object
    */
  void on_activate();

  /**
    * @brief Deactivate object
    */
  void on_deactivate();

  /**
    * @brief Add an optimal trajectory to visualize
    * @param trajectory Optimal trajectory
    */
  void add(
    const xt::xtensor<float, 2> & trajectory, const std::string & marker_namespace,
    const builtin_interfaces::msg::Time & cmd_stamp);

  /**
    * @brief Add candidate trajectories to visualize
    * @param trajectories Candidate trajectories
    */
  void add(const models::Trajectories & trajectories, const std::string & marker_namespace);

  /**
    * @brief Visualize the plan
    * @param plan Plan to visualize
    */
  void visualize(const nav_msgs::msg::Path & plan);

  /**
    * @brief Reset object
    */
  void reset();

protected:
  std::string frame_id_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>>
  trajectories_publisher_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> transformed_path_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> optimal_path_pub_;

  std::unique_ptr<nav_msgs::msg::Path> optimal_path_;
  std::unique_ptr<visualization_msgs::msg::MarkerArray> points_;
  int marker_id_ = 0;

  ParametersHandler * parameters_handler_;

  size_t trajectory_step_{0};
  size_t time_step_{0};

  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__TOOLS__TRAJECTORY_VISUALIZER_HPP_
