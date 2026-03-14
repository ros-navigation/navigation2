// Copyright (c) 2025 Open Navigation LLC
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

#ifndef NAV2_MPPI_CONTROLLER__TOOLS__CRITICS_VISUALIZER_HPP_
#define NAV2_MPPI_CONTROLLER__TOOLS__CRITICS_VISUALIZER_HPP_

#include <Eigen/Dense>

#include <memory>
#include <string>
#include <vector>

#include "nav2_msgs/msg/critics_stats.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"

#include "nav2_mppi_controller/tools/parameters_handler.hpp"
#include "nav2_mppi_controller/critic_data.hpp"

namespace mppi
{

/**
 * @class mppi::CriticsVisualizer
 * @brief Handles visualization of critic evaluation statistics
 */
class CriticsVisualizer
{
public:
  CriticsVisualizer() = default;

  /**
    * @brief Configure critics visualizer
    * @param parent WeakPtr to node
    * @param name Name of plugin
    * @param parameters_handler Parameter handler object
    */
  void on_configure(
    nav2::LifecycleNode::WeakPtr parent, const std::string & name,
    ParametersHandler * parameters_handler);

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
    * @brief Whether cost tracking is needed for any visualization feature
    * @return True if cost diffs should be captured during evaluation
    */
  bool shouldTrackCosts() const;

  /**
    * @brief Prepare CriticData for evaluation (initialize per-critic cost tracking)
    * @param data CriticData to prepare
    * @param num_critics Number of critics that will be evaluated
    */
  void prepareEvaluation(CriticData & data, size_t num_critics) const;

  /**
    * @brief Process collected cost diffs: populate per-critic costs, compute stats, publish
    * @param critic_names Ordered names of critics that were evaluated
    * @param cost_diffs Per-critic cost diff arrays (same order as critic_names)
    * @param data CriticData (for trajectory_collisions mask and individual_critics_cost)
    */
  void processEvaluation(
    const std::vector<std::string> & critic_names,
    const std::vector<Eigen::ArrayXf> & cost_diffs,
    CriticData & data) const;

protected:
  nav2::LifecycleNode::WeakPtr parent_;
  ParametersHandler * parameters_handler_;

  nav2::Publisher<nav2_msgs::msg::CriticsStats>::SharedPtr critics_effect_pub_;
  bool publish_critics_stats_{false};
  bool visualize_per_critic_costs_{false};

  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__TOOLS__CRITICS_VISUALIZER_HPP_
