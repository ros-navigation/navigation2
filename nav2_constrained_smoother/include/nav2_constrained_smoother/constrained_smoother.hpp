// Copyright (c) 2021 RoboTech Vision
// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
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

#ifndef NAV2_CONSTRAINED_SMOOTHER__CONSTRAINED_SMOOTHER_HPP_
#define NAV2_CONSTRAINED_SMOOTHER__CONSTRAINED_SMOOTHER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>

#include "nav2_core/smoother.hpp"
#include "nav2_constrained_smoother/smoother.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

namespace nav2_constrained_smoother
{

/**
 * @class nav2_constrained_smoother::ConstrainedSmoother
 * @brief Regulated pure pursuit controller plugin
 */
class ConstrainedSmoother : public nav2_core::Smoother
{
public:
  /**
   * @brief Constructor for nav2_constrained_smoother::ConstrainedSmoother
   */
  ConstrainedSmoother() = default;

  /**
   * @brief Destrructor for nav2_constrained_smoother::ConstrainedSmoother
   */
  ~ConstrainedSmoother() override = default;

  /**
   * @brief Configure smoother parameters and member variables
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer
   * @param costmap_ros Costmap2DROS object of environment
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub) override;

  /**
   * @brief Cleanup controller state machine
   */
  void cleanup() override;

  /**
   * @brief Activate controller state machine
   */
  void activate() override;

  /**
   * @brief Deactivate controller state machine
   */
  void deactivate() override;

  /**
   * @brief Method to smooth given path
   *
   * @param path In-out path to be optimized
   * @param max_time Maximum duration smoothing should take
   * @return Smoothed path
   */
  bool smooth(
    nav_msgs::msg::Path & path,
    const rclcpp::Duration & max_time) override;

protected:
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  rclcpp::Logger logger_ {rclcpp::get_logger("ConstrainedSmoother")};

  std::unique_ptr<nav2_constrained_smoother::Smoother> smoother_;
  SmootherParams smoother_params_;
  OptimizerParams optimizer_params_;
};

}  // namespace nav2_constrained_smoother

#endif  // NAV2_CONSTRAINED_SMOOTHER__CONSTRAINED_SMOOTHER_HPP_
