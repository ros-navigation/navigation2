// Copyright (c) 2022, Samsung Research America
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

#ifndef NAV2_SMOOTHER__SIMPLE_SMOOTHER_HPP_
#define NAV2_SMOOTHER__SIMPLE_SMOOTHER_HPP_

#include <cmath>
#include <vector>
#include <string>
#include <iostream>
#include <memory>
#include <queue>
#include <utility>

#include "nav2_core/smoother.hpp"
#include "nav2_smoother/smoother_utils.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "angles/angles.h"
#include "tf2/utils.h"

namespace nav2_smoother
{

/**
 * @class nav2_smoother::SimpleSmoother
 * @brief A path smoother implementation
 */
class SimpleSmoother : public nav2_core::Smoother
{
public:
  /**
   * @brief A constructor for nav2_smoother::SimpleSmoother
   */
  SimpleSmoother() = default;

  /**
   * @brief A destructor for nav2_smoother::SimpleSmoother
   */
  ~SimpleSmoother() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    std::string name, std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>) override;

  /**
   * @brief Method to cleanup resources.
   */
  void cleanup() override {costmap_sub_.reset();}

  /**
   * @brief Method to activate smoother and any threads involved in execution.
   */
  void activate() override {}

  /**
   * @brief Method to deactivate smoother and any threads involved in execution.
   */
  void deactivate() override {}

  /**
   * @brief Method to smooth given path
   *
   * @param path In-out path to be smoothed
   * @param max_time Maximum duration smoothing should take
   * @return If smoothing was completed (true) or interrupted by time limit (false)
   */
  bool smooth(
    nav_msgs::msg::Path & path,
    const rclcpp::Duration & max_time) override;

protected:
  /**
   * @brief Smoother method - does the smoothing on a segment
   * @param path Reference to path
   * @param reversing_segment Return if this is a reversing segment
   * @param costmap Pointer to minimal costmap
   * @param max_time Maximum time to compute, stop early if over limit
   * @return If smoothing was successful
   */
  bool smoothImpl(
    nav_msgs::msg::Path & path,
    bool & reversing_segment,
    const nav2_costmap_2d::Costmap2D * costmap,
    const double & max_time);

  /**
   * @brief Get the field value for a given dimension
   * @param msg Current pose to sample
   * @param dim Dimension ID of interest
   * @return dim value
   */
  inline double getFieldByDim(
    const geometry_msgs::msg::PoseStamped & msg,
    const unsigned int & dim);

  /**
   * @brief Set the field value for a given dimension
   * @param msg Current pose to sample
   * @param dim Dimension ID of interest
   * @param value to set the dimention to for the pose
   */
  inline void setFieldByDim(
    geometry_msgs::msg::PoseStamped & msg, const unsigned int dim,
    const double & value);

  double tolerance_, data_w_, smooth_w_;
  int max_its_, refinement_ctr_;
  bool do_refinement_;
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  rclcpp::Logger logger_{rclcpp::get_logger("SimpleSmoother")};
};

}  // namespace nav2_smoother

#endif  // NAV2_SMOOTHER__SIMPLE_SMOOTHER_HPP_
