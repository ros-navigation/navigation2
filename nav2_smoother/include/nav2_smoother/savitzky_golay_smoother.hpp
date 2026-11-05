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

#ifndef NAV2_SMOOTHER__SAVITZKY_GOLAY_SMOOTHER_HPP_
#define NAV2_SMOOTHER__SAVITZKY_GOLAY_SMOOTHER_HPP_

#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <string>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <utility>

#include "nav2_core/smoother.hpp"
#include "nav2_util/smoother_utils.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_ros_common/node_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "angles/angles.h"
#include "tf2/utils.hpp"
#include "nav2_ros_common/tf2_factories.hpp"

namespace nav2_smoother
{

/**
 * @class nav2_smoother::SavitzkyGolaySmoother
 * @brief A path smoother implementation using Savitzky Golay filters
 */
class SavitzkyGolaySmoother : public nav2_core::Smoother
{
public:
  /**
   * @brief A constructor for nav2_smoother::SavitzkyGolaySmoother
   */
  SavitzkyGolaySmoother() = default;

  /**
   * @brief A destructor for nav2_smoother::SavitzkyGolaySmoother
   */
  ~SavitzkyGolaySmoother() override = default;

  void configure(
    const nav2::LifecycleNode::WeakPtr &,
    std::string name, nav2::TransformBuffer::SharedPtr,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>) override;

  /**
   * @brief Method to cleanup resources.
   */
  void cleanup() override;

  /**
   * @brief Method to activate smoother and any threads involved in execution.
   */
  void activate() override;

  /**
   * @brief Method to deactivate smoother and any threads involved in execution.
   */
  void deactivate() override;

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

  /**
   * @brief Method to calculate SavitzkyGolay Coefficients
   */
  void calculateCoefficients();

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
    bool & reversing_segment);

  /**
   * @brief Callback executed when a parameter change is detected
   * This callback is triggered when one or more parameters are about to be updated.
   * It checks the validity of parameter values and rejects updates that would lead
   * to invalid or inconsistent configurations, or that arrive while smoothing is
   * in progress
   * @param parameters List of parameters that are being updated.
   * @return rcl_interfaces::msg::SetParametersResult Result indicating whether the update is accepted.
   */
  rcl_interfaces::msg::SetParametersResult validateParameterUpdatesCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  /**
   * @brief Apply parameter updates after validation
   * This callback is executed when parameters have been successfully updated.
   * It updates the internal configuration of the smoother with the new parameter values
   * and recomputes the filter coefficients if the window or polynomial order changed.
   * @param parameters List of parameters that have been updated.
   */
  void updateParametersCallback(const std::vector<rclcpp::Parameter> & parameters);

  bool do_refinement_, enforce_path_inversion_;
  int refinement_num_, window_size_, half_window_size_, poly_order_;
  Eigen::VectorXd sg_coeffs_;
  std::string plugin_name_;
  nav2::LifecycleNode::WeakPtr node_;
  std::mutex mutex_;
  rclcpp::Logger logger_{rclcpp::get_logger("SGSmoother")};

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handler_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_params_handler_;
};

}  // namespace nav2_smoother

#endif  // NAV2_SMOOTHER__SAVITZKY_GOLAY_SMOOTHER_HPP_
