/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, User
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#ifndef NAV2_COSTMAP_2D__PREDICTIVE_INFLATION_LAYER_HPP_
#define NAV2_COSTMAP_2D__PREDICTIVE_INFLATION_LAYER_HPP_

#include <cmath>
#include <mutex>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"

namespace nav2_costmap_2d
{

/**
 * @class PredictiveInflationLayer
 * @brief Inflation layer that modulates costs based on robot heading and nominal speed
 */
class PredictiveInflationLayer : public InflationLayer
{
public:
  /**
   * @brief A constructor
   */
  PredictiveInflationLayer();

  /**
   * @brief A destructor
   */
  virtual ~PredictiveInflationLayer() = default;

  /**
   * @brief Initialization process of layer on startup
   */
  void onInitialize() override;

  /**
   * @brief Update the bounds of the master costmap by this layer's update dimensions
   * @param robot_x X pose of robot
   * @param robot_y Y pose of robot
   * @param robot_yaw Robot orientation
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y, double * max_x, double * max_y) override;

  /**
   * @brief Update the costs in the master costmap in the window
   * @param master_grid The master costmap grid to update
   * @param min_i X min map coord of the window to update
   * @param min_j Y min map coord of the window to update
   * @param max_i X max map coord of the window to update
   * @param max_j Y max map coord of the window to update
   */
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

protected:
  /**
   * @brief Load and validate parameters
   */
  void loadParameters();

  /**
   * @brief Callback executed when a parameter change is detected
   * @param parameters List of changed parameters
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  /**
   * @brief Compute the directional weight based on the angle to the obstacle
   * @param angle Absolute angle between robot heading and obstacle vector
   * @return Weight factor (1.0 = standard, >1.0 = inflated, <1.0 = reduced)
   */
  double computeDirectionalWeight(double angle) const;

  /**
   * @brief Compute the speed-based weight component
   * @return Speed weight factor
   */
  double computeSpeedWeight() const;

  /**
   * @brief Compute the effective distance for cost calculation
   * @param distance Actual Euclidean distance
   * @param weight Combined directional and speed weight
   * @return Effective distance (smaller means closer/higher cost)
   */
  double computeEffectiveDistance(double distance, double weight) const;

  /**
   * @brief Reverse lookup from cost to distance (inverse of computeCost)
   * @param cost Cost value [0-255]
   * @return Distance in cells
   */
  double costToDistance(unsigned char cost) const;

  // Parameters
  bool predictive_mode_;
  double nominal_speed_;
  double speed_scale_;
  double max_inflation_scale_;

  // Angle thresholds (radians)
  double forward_angle_;
  double side_angle_;

  // Directional weights
  double forward_weight_;
  double side_weight_;
  double rear_weight_;

  // Cached state
  double base_inflation_radius_;  // The "true" radius configured by user
  double last_robot_x_;
  double last_robot_y_;
  double last_robot_yaw_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__PREDICTIVE_INFLATION_LAYER_HPP_
