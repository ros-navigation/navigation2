// Copyright (c) 2026
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

#ifndef NAV2_MPPI_CONTROLLER__POLYGON_UTILS__VELOCITY_POLYGON_HPP_
#define NAV2_MPPI_CONTROLLER__POLYGON_UTILS__VELOCITY_POLYGON_HPP_

#include <string>
#include <vector>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_costmap_2d/inflation_layer_interface.hpp"
#include "nav2_mppi_controller/tools/parameters_handler.hpp"

namespace mppi::polygon_utils
{
struct SubPolygon
{
  std::string name;
  nav2_costmap_2d::Footprint poly;
  double circumscribed_radius{0.0};
  double inscribed_radius{0.0};
  double linear_min{0.0};
  double linear_max{0.0};
  double theta_min{0.0};
  double theta_max{0.0};
  double direction_start_angle{-M_PI};    // holonomic only
  double direction_end_angle{M_PI};       // holonomic only

  // -1.0 means inflation doesn't reach this footprint's edge
  // this shortcut cannot be trusted for this bucket at all

  double tau_circumscribed{-1.0};
  double tau_inscribed{-1.0};
};
/**
 * @class mppi::polygon_utils::VelocityPolygon
 * @brief Selects one of several statically-configured polygons based on the
 * robot's current linear/angular speed. Stateless: update() writes its
 * result to the caller's own storage rather than mutating shared members,
 * so one instance can be queried safely from multiple threads (e.g. an
 * OpenMP-parallel per-pose loop) at once.
 */
class VelocityPolygon
{
public:
  VelocityPolygon() = default;

  bool onConfigure(const std::string & name, ParametersHandler * param_handler);

  /**
   * @brief Precomputes tau_circumscribed/tau_inscribed for every bucket from
   * the current inflation layer. Called once per score(), not per pose.
   */
  void computeBucketThresholds(
    const std::shared_ptr<nav2_costmap_2d::InflationLayerInterface> & inflation_layer,
    double resolution);

  /**
   * @brief Looks up the sub-polygon whose velocity window contains (vx, vy, wz).
   * @return a non-owning pointer to the matching bucket (valid for this
   * object's lifetime, never null-checked-then-stored past the call), or
   * nullptr if no bucket's window contains this velocity.
   */
  const SubPolygon * findPolygon(double vx, double vy, double wz) const;

protected:
  /**
   * @brief Whether (vx, vy, wz) falls within a sub-polygon's configured window
   */
  bool isInRange(double vx, double vy, double wz, const SubPolygon & sub) const;

  std::string name_;
  ParametersHandler * parameters_handler_{nullptr};
  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};

  bool holonomic_{false};
  std::vector<SubPolygon> sub_polygons_;
};

}  // namespace mppi::polygon_utils

#endif  // NAV2_MPPI_CONTROLLER__POLYGON_UTILS__VELOCITY_POLYGON_HPP_
