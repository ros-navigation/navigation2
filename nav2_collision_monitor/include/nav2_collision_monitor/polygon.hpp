// Copyright (c) 2022 Samsung Research Russia
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

#ifndef NAV2_COLLISION_MONITOR__POLYGON_HPP_
#define NAV2_COLLISION_MONITOR__POLYGON_HPP_

#include <vector>
#include <string>

#include "nav2_collision_monitor/polygon_base.hpp"

namespace nav2_collision_monitor
{

/**
 * @brief Polygon shape implementaiton
 */
class Polygon : public PolygonBase
{

public:
  /**
   * @brief Polygon class constructor
   */
  Polygon(
    const nav2_util::LifecycleNode::WeakPtr & node,
    const std::string & polygon_name,
    const std::string & base_frame_id,
    const double simulation_time_step);
  /**
   * @brief Polygon class destructor
   */
  virtual ~Polygon();

  /**
   * @brief Gets polygon points
   * @param poly Output polygon points (vertices)
   */
  virtual void getPolygon(std::vector<Point> & poly);

  /**
   * @brief Gets number of points inside polygon
   * @param points Input array of points to be checked
   * @return Number of points inside polygon. If there are no points,
   * returns zero-value
   */
  virtual int getPointsInside(const std::vector<Point> & points);

protected:
  /**
   * @brief Supporting routine obtaining all ROS-parameters.
   * Implementation for Polygon class. Calls PolygonBase::getParameters() inside.
   * @param polygon_topic Output name of polygon publishing topic
   * @return True if all parameters were obtained or false in failure case
   */
  virtual bool getParameters(std::string & polygon_topic);

  /**
   * @brief Checks if point is inside polygon
   * @param point Given point to check
   * @return True if given point is inside polygon, otherwise false
   */
  bool isPointInside(const Point & point);

  /// @brief Polygon points (vertices)
  std::vector<Point> poly_;
};  // class Polygon

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__POLYGON_HPP_
