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

#ifndef NAV2_COLLISION_MONITOR__CIRCLE_HPP_
#define NAV2_COLLISION_MONITOR__CIRCLE_HPP_

#include <vector>

#include "nav2_collision_monitor/polygon_base.hpp"

namespace nav2_collision_monitor
{

class Circle : public PolygonBase
{
public:
  Circle(
    const nav2_util::LifecycleNode::WeakPtr & node,
    const std::string & polygon_name,
    const std::string & base_frame_id,
    const double simulation_time_step);
  virtual ~Circle();

  virtual void getPolygon(std::vector<Point> & poly);

  virtual int getPointsInside(const std::vector<Point> & points);

protected:
  // @brief Supporting routine obtaining all ROS-parameters.
  // Implementation for Circle class. Calls PolygonBase::getParameters() inside.
  // @param polygon_topic Output name of polygon publishing topic
  // @return True if all parameters were obtained or false in failure case
  virtual bool getParameters(std::string & polygon_topic);

  double radius_;
  double radius_squared_;
};  // class Circle

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__CIRCLE_HPP_
