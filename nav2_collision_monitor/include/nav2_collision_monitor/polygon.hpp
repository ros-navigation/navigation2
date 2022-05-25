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

#include "nav2_collision_monitor/polygon_base.hpp"

namespace nav2_collision_monitor
{

class Polygon : public PolygonBase
{

public:
Polygon(nav2_util::LifecycleNode * node, const EmergencyModel em, const std::vector<Point> poly);
virtual ~Polygon();

virtual void getPoly(std::vector<Point> & poly);

virtual bool isPointInside(const Point & point);

private:
std::vector<Point> poly_;
};  // class Polygon

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__POLYGON_HPP_
