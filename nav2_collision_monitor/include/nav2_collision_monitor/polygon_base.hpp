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

#ifndef NAV2_COLLISION_MONITOR__POLYGON_BASE_HPP_
#define NAV2_COLLISION_MONITOR__POLYGON_BASE_HPP_

#include "nav2_util/lifecycle_node.hpp"

#include "nav2_collision_monitor/types.hpp"

namespace nav2_collision_monitor
{

enum PolygonType
{
  POLYGON_BASE = 0,
  POLYGON = 1,
  CIRCLE = 2
};

class PolygonBase
{
public:
PolygonBase();
PolygonBase(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string polygon_name,
  const double simulation_time_step);
virtual ~PolygonBase();

bool getParameters();

PolygonType getPolygonType();
void setPolygonType(const PolygonType pt);

ActionType getActionType();

int getStopPoints();
void setStopPoints(const int sp);

double getSlowdown();
void setSlowdown(const double slowdown);

double getTimeBeforeCollision();
void setTimeBeforeCollision(const double tbc);

virtual void getPolygon(std::vector<Point> & poly) = 0;

// Returns estimated (simulated) time before collision and pose where the collision will occur.
// If there is no collision, coll_time will be negative.
void getCollision(
  const Point & point, const Velocity & velocity, double & coll_time);

// Returns safe velocity to keep to a coll_time before collision
Velocity getSafeVelocity(
  const Velocity & velocity, const double coll_time);

virtual bool isPointInside(const Point & point) = 0;

protected:
// Collision Monitor node
nav2_util::LifecycleNode::WeakPtr node_;

PolygonType polygon_type_;
std::string polygon_name_;
ActionType action_type_;
// Minimal number of points to enter inside polygon that causing robot to stop
int stop_points_;
// Robot slowdown (share of its actual speed)
double slowdown_;
// Time before collision in seconds
double time_before_collision_;
// Time step for robot movement simulation
double simulation_time_step_;

};  // class PolygonBase

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__POLYGON_BASE_HPP_
