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

#include "nav2_collision_monitor/polygon_base.hpp"
#include "nav2_collision_monitor/dynamics.hpp"

namespace nav2_collision_monitor
{

PolygonBase::PolygonBase()
: node_(nullptr), polygon_type_(POLYGON_BASE), emergency_model_(DO_NOTHING),
  emergency_thresh_(-1), time_before_crash_(-1.0)
{
}

PolygonBase::PolygonBase(nav2_util::LifecycleNode * node, const EmergencyModel em)
: node_(node), polygon_type_(POLYGON_BASE), emergency_model_(em)
{
  RCLCPP_INFO(node_->get_logger(), "Creating PolygonBase");
}

PolygonBase::~PolygonBase()
{
  if (node_)
  {
    RCLCPP_INFO(node_->get_logger(), "Destroying PolygonBase");
  }
}

PolygonType PolygonBase::getPolygonType()
{
  return polygon_type_;
}

void PolygonBase::setPolygonType(const PolygonType pt)
{
  polygon_type_ = pt;
}

EmergencyModel PolygonBase::getEmergencyModel()
{
  return emergency_model_;
}

int PolygonBase::getEmergencyThresh()
{
  return emergency_thresh_;
}

void PolygonBase::setEmergencyThresh(const int et)
{
  emergency_thresh_ = et;
}

double PolygonBase::getSlowdown()
{
  return slowdown_;
}

void PolygonBase::setSlowdown(const double slowdown)
{
  slowdown_ = slowdown;
}

double PolygonBase::getTimeBeforeCrash()
{
  return time_before_crash_;
}

void PolygonBase::setTimeBeforeCrash(const double tbc)
{
  time_before_crash_ = tbc;
}

void PolygonBase::getCollision(
  const Point & point, const Velocity & velocity, double & coll_time)
{
  // Initial robot pose is {0,0} in base_footprint coordinates
  Pose pose = {0, 0, 0};
  Velocity vel = velocity;
  double d_time = 0.02;

  // Robot movement simulation
  for (double time = 0.0; time <= time_before_crash_; time += d_time) {
    // NOTE: vel is changing during the simulation
    movePose(vel, d_time, pose);
    Point point_fixed = point;
    fixPoint(pose, point_fixed);
    if (isPointInside(point_fixed)) {
      coll_time = time;
      return;
    }
  }

  // There is no collision
  coll_time = -1.0;
}

Velocity PolygonBase::getSafeVelocity(
  const Velocity & velocity, const double coll_time)
{
  const double change_ratio = coll_time / time_before_crash_;
  return velocity * change_ratio;
}

}  // namespace nav2_collision_monitor
