// Copyright (c) 2022 Dexory
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

#include "nav2_collision_monitor/polygon_velocity.hpp"


namespace nav2_collision_monitor
{

PolygonVelocity::PolygonVelocity(
  const std::vector<Point> & poly,
  const std::string & polygon_name,
  const double & linear_max,
  const double & linear_min,
  const double & direction_max,
  const double & direction_min,
  const double & theta_max,
  const double & theta_min)
: poly_(poly), polygon_name_(polygon_name), linear_max_(linear_max), linear_min_(linear_min),
  direction_max_(direction_max), direction_min_(direction_min),
  theta_max_(theta_max), theta_min_(theta_min)
{
  RCLCPP_INFO(logger_, "[%s]: Creating PolygonVelocity", polygon_name_.c_str());
}


PolygonVelocity::~PolygonVelocity()
{
  RCLCPP_INFO(logger_, "[%s]: Destroying PolygonVelocity", polygon_name_.c_str());
  poly_.clear();
}

bool PolygonVelocity::isInRange(const Velocity & cmd_vel_in)
{
  const double twist_linear = std::hypot(cmd_vel_in.x, cmd_vel_in.y);

  // check if direction in angle range(min -> max)
  double direction = std::atan2(cmd_vel_in.y, cmd_vel_in.x);
  bool direction_in_range;
  if (direction_min_ <= direction_max_) {
    direction_in_range = (direction >= direction_min_ && direction <= direction_max_);
  } else {
    direction_in_range = (direction >= direction_min_ || direction <= direction_max_);
  }

  return twist_linear <= linear_max_ &&
         twist_linear >= linear_min_ &&
         direction_in_range &&
         cmd_vel_in.tw <= theta_max_ &&
         cmd_vel_in.tw >= theta_min_;
}


std::vector<Point> PolygonVelocity::getPolygon()
{
  return poly_;
}


}  // namespace nav2_collision_monitor
