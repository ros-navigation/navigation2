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

#include "nav2_collision_monitor/dynamics.hpp"

#include <cmath>

namespace nav2_collision_monitor
{

void fixPoint(const Velocity & velocity, const double dt, Point & point)
{
  // p = R*p' + vel*dt
  // p' = Rt * (p - vel*dt)
  // R = [ cos_theta sin_theta]
  //     [-sin_theta cos_theta]

  const double mul_x = point.x - velocity.x * dt;
  const double mul_y = point.y - velocity.y * dt;
  const double theta = velocity.tw * dt;
  const double cos_theta = std::cos(theta);
  const double sin_theta = std::sin(theta);
  point.x = mul_x * cos_theta + mul_y * sin_theta;
  point.y = -mul_x * sin_theta + mul_y * cos_theta;
}

void fixPoint(const Pose & curr_pose, Point & point)
{
  // p = R*p' + curr_pose
  // p' = Rt * (p - curr_pose)
  const double mul_x = point.x - curr_pose.x;
  const double mul_y = point.y - curr_pose.y;
  const double cos_theta = std::cos(curr_pose.theta);
  const double sin_theta = std::sin(curr_pose.theta);
  point.x = mul_x * cos_theta + mul_y * sin_theta;
  point.y = -mul_x * sin_theta + mul_y * cos_theta;
}

void movePose(Velocity & velocity, const double dt, Pose & pose)
{
  const double theta = velocity.tw * dt;
  const double cos_theta = std::cos(theta);
  const double sin_theta = std::sin(theta);

  // p' = p + vel*dt
  pose.x = pose.x + velocity.x * dt;
  pose.y = pose.y + velocity.y * dt;
  // Rotate the pose on theta
  pose.theta = pose.theta + theta;

  // vel' = R*vel
  const double velocity_upd_x = velocity.x * cos_theta - velocity.y * sin_theta;
  const double velocity_upd_y = velocity.x * sin_theta + velocity.y * cos_theta;
  velocity.x = velocity_upd_x;
  velocity.y = velocity_upd_y;
}

}  // namespace nav2_collision_monitor
