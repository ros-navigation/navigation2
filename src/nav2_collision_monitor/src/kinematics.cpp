// Copyright (c) 2022 Samsung R&D Institute Russia
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

#include "nav2_collision_monitor/kinematics.hpp"

#include <cmath>

namespace nav2_collision_monitor
{

void transformPoints(const Pose & pose, std::vector<Point> & points)
{
  const double cos_theta = std::cos(pose.theta);
  const double sin_theta = std::sin(pose.theta);

  for (Point & point : points) {
    // p = R*p' + pose
    // p' = Rt * (p - pose)
    // where:
    //   p - point coordinates in initial frame
    //   p' - point coordinates in a new frame
    //   R - rotation matrix =
    //     [cos_theta -sin_theta]
    //     [sin_theta  cos_theta]
    //   Rt - transposed (inverted) rotation matrix
    const double mul_x = point.x - pose.x;
    const double mul_y = point.y - pose.y;
    point.x = mul_x * cos_theta + mul_y * sin_theta;
    point.y = -mul_x * sin_theta + mul_y * cos_theta;
  }
}

void projectState(const double & dt, Pose & pose, Velocity & velocity)
{
  const double theta = velocity.tw * dt;
  const double cos_theta = std::cos(theta);
  const double sin_theta = std::sin(theta);

  // p' = p + vel*dt
  // where:
  //   p - initial pose
  //   p' - projected pose
  pose.x = pose.x + velocity.x * dt;
  pose.y = pose.y + velocity.y * dt;
  // Rotate the pose on theta
  pose.theta = pose.theta + theta;

  // vel' = R*vel
  // where:
  //   vel - initial velocity
  //   R - rotation matrix
  //   vel' - rotated velocity
  const double velocity_upd_x = velocity.x * cos_theta - velocity.y * sin_theta;
  const double velocity_upd_y = velocity.x * sin_theta + velocity.y * cos_theta;
  velocity.x = velocity_upd_x;
  velocity.y = velocity_upd_y;
}

}  // namespace nav2_collision_monitor
