// Copyright (c) 2020, Samsung Research America
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
// limitations under the License. Reserved.

#ifndef SMAC_PLANNER__POSE_HPP_
#define SMAC_PLANNER__POSE_HPP_

namespace smac_planner
{

struct Pose
{
  Pose()
  : x(0.), y(0.), theta(0.)
  {
  }

  Pose(const double & x_in, const double & y_in, const double & theta_in)
  : x(x_in), y(y_in), theta(theta_in)
  {
  }

  double x, y, theta;
};

}  // namespace nav2_smac_planner

#endif  // SMAC_PLANNER__A_STAR_HPP_
