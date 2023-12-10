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

#ifndef NAV2_COLLISION_MONITOR__KINEMATICS_HPP_
#define NAV2_COLLISION_MONITOR__KINEMATICS_HPP_

#include <vector>

#include "nav2_collision_monitor/types.hpp"

namespace nav2_collision_monitor
{

/**
 * @brief Do a transformation of points' coordinates from the frame coinciding with the (0,0)
 * origin to the non-existing in ROS frame, which origin is equal to pose
 * @param pose Origin of the new frame
 * @param points Array of points whose coordinates will be transformed
 */
void transformPoints(const Pose & pose, std::vector<Point> & points);

/**
 * @brief Linearly projects pose towards to velocity direction on dt time interval.
 * Turns the velocity on twist angle for dt time interval.
 * @param dt Time step (in seconds). Should be relatively small
 * to consider all movements to be linear.
 * @param pose Pose to be projected
 * @param velocity Velocity at which the pose to be moved. It is also being rotated
 * on according twist angle.
 */
void projectState(const double & dt, Pose & pose, Velocity & velocity);

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__KINEMATICS_HPP_
