// Copyright (c) 2018 Intel Corporation
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

#ifndef NAV2_MOTION_PRIMITIVES__SPIN_HPP_
#define NAV2_MOTION_PRIMITIVES__SPIN_HPP_

#include <chrono>
#include <string>
#include <memory>

#include "nav2_motion_primitives/motion_primitive.hpp"
#include "nav2_tasks/spin_task.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

namespace nav2_motion_primitives
{

class Spin : public MotionPrimitive<nav2_tasks::SpinCommand, nav2_tasks::SpinResult>
{
public:
  explicit Spin(rclcpp::Node::SharedPtr & node);
  ~Spin();

  nav2_tasks::TaskStatus onRun(const nav2_tasks::SpinCommand::SharedPtr command) override;

  nav2_tasks::TaskStatus onCycleUpdate(nav2_tasks::SpinResult & result) override;

protected:
  double min_rotational_vel_;
  double max_rotational_vel_;
  double rotational_acc_lim_;
  double goal_tolerance_angle_;

  double start_yaw_;

  std::chrono::system_clock::time_point start_time_;

  nav2_tasks::TaskStatus timedSpin();

  nav2_tasks::TaskStatus controlledSpin();
};

}  // namespace nav2_motion_primitives

#endif  // NAV2_MOTION_PRIMITIVES__SPIN_HPP_
