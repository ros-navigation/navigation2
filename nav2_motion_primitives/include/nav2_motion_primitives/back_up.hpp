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

#ifndef NAV2_MOTION_PRIMITIVES__BACK_UP_HPP_
#define NAV2_MOTION_PRIMITIVES__BACK_UP_HPP_

#include <chrono>
#include <memory>

#include "nav2_motion_primitives/motion_primitive.hpp"
#include "nav2_msgs/action/back_up.hpp"

namespace nav2_motion_primitives
{
using BackUpAction = nav2_msgs::action::BackUp;

class BackUp : public MotionPrimitive<BackUpAction>
{
public:
  explicit BackUp(rclcpp::Node::SharedPtr & node);
  ~BackUp();

  Status onRun(const std::shared_ptr<const BackUpAction::Goal> command) override;

  Status onCycleUpdate() override;

protected:
  double min_linear_vel_;
  double max_linear_vel_;
  double linear_acc_lim_;

  nav_msgs::msg::Odometry::SharedPtr initial_pose_;
  double command_x_;
};

}  // namespace nav2_motion_primitives

#endif  // NAV2_MOTION_PRIMITIVES__BACK_UP_HPP_
