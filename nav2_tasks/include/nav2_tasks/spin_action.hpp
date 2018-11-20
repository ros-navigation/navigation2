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

#ifndef NAV2_TASKS__SPIN_ACTION_HPP_
#define NAV2_TASKS__SPIN_ACTION_HPP_

#include <string>
#include <memory>
#include <cmath>

#include "nav2_tasks/bt_conversions.hpp"
#include "nav2_tasks/bt_action_node.hpp"
#include "nav2_tasks/spin_task.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace nav2_tasks
{

class SpinAction : public BtActionNode<SpinCommand, SpinResult>
{
public:
  explicit SpinAction(const std::string & action_name)
  : BtActionNode<SpinCommand, SpinResult>(action_name)
  {
    // Create the input and output messages
    command_ = std::make_shared<nav2_tasks::SpinCommand>();
    result_ = std::make_shared<nav2_tasks::SpinResult>();

    tf2::Quaternion quaternion;
    // Rotate 90deg CCW
    quaternion.setRPY(0, 0, M_PI/2); // yaw, pitch and roll are rotation in z, y, x respectively
    // quaternion.normalize();

    geometry_msgs::msg::Quaternion quaternion_msg;
    quaternion_msg = tf2::toMsg(quaternion);

    command_->quaternion.x = quaternion_msg.x;
    command_->quaternion.y = quaternion_msg.y;
    command_->quaternion.z = quaternion_msg.z;
    command_->quaternion.w = quaternion_msg.w;
  }
};

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__SPIN_ACTION_HPP_
