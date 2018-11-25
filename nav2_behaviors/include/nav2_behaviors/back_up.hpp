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

#ifndef NAV2_BEHAVIORS__BACK_UP_HPP_
#define NAV2_BEHAVIORS__BACK_UP_HPP_

#include <string>
#include <memory>

#include "nav2_tasks/back_up_task.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/empty.hpp"
#include "nav2_robot/robot.hpp"

namespace nav2_behaviors
{

class BackUp : public rclcpp::Node
{
public:
  BackUp();
  ~BackUp();

  nav2_tasks::TaskStatus backUp(const nav2_tasks::BackUpCommand::SharedPtr command);

protected:
  double min_linear_vel_, max_linear_vel_, linear_acc_lim_, goal_tolerance_distance_;

  std::unique_ptr<nav2_robot::Robot> robot_;

  std::unique_ptr<nav2_tasks::BackUpTaskServer> task_server_;

  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> vel_pub_;

  nav2_tasks::TaskStatus timedBackup();

  nav2_tasks::TaskStatus controlledBackup();
};

}  // nav2_behaviors

#endif  // NAV2_BEHAVIORS__BACK_UP_HPP_
