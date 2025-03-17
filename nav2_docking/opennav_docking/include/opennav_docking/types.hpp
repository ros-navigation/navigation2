// Copyright (c) 2024 Open Navigation LLC
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

#ifndef OPENNAV_DOCKING__TYPES_HPP_
#define OPENNAV_DOCKING__TYPES_HPP_

#include <unordered_map>
#include <string>

#include "nav2_msgs/action/dock_robot.hpp"
#include "nav2_msgs/action/undock_robot.hpp"
#include "opennav_docking_core/charging_dock.hpp"
#include "opennav_docking_core/docking_exceptions.hpp"

typedef nav2_msgs::action::DockRobot DockRobot;
typedef nav2_msgs::action::UndockRobot UndockRobot;

/**
* @struct A dock instance struct for a database
*/
struct Dock
{
  geometry_msgs::msg::PoseStamped getStagingPose()
  {
    return this->plugin->getStagingPose(this->pose, this->frame);
  }

  geometry_msgs::msg::Pose pose;
  std::string frame;
  std::string type;
  std::string id;
  opennav_docking_core::ChargingDock::Ptr plugin{nullptr};
};

using opennav_docking_core::ChargingDock;
using DockPluginMap = std::unordered_map<std::string, opennav_docking_core::ChargingDock::Ptr>;
using DockMap = std::unordered_map<std::string, Dock>;

#endif  // OPENNAV_DOCKING__TYPES_HPP_
