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

#include <memory>
#include <string>

#include "opennav_docking_bt/undock_robot.hpp"

namespace opennav_docking_bt
{

UndockRobotAction::UndockRobotAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<Action>(xml_tag_name, action_name, conf)
{
}

void UndockRobotAction::on_tick()
{
  // Get core inputs about what to perform
  getInput("dock_type", goal_.dock_type);
  getInput("max_undocking_time", goal_.max_undocking_time);
}

BT::NodeStatus UndockRobotAction::on_success()
{
  setOutput("success", result_.result->success);
  setOutput("error_code_id", ActionResult::NONE);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus UndockRobotAction::on_aborted()
{
  setOutput("success", result_.result->success);
  setOutput("error_code_id", result_.result->error_code);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus UndockRobotAction::on_cancelled()
{
  setOutput("error_code_id", ActionResult::NONE);
  return BT::NodeStatus::SUCCESS;
}

void UndockRobotAction::halt()
{
  BtActionNode::halt();
}

}  // namespace opennav_docking_bt

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<opennav_docking_bt::UndockRobotAction>(
        name, "undock_robot", config);
    };

  factory.registerBuilder<opennav_docking_bt::UndockRobotAction>(
    "UndockRobot", builder);
}
