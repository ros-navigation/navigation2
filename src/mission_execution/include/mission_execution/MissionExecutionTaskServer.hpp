// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#ifndef MISSION_EXECUTION__MISSIONEXECUTIONTASKSERVER_HPP_
#define MISSION_EXECUTION__MISSIONEXECUTIONTASKSERVER_HPP_

#include "task/TaskServer.hpp"
#include "nav2_msgs/msg/mission_plan.hpp"

typedef TaskServer<nav2_msgs::msg::MissionPlan, std_msgs::msg::String> MissionExecutionTaskServer;

#endif  // MISSION_EXECUTION__MISSIONEXECUTIONTASKSERVER_HPP_
