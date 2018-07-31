// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#ifndef MISSION_EXECUTION__MISSIONEXECUTIONTASKCLIENT_HPP_
#define MISSION_EXECUTION__MISSIONEXECUTIONTASKCLIENT_HPP_

#include "task/TaskClient.hpp"
#include "nav2_msgs/msg/mission_plan.hpp"

typedef TaskClient<nav2_msgs::msg::MissionPlan, std_msgs::msg::String> MissionExecutionTaskClient;

#endif  // MISSION_EXECUTION__MISSIONEXECUTIONTASKCLIENT_HPP_
