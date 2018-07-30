// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#ifndef MISSION_EXECUTION__MISSIONEXECUTIONTASKCLIENT_HPP_
#define MISSION_EXECUTION__MISSIONEXECUTIONTASKCLIENT_HPP_

#include "task/TaskClient.hpp"

typedef TaskClient<std_msgs::msg::String, std_msgs::msg::String> MissionExecutionTaskClient;

#endif  // MISSION_EXECUTION__MISSIONEXECUTIONTASKCLIENT_HPP_
