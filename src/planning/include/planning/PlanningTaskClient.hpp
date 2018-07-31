// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#ifndef PLANNING__PLANNINGTASKCLIENT_HPP_
#define PLANNING__PLANNINGTASKCLIENT_HPP_

#include "task/TaskClient.hpp"

typedef TaskClient<std_msgs::msg::String, std_msgs::msg::String> PlanningTaskClient;
//typedef TaskClient<nav2_msgs::msg::PathEndPoints, nav2_msgs::msg::Path> PlanningTaskClient;

#endif  // PLANNING__PLANNINGTASKCLIENT_HPP_
