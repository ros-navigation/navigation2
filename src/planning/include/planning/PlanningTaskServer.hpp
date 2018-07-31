// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#ifndef PLANNING__PLANNINGTASKSERVER_HPP_
#define PLANNING__PLANNINGTASKSERVER_HPP_

#include "task/TaskServer.hpp"

typedef TaskServer<std_msgs::msg::String, std_msgs::msg::String> PlanningTaskServer;
//typedef TaskServer<nav2_msgs::msg::PathEndPoints, nav2_msgs::msg::Path> PlanningTaskClient;

#endif  // PLANNING__PLANNINGTASKSERVER_HPP_
