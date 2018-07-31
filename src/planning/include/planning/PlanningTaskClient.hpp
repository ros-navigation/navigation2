// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#ifndef PLANNING__PLANNINGTASKCLIENT_HPP_
#define PLANNING__PLANNINGTASKCLIENT_HPP_

#include "task/TaskClient.hpp"
#include "nav2_msgs/msg/path_end_points.hpp"
#include "nav2_msgs/msg/path.hpp"

typedef TaskClient<nav2_msgs::msg::PathEndPoints, nav2_msgs::msg::Path> PlanningTaskClient;

#endif  // PLANNING__PLANNINGTASKCLIENT_HPP_
