// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#include "task/NavigationTask.hpp"

NavigationTask::NavigationTask(Robot * robot)
: RobotTask(robot), planner_(robot), controller_(robot)
{
}

NavigationTask::~NavigationTask()
{
}
