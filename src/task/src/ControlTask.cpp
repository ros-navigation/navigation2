// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#include "task/ControlTask.hpp"

ControlTask::ControlTask(const std::string & name, Robot * robot)
: RobotTask(name, robot)
{
}

ControlTask::~ControlTask()
{
}
