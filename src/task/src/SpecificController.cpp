// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#include "task/SpecificController.hpp"

SpecificController::SpecificController(const std::string & name, Robot * robot)
: ControlTask(name, robot)
{
}

SpecificController::~ControlTask()
{
}

void
SpecificController::executePlan()
{
}

void
SpecificController::workerThread()
{
}

