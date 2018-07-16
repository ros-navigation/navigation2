// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#include "mission_execution/MissionExecutionStateMachine.hpp"

MissionExecutionStateMachine::MissionExecutionStateMachine(
  MissionExecutionStateMachineBehavior * behavior)
: m_behavior(behavior)
{
}

MissionExecutionStateMachine::~MissionExecutionStateMachine()
{
}

void MissionExecutionStateMachine::run()
{
}

void MissionExecutionStateMachine::fireEvent(const MissionExecutionEvent /*eventToFire*/)
{
}

void MissionExecutionStateMachine::halt()
{
}
