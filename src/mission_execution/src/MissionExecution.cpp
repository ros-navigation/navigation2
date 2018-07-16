// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#include "mission_execution/MissionExecution.hpp"

MissionExecution::MissionExecution()
: Node("mission_execution"),
  m_MissionExecutionStateMachine(this),
  m_missionPlan(nullptr)
{
}

MissionExecution::~MissionExecution()
{
}

void MissionExecution::executeMission(const MissionPlan * missionPlan)
{
  m_missionPlan = missionPlan;
}

void MissionExecution::cancelMission()
{
}

void MissionExecution::doReadyState()
{
}

void MissionExecution::doExecutingState()
{
}

void MissionExecution::doCancelingState()
{
}

void MissionExecution::doRecoveringState()
{
}

void MissionExecution::doAbortingState()
{
}
