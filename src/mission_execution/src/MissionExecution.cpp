// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#include "mission_execution/MissionExecution.hpp"

MissionExecution::MissionExecution()
: Node("mission_execution"),
  missionExecutionStateMachine_(this),
  missionPlan_(nullptr)
{
}

MissionExecution::~MissionExecution()
{
}

void MissionExecution::executeMission(const MissionPlan * missionPlan)
{
  missionPlan_ = missionPlan;
}

void MissionExecution::cancelMission()
{
  // TODO: Do the work to cancel the in-flight mission

  // Initiate the state transition
  missionExecutionStateMachine_.fireEvent(MissionExecutionStateMachine::EVENT_MISSION_CANCELED);
}

void MissionExecution::doReadyState()
{
  // TODO: Wait for a mission to execute

  // TODO: Once we receive a mission plan, we may want to do some work, such as validating its syntax and semantics

  // TODO: Now that we know the mission plan is ready for execution, get started

  // TODO: Normal processing loop for the mission plan


  // Initiate the state transition
  missionExecutionStateMachine_.fireEvent(MissionExecutionStateMachine::EVENT_MISSION_EXECUTED);
  // Or:
  missionExecutionStateMachine_.fireEvent(MissionExecutionStateMachine::EVENT_MISSION_FAILED);
  // Or:
  missionExecutionStateMachine_.fireEvent(MissionExecutionStateMachine::EVENT_EXECUTE_RECOVERY);
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
  // TODO: Do the work to abort the current mission

  // Initiate the state transition
  missionExecutionStateMachine_.fireEvent(MissionExecutionStateMachine::EVENT_MISSION_FAILED);
}
