// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#ifndef MISSION_EXECUTION__MISSIONEXECUTIONSTATEMACHINE_HPP_
#define MISSION_EXECUTION__MISSIONEXECUTIONSTATEMACHINE_HPP_

#include "mission_execution/MissionExecutionStateMachineBehavior.hpp"

class MissionExecutionStateMachine
{
public:
  explicit MissionExecutionStateMachine(MissionExecutionStateMachineBehavior * behavior);
  virtual ~MissionExecutionStateMachine();

  // No default constructor
  MissionExecutionStateMachine() = delete;

  typedef enum MissionExecutionEvent
  {
    EVENT_CANCEL_MISSION,
    EVENT_EXECUTE_MISSION,
    EVENT_EXECUTE_RECOVERY,
    EVENT_EXECUTION_FAILED,
    EVENT_MISSION_CANCELED,
    EVENT_MISSION_EXECUTED,
    EVENT_MISSION_FAILED,
    EVENT_RECOVERY_FAILED,
    EVENT_RECOVERY_SUCCESSFUL
  } MissionExecutionEvent;

  void run();
  void fireEvent(const MissionExecutionEvent eventToFire);
  void halt();

private:
  void initStateMachine();
  bool isValidStateMachine();

  MissionExecutionStateMachineBehavior * behavior_;
  void * impl_;
};

#endif  // MISSION_EXECUTION__MISSIONEXECUTIONSTATEMACHINE_HPP_
