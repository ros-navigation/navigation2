// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#ifndef MISSION_EXECUTION__MISSIONEXECUTIONSTATEMACHINEBEHAVIOR_HPP_
#define MISSION_EXECUTION__MISSIONEXECUTIONSTATEMACHINEBEHAVIOR_HPP_

class MissionExecutionStateMachineBehavior
{
public:
  virtual void doReadyState() = 0;
  virtual void doExecutingState() = 0;
  virtual void doCancelingState() = 0;
  virtual void doRecoveringState() = 0;
  virtual void doAbortingState() = 0;
};

#endif  // MISSION_EXECUTION__MISSIONEXECUTIONSTATEMACHINEBEHAVIOR_HPP_
