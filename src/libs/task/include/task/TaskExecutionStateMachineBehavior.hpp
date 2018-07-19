// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#ifndef TASK__TASKEXECUTIONSTATEMACHINEBEHAVIOR_HPP_
#define TASK__TASKEXECUTIONSTATEMACHINEBEHAVIOR_HPP_

class TaskExecutionStateMachineBehavior
{
public:
  virtual void doReadyState() = 0;
  virtual void doExecutingState() = 0;
  virtual void doCancelingState() = 0;
  virtual void doRecoveringState() = 0;
  virtual void doAbortingState() = 0;
};

#endif  // TASK__TASKEXECUTIONSTATEMACHINEBEHAVIOR_HPP_
