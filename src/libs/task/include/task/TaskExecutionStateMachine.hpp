// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#ifndef TASK__TASKEXECUTIONSTATEMACHINE_HPP_
#define TASK__TASKEXECUTIONSTATEMACHINE_HPP_

#include "task/TaskExecutionStateMachineBehavior.hpp"

class TaskExecutionStateMachine
{
public:
  explicit TaskExecutionStateMachine(TaskExecutionStateMachineBehavior * behavior);
  virtual ~TaskExecutionStateMachine();

  // No default constructor
  TaskExecutionStateMachine() = delete;

  typedef enum TaskExecutionEvent
  {
    EVENT_CANCEL_TASK,
    EVENT_EXECUTE_TASK,
    EVENT_EXECUTE_TASK_RECOVERY,
    EVENT_TASK_EXECUTION_FAILED,
    EVENT_TASK_CANCELED,
    EVENT_TASK_EXECUTED,
    EVENT_TASK_FAILED,
    EVENT_TASK_RECOVERY_FAILED,
    EVENT_TASK_RECOVERY_SUCCEEDED,
  } TaskExecutionEvent;

  void run();
  void fireEvent(const TaskExecutionEvent eventToFire);
  void halt();

private:
  void initStateMachine();
  bool isValidStateMachine();

  TaskExecutionStateMachineBehavior * behavior_;
  void * impl_;
};

#endif  // TASK__TASKEXECUTIONSTATEMACHINE_HPP_
