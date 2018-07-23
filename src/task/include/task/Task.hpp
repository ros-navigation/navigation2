// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#ifndef TASK__TASK_HPP_
#define TASK__TASK_HPP_

#include "rclcpp/rclcpp.hpp"

class Task
{
public:
  virtual ~Task() {}

protected:
  // Emulate ROS Action for now:
  // C++ templates for (Our)SimpleActionServer, (Our)SimpleActionClient
  //
  //   * Subscriber for ROS topic for commands w/ parameters
  //      * Execute Command
  //      * Cancel Command
  //   * Publisher for status updates
  //   * Publisher for completion
};

#endif  // TASK__TASK_HPP_
