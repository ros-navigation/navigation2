// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#ifndef TASK__PLANNINGTASK_HPP_
#define TASK__PLANNINGTASK_HPP_

#include "task/Task.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class PlanningTask : public Task
{
public:
  PlanningTask(const std::string & name);
  PlanningTask() = delete;
  ~PlanningTask();

  virtual void createPlan(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) = 0;
};

#endif  // TASK__PLANNINGTASK_HPP_
