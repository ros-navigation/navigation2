// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#ifndef PLANNING__PLANNINGTASK_HPP_
#define PLANNING__PLANNINGTASK_HPP_

#include "task/TaskServer.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class PlanningTask : public TaskServer
{
public:
  PlanningTask(const std::string & name);
  PlanningTask() = delete;
  ~PlanningTask();

  //virtual void createPlan(const geometry_msgs::msg::PoseStamped & start,
    //const geometry_msgs::msg::PoseStamped & goal) = 0;
};

#endif  // PLANNING__PLANNINGTASK_HPP_
