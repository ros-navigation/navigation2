// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#ifndef NAVIGATION__SIMPLENAVIGATOR_HPP_
#define NAVIGATION__SIMPLENAVIGATOR_HPP_

#include <memory>
#include "navigation/NavigateToPoseTaskServer.hpp"
#include "planning/PlanningTaskClient.hpp"
#include "control/ControlTaskClient.hpp"
#include "robot/Robot.hpp"

class SimpleNavigator : public NavigateToPoseTaskServer
{
public:
  SimpleNavigator(const std::string & name, Robot * robot);
  SimpleNavigator() = delete;
  ~SimpleNavigator();

  TaskStatus execute(const std_msgs::msg::String::SharedPtr command) override;

protected:
  std::unique_ptr<PlanningTaskClient> planner_;
  std::unique_ptr<ControlTaskClient> controller_;
};

#endif  // NAVIGATION__SIMPLENAVIGATOR_HPP_
