// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#ifndef NAVIGATION__SIMPLENAVIGATOR_HPP_
#define NAVIGATION__SIMPLENAVIGATOR_HPP_

#include "navigation/NavigateToPoseTask.hpp"
#include "planning/AStarPlanner.hpp"
#include "control/DwaController.hpp"

class SimpleNavigator : public NavigateToPoseTask
{
public:
  SimpleNavigator(const std::string & name, Robot * robot);
  SimpleNavigator() = delete;
  ~SimpleNavigator();

  void navigateTo(/*pose*/) override;

protected:
  void workerThread();

  // TODO: These will be the client-side proxies (like SimpleActionClient):
  AStarPlanner * planner_;
  DwaController * controller_;
};

#endif  // NAVIGATION__SIMPLENAVIGATOR_HPP_
