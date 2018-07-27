// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#ifndef NAVIGATION__SIMPLENAVIGATOR_HPP_
#define NAVIGATION__SIMPLENAVIGATOR_HPP_

#include "navigation/NavigateToPoseTask.hpp"
#include "task/TaskClient.hpp"

class SimpleNavigator : public NavigateToPoseTask
{
public:
  SimpleNavigator(const std::string & name, Robot * robot);
  SimpleNavigator() = delete;
  ~SimpleNavigator();

  // void navigateTo(/*pose*/) override;
  void execute();

protected:
  TaskClient * planner_;
  TaskClient * controller_;
};

#endif  // NAVIGATION__SIMPLENAVIGATOR_HPP_
