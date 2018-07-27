// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#ifndef MISSION_EXECUTION__MISSIONEXECUTION_HPP_
#define MISSION_EXECUTION__MISSIONEXECUTION_HPP_

#include <string>
#include "mission_execution/MissionPlan.hpp"
#include "task/TaskServer.hpp"
#include "task/TaskClient.hpp"

class MissionExecution : public TaskServer
{
public:
  explicit MissionExecution(const std::string & name);
  virtual ~MissionExecution();

  TaskServer::Status execute(const CommandMsg::SharedPtr command) override;

private:
  TaskClient * navigateToPoseTask_;
};

#endif  // MISSION_EXECUTION__MISSIONEXECUTION_HPP_
