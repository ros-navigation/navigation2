// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#ifndef MISSION_EXECUTION__MISSIONEXECUTION_HPP_
#define MISSION_EXECUTION__MISSIONEXECUTION_HPP_

#include <string>
#include <memory>
#include "mission_execution/MissionExecutionTaskServer.hpp"
#include "mission_execution/MissionPlan.hpp"
#include "navigation/NavigateToPoseTaskClient.hpp"

class MissionExecution : public MissionExecutionTaskServer
{
public:
  explicit MissionExecution(const std::string & name);
  virtual ~MissionExecution();

  TaskStatus execute(const std_msgs::msg::String::SharedPtr command) override;

private:
  std::unique_ptr<NavigateToPoseTaskClient> navigationTask_;
};

#endif  // MISSION_EXECUTION__MISSIONEXECUTION_HPP_
