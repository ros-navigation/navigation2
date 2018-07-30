// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#ifndef CONTROL__DWACONTROLLER_HPP_
#define CONTROL__DWACONTROLLER_HPP_

#include "control/ControlTaskServer.hpp"
#include "robot/Robot.hpp"

class DwaController : public ControlTaskServer
{
public:
  DwaController(const std::string & name, Robot * robot);
  DwaController() = delete;
  ~DwaController();

  TaskServer::Status execute(const std_msgs::msg::String::SharedPtr command) override;
};

#endif  // CONTROL__DWACONTROLLER_HPP_
