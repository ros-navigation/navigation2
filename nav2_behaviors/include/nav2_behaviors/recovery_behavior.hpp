// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2018 Simbe Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_RECOVERY_MANAGER__RECOVERY_INTERFACE_HPP_
#define NAV2_RECOVERY_MANAGER__RECOVERY_INTERFACE_HPP_

#include <string>

namespace nav2_recovery_manager
{

// Provides an interface for defining a recovery behavior
class RecoveryBehavior
{
public:
  explicit RecoveryBehavior(const std::string name /*, RobotInterface, ROS node*/)
  : name_(name)
  {
  }

  virtual void initialize() = 0;

  virtual bool run() = 0;

  virtual ~RecoveryBehavior(){}

  std::string name() { return name_; };

protected:
  std::string name_;
};

}  // nav2_recovery_manager

#endif  // NAV2_RECOVERY_MANAGER__RECOVERY_INTERFACE_HPP_