// Copyright (c) 2018 Intel Corporation
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

#ifndef NAV2_ROBOT__ROBOT_HPP_
#define NAV2_ROBOT__ROBOT_HPP_

namespace nav2_robot
{

class Robot
{
public:
  virtual ~Robot() {}

  // Commands to the robot
  virtual void enterSafeState() = 0;
};

}  // namespace nav2_robot

#endif  // NAV2_ROBOT__ROBOT_HPP_
