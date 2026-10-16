// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2019 Steven Macenski
// Copyright (c) 2019 Samsung Research America
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

#include <cmath>

#include "nav2_util/robot_utils.hpp"

namespace nav2_util
{

bool validateTwist(const geometry_msgs::msg::Twist & msg)
{
  if (std::isinf(msg.linear.x) || std::isnan(msg.linear.x)) {
    return false;
  }

  if (std::isinf(msg.linear.y) || std::isnan(msg.linear.y)) {
    return false;
  }

  if (std::isinf(msg.linear.z) || std::isnan(msg.linear.z)) {
    return false;
  }

  if (std::isinf(msg.angular.x) || std::isnan(msg.angular.x)) {
    return false;
  }

  if (std::isinf(msg.angular.y) || std::isnan(msg.angular.y)) {
    return false;
  }

  if (std::isinf(msg.angular.z) || std::isnan(msg.angular.z)) {
    return false;
  }

  return true;
}

}  // end namespace nav2_util
