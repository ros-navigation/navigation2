// Copyright (c) 2019 Intel Corporation
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

#ifndef NAV2_UTIL__IS_LOCALIZED_HPP_
#define NAV2_UTIL__IS_LOCALIZED_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "nav2_robot/robot.hpp"

namespace nav2_util
{

class IsLocalized
{

public:
  IsLocalized(const rclcpp::Node::SharedPtr & node);
  bool isLocalized();

private:
  static const int cov_x_ = 0;
  static const int cov_y_ = 7;
  static const int cov_a_ = 35;

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<nav2_robot::Robot> robot_;

  double x_tol_;
  double y_tol_;
  double rot_tol_;
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__IS_LOCALIZED_HPP_
