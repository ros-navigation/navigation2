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

#ifndef NAV2_UTIL__POINT2D_HPP_
#define NAV2_UTIL__POINT2D_HPP_

#include <ostream>
#include <cmath>
#include "angleutils.hpp"

namespace nav2_util
{

struct Point2D
{
  double x, y;

  Point2D(const double x, const double y)
  : x(x), y(y) {}

  bool operator==(const Point2D & obj) const
  {
    if (x == obj.x && y == obj.y) {
      return true;
    } else {
      return false;
    }
  }

  friend std::ostream & operator<<(std::ostream & os, const Point2D & point)
  {
    return os <<
           "x: " << point.x <<
           " y: " << point.y;
  }
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__POINT2D_HPP_
