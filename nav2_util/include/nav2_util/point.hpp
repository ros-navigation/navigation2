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

#ifndef NAV2_UTIL__POINT_HPP_
#define NAV2_UTIL__POINT_HPP_

#include <ostream>
#include <cmath>
#include "angleutils.hpp"

namespace nav2_util
{

struct Point
{
  double x, y;

  Point()
  : x(0), y(0)
  {
  }

  Point(const double x, const double y)
  : x(x), y(y)
  {
  }

  bool operator==(const Point & obj) const
  {
    if (x == obj.x && y == obj.y) {
      return true;
    } else {
      return false;
    }
  }

  friend std::ostream & operator<<(std::ostream & os, const Point & point)
  {
    return os <<
           "x: " << point.x <<
           " y: " << point.y;
  }

  void rotateAroundPoint(const double theta, const Point & reference)
  {
    // translate point such that the reference is now the origin
    double x_t = x - reference.x;
    double y_t = y - reference.y;

    // rotate
    double angle = angleutils::normalize(theta);
    double x_tr = x_t * cos(angle) - y_t * sin(angle);
    double y_tr = x_t * sin(angle) + y_t * cos(angle);

    // undo the translation
    x = x_tr + reference.x;
    y = y_tr + reference.y;
  }
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__POINT_HPP_
