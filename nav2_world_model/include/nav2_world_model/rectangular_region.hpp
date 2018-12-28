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

#ifndef NAV2_WORLD_MODEL__RECTANGULAR_REGION_HPP_
#define NAV2_WORLD_MODEL__RECTANGULAR_REGION_HPP_

#include "geometry_msgs/msg/point.hpp"

namespace nav2_world_model
{

// convenient for storing x/y point pairs
struct RectangularRegion
{
  RectangularRegion()
  : bottom_left_vertex_(geometry_msgs::msg::Point()),
    top_left_vertex_(geometry_msgs::msg::Point()),
    top_right_vertex_(geometry_msgs::msg::Point()),
    bottom_right_vertex_(geometry_msgs::msg::Point())
  {
  }

  RectangularRegion(const geometry_msgs::msg::Point & bl, const geometry_msgs::msg::Point & tl,
    const geometry_msgs::msg::Point tr, const geometry_msgs::msg::Point br)
  : bottom_left_vertex_(bl),
    top_left_vertex_(tl),
    top_right_vertex_(tr),
    bottom_right_vertex_(br)
  {
  }

  geometry_msgs::msg::Point bottom_left_vertex_;
  geometry_msgs::msg::Point top_left_vertex_;
  geometry_msgs::msg::Point top_right_vertex_;
  geometry_msgs::msg::Point bottom_right_vertex_;
};

}  // namespace nav2_world_model

#endif  // NAV2_WORLD_MODEL__RECTANGULAR_REGION_HPP_
