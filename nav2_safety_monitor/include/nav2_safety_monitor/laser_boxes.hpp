// Copyright (c) 2019 Steve Macenski
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

#ifndef NAV2_SAFETY_MONITOR__LASER_BOXES_HPP_
#define NAV2_SAFETY_MONITOR__LASER_BOXES_HPP_

namespace nav2_safety_monitor
{

struct LaserBoxes
{
  LaserBoxes() {
    return;
  }
  
  LaserBoxes(std::vector<double>& vals) {
    width = vals.at(0);
    length = vals.at(1);
    offset_x = vals.at(2);
    offset_y = vals.at(3);
  }

  double offset_x, offset_y;
  double width, length;
};

} // end namespace nav2_safety_monitor

#endif //NAV2_SAFETY_MONITOR__LASER_BOXES_HPP_
