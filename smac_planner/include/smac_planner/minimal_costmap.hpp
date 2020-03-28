// Copyright (c) 2020, Samsung Research America
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
// limitations under the License. Reserved.

#ifndef SMAC_PLANNER__MINIMAL_COSTMAP_HPP_
#define SMAC_PLANNER__MINIMAL_COSTMAP_HPP_

#include <cmath>
#include <vector>
#include <iostream>
#include <unordered_map>
#include <memory>
#include <queue>
#include <utility>

// doxyogen
// this is another argument for a costmap interface header

namespace smac_planner
{

class MinimalCostmap
{
public:
  MinimalCostmap(
    unsigned char * & char_costmap,
    const unsigned int & size_x,
    const unsigned int & size_y, 
    const double & origin_x,
    const double & origin_y,
    const double & resolution)
  : _char_costmap(char_costmap),
    _size_x(size_x),
    _size_y(size_y),
    _origin_x(origin_x),
    _origin_y(origin_y),
    _resolution(resolution)
  {
  }

  inline double getCost(const int & mx, const int & my) const
  {
    return _char_costmap[getIndex(mx, my)];
  }

  inline bool worldToMap(const double & wx, const double & wy, unsigned int & mx, unsigned int & my) const
  {
    if (wx < _origin_x || wy < _origin_y) {
      return false;
    }

    mx = static_cast<int>((wx - _origin_x) / _resolution);
    my = static_cast<int>((wy - _origin_y) / _resolution);

    if (mx < _size_x && my < _size_y) {
      return true;
    }

    return false;
  }

  inline unsigned int getIndex(const unsigned int & mx, const unsigned int & my) const
  {
    return my * _size_x + mx;
  }

  inline unsigned int sizeX() const
  {
    return _size_x;
  }

  inline unsigned int sizeY() const
  {
    return _size_y;
  }


protected:
  unsigned char * _char_costmap;
  unsigned int _size_x;
  unsigned int _size_y;
  double _origin_x;
  double _origin_y;
  double _resolution;

};

}  // namespace smac_planner

#endif  // SMAC_PLANNER__MINIMAL_COSTMAP_HPP_
