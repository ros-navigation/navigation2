// Copyright (c) 2022 Samsung R&D Institute Russia
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

#ifndef NAV2_COSTMAP_2D__ORDER_LAYER_HPP_
#define NAV2_COSTMAP_2D__ORDER_LAYER_HPP_

#include "nav2_costmap_2d/layer.hpp"

namespace nav2_costmap_2d
{

class OrderLayer : public nav2_costmap_2d::Layer
{
public:
  OrderLayer();

  virtual void activate();
  virtual void deactivate();

  virtual void reset() {}
  virtual bool isClearable() {return false;}

  virtual void updateBounds(
    double, double, double, double *, double *, double *, double *);

  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D &, int, int, int, int);

private:
  bool activated_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__ORDER_LAYER_HPP_
