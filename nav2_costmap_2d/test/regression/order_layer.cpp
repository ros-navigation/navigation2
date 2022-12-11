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

#include "order_layer.hpp"

#include <chrono>
#include <stdexcept>

using namespace std::chrono_literals;

namespace nav2_costmap_2d
{

OrderLayer::OrderLayer()
: activated_(false)
{
}

void OrderLayer::activate()
{
  std::this_thread::sleep_for(100ms);
  activated_ = true;
}

void OrderLayer::deactivate()
{
  activated_ = false;
}

void OrderLayer::updateBounds(
  double, double, double, double *, double *, double *, double *)
{
  if (!activated_) {
    throw std::runtime_error("update before activated");
  }
}

void OrderLayer::updateCosts(
  nav2_costmap_2d::Costmap2D &, int, int, int, int)
{
  if (!activated_) {
    throw std::runtime_error("update before activated");
  }
}

}  // namespace nav2_costmap_2d

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::OrderLayer, nav2_costmap_2d::Layer)
