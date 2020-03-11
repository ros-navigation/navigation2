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

#ifndef SMAC_PLANNER__NODE_HPP_
#define SMAC_PLANNER__NODE_HPP_

#include <vector>
#include <iostream>
#include <queue>
#include <limits>

#include "smac_planner/conversion_utils.hpp"

namespace smac_planner
{

class Node 
{
public:
  Node(char cost_in, const unsigned int index)
  : last_node(nullptr),
    cell_cost(static_cast<float>(cost_in)),
    accumulated_cost(std::numeric_limits<float>::max()),
    i(index),
    was_visited(false),
    is_queued(false)
  {
  }

  ~Node()
  {
    last_node = nullptr;
  }

  bool operator==(const Node & rhs)
  {
    return this->i == rhs.i;
  }

  float & getAccumulatedCost()
  {
    return accumulated_cost;
  }

  void setAccumulatedCost(const float cost_in)
  {
    accumulated_cost = cost_in;
  }

  float & getCost()
  {
    return cell_cost;
  }

  bool wasVisited()
  {
    return was_visited;
  }

  void visited()
  {
    was_visited = true;
    is_queued = false;
  }

  bool isQueued()
  {
    return is_queued;
  }

  void queued()
  {
    is_queued = true;
  }

  unsigned int & getIndex()
  {
    return i;
  }

  Node * last_node;

private:
  float cell_cost;
  float accumulated_cost;
  unsigned int i;
  bool was_visited;
  bool is_queued;
};

}  // namespace smac_planner

#endif  // SMAC_PLANNER__NODE_HPP_
