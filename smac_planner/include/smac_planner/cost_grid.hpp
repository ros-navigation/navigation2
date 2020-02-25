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

#ifndef SMAC_PLANNER__A_STAR_HPP_
#define SMAC_PLANNER__A_STAR_HPP_

#include <vector>
#include <iostream>
#include <queue>

#include "smac_planner/conversion_utils.hpp"
#include "smac_planner/pose.hpp"

namespace smac_planner
{

class Node 
{
public:
  Node(const double cost_in)
  : cost(cost_in),
    last_node(nullptr),
    in_closed_set(false)
  {
  }

  ~Node()
  {
    last_node = nullptr;
  }

  double getCost()
  {
    return cost;
  }

  bool wasVisited()
  {
    return visited;
  }

  void visited()
  {
    visited = true;
  }

  Node * last_node;

private:
  double cost;
  bool visited; // we can save on the potential field here: total 6 bytes per cell
}

// struct NodeComparator
// {
//   bool operator()(const Node & a, const Node & b) const
//   {
//     return a.cost > b.cost;
//   }
// };

// typedef NodeQueue std::priority_queue<Node*, std::vector<Node *>, NodeComparator>;

class CostGrid
{
public:
  CostGrid();
  ~CostGrid();

  void resize(const uint x, const uint y)
  {
    _nodes.clear();
    _nodes.reserve(x * y);
  }

  void addNode(const uint & i, const double & cost)
  {
    _nodes[i] = Node(cost); // this needs to already include travel cost??
  }

  std::vector<Node> _nodes;
}

}  // namespace smac_planner

#endif  // SMAC_PLANNER__A_STAR_HPP_
