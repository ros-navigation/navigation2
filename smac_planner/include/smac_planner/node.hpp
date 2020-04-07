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

namespace smac_planner
{

/**
 * @class smac_planner::Node
 * @brief Node implementation for graph
 */
class Node
{
public:
  /**
   * @brief A constructor for smac_planner::Node
   * @param cost_in The costmap cost at this node
   * @param index The index of this node for self-reference
   */
  explicit Node(unsigned char & cost_in, const unsigned int index)
  : last_node(nullptr),
    cell_cost(static_cast<float>(cost_in)),
    accumulated_cost(std::numeric_limits<float>::max()),
    i(index),
    was_visited(false),
    is_queued(false)
  {
  }

  /**
   * @brief A destructor for smac_planner::Node
   */
  ~Node()
  {
    last_node = nullptr;
  }

  /**
   * @brief operator== for comparisons
   * @param Node right hand side node reference
   * @return If cell indicies are equal
   */
  bool operator==(const Node & rhs)
  {
    return this->i == rhs.i;
  }

  /**
   * @brief Reset method for new search
   * @param cost_in The costmap cost at this node
   * @param index The index of this node for self-reference
   */
  void reset(const unsigned char & cost, const unsigned int index)
  {
    last_node = nullptr;
    cell_cost = static_cast<float>(cost);
    accumulated_cost = std::numeric_limits<float>::max();
    i = index;
    was_visited = false;
    is_queued = false;
  }

  /**
   * @brief Gets the accumulated cost at this node
   * @return accumulated cost
   */
  float & getAccumulatedCost()
  {
    return accumulated_cost;
  }

  /**
   * @brief Sets the accumulated cost at this node
   * @param reference to accumulated cost
   */
  void setAccumulatedCost(const float cost_in)
  {
    accumulated_cost = cost_in;
  }

  /**
   * @brief Gets the costmap cost at this node
   * @return costmap cost
   */
  float & getCost()
  {
    return cell_cost;
  }

  /**
   * @brief Gets if cell has been visited in search
   * @param If cell was visited
   */
  bool & wasVisited()
  {
    return was_visited;
  }

  /**
   * @brief Sets if cell has been visited in search
   */
  void visited()
  {
    was_visited = true;
    is_queued = false;
  }

  /**
   * @brief Gets if cell is currently queued in search
   * @param If cell was queued
   */
  bool & isQueued()
  {
    return is_queued;
  }

  /**
   * @brief Sets if cell is currently queued in search
   */
  void queued()
  {
    is_queued = true;
  }

  /**
   * @brief Gets cell index
   * @return Reference to cell index
   */
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
