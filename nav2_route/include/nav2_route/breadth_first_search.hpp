// Copyright (c) 2020, Samsung Research America
// Copyright (c) 2023 Joshua Wallace
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

#ifndef NAV2_ROUTE__BREADTH_FIRST_SEARCH_HPP_
#define NAV2_ROUTE__BREADTH_FIRST_SEARCH_HPP_

#include <queue>
#include <memory>
#include <unordered_map>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/line_iterator.hpp"

namespace nav2_route
{

/**
 * @struct nav2_route::SimpleNode
 * @brief A Node implementation for the breadth first search
 */
struct SimpleNode
{
  SimpleNode(unsigned int index)
  : index(index),
    explored(false) {}

  unsigned int index;
  bool explored;
};

/**
 * @class nav2_route::BreadthFirstSearch
 * @brief Preforms a breadth first search between the start and goal
 */
class BreadthFirstSearch
{
public:
  typedef SimpleNode * NodePtr;
  typedef std::vector<NodePtr> NodeVector;

  /**
   * @brief Initialize the search algorithm
   * @param costmap Costmap to use to check for state validity
   */
  void initialize(nav2_costmap_2d::Costmap2D * costmap, int max_iterations);

  /**
   * @brief Set the start of the breadth first search
   * @param mx The x map coordinate of the start
   * @param my The y map coordinate of the start
   */
  void setStart(unsigned int mx, unsigned int my);

  /**
   * @brief Set the goal of the breadth first search
   * @param goals The array of goals to search for
   */
  void setGoals(std::vector<nav2_costmap_2d::MapLocation> & goals);

  /**
   * @brief Find the closest goal to the start given a costmap, start and goal
   * @param goal The index of the goal array provided to the search
   * @throws nav2_core::PlannerTimedOut If the max iterations were reached
   * @throws nav2_core::NoValidPathCouldBeFound If no valid path could be found by the search
   */
  void search(unsigned int & goal);

  /**
   * @brief Preform a ray trace check to see if the node is directly visable
   * @param True if the node is visable
   */
  bool isNodeVisible();


  /**
   * @brief Get the graph
   */
  std::unordered_map<unsigned int, SimpleNode> * getGraph();

  /**
   * @brief clear the graph
   */
  void clearGraph();

private:
  /**
   * @brief Adds the node associated with the index to the graph
   * @param index The index of the node
   * @return node A pointer to the node added into the graph
   */
  NodePtr addToGraph(const unsigned int index);

  /**
   * @brief Retrieve all valid neighbors of a node
   * @param parent_index The index to the parent node of the neighbors
   */
  void getNeighbors(unsigned int parent_index, NodeVector & neighbors);

  /**
   * @brief Checks if the index is in collision
   * @param index The index to check
   */
  bool inCollision(unsigned int index);

  std::unordered_map<unsigned int, SimpleNode> graph_;

  NodePtr start_;
  NodeVector goals_;

  unsigned int x_size_;
  unsigned int y_size_;
  unsigned int max_index_;
  std::vector<int> neighbors_grid_offsets_;
  nav2_costmap_2d::Costmap2D * costmap_;
  int max_iterations_;
};
}  // namespace nav2_route

#endif  // NAV2_ROUTE__BREADTH_FIRST_SEARCH_HPP_
