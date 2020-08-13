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
#include <unordered_map>
#include <memory>
#include <queue>
#include <utility>

#include "smac_planner/node.hpp"
#include "smac_planner/types.hpp"
#include "smac_planner/constants.hpp"

namespace smac_planner
{

/**
 * @class smac_planner::AStarAlgorithm
 * @brief An A* implementation for planning in a costmap. Templated based on the Node type.
 */
template<typename NodeT>
class AStarAlgorithm
{
public:
  typedef NodeT * NodePtr;
  typedef std::vector<NodeT> Graph;
  typedef std::vector<NodePtr> NodeVector;
  typedef std::pair<float, NodePtr> NodeElement;
  typedef std::pair<float, unsigned int> NodeHeuristicPair;

  struct NodeComparator
  {
    bool operator()(const NodeElement & a, const NodeElement & b) const
    {
      return a.first > b.first;
    }
  };

  typedef std::priority_queue<NodeElement, std::vector<NodeElement>, NodeComparator> NodeQueue;

  /**
   * @brief A constructor for smac_planner::PlannerServer
   * @param neighborhood The type of neighborhood to use for search (4 or 8 connected)
   */
  explicit AStarAlgorithm(const Neighborhood & neighborhood);

  /**
   * @brief A destructor for smac_planner::AStarAlgorithm
   */
  ~AStarAlgorithm();

  /**
   * @brief Initialization of the planner with defaults
   * @param travel_cost_scale Cost to travel from adjacent nodes to another
   * @param allow_unknown Allow search in unknown space, good for navigation while mapping
   * @param max_iterations Maximum number of iterations to use while expanding search
   * @param max_on_approach_iterations Maximum number of iterations before returning a valid
   * path once within thresholds to refine path
   * comes at more compute time but smoother paths.
   */
  void initialize(
    const float & travel_cost_scale,
    const bool & allow_unknown,
    int & max_iterations,
    const int & max_on_approach_iterations);

  /**
   * @brief Creating path from given costmap, start, and goal
   * @param path Reference to a vector of indicies of generated path
   * @param num_iterations Reference to number of iterations to create plan
   * @param tolerance Reference to tolerance in costmap nodes
   * @return if plan was successful
   */
  bool createPath(IndexPath & path, int & num_iterations, const float & tolerance);

  /**
   * @brief Set the costs of the graph
   * @param x The total number of nodes in the X direction
   * @param y The total number of nodes in the X direction
   * @param costs unsigned char * to the costs in the graph
   */
  void setCosts(
    const unsigned int & x,
    const unsigned int & y,
    unsigned char * & costs);

  /**
   * @brief Set the goal for planning, as a node index
   * @param value The node index of the goal
   */
  void setGoal(const unsigned int & value);

  /**
   * @brief Set the starting pose for planning, as a node index
   * @param value The node index of the start
   */
  void setStart(const unsigned int & value);

  /**
   * @brief Set the starting pose for planning, as a node index
   * @param node Node pointer to the goal node to backtrace
   * @param path Reference to a vector of indicies of generated path
   * @return whether the path was able to be backtraced
   */
  bool backtracePath(NodePtr & node, IndexPath & path);

  /**
   * @brief Get maximum number of iterations to plan
   * @return Reference to Maximum iterations parameter
   */
  int & getMaxIterations();

private:
  /**
   * @brief Get pointer reference to starting node
   * @return Node pointer reference to starting node
   */
  inline NodePtr & getStart();

  /**
   * @brief Get pointer reference to goal node
   * @return Node pointer reference to goal node
   */
  inline NodePtr & getGoal();

  /**
   * @brief Get pointer to next goal in open set
   * @return Node pointer reference to next heuristically scored node
   */
  inline NodePtr getNode();

  /**
   * @brief Get pointer to next goal in open set
   * @param cost The cost to sort into the open set of the node
   * @param node Node pointer reference to add to open set
   */
  inline void addNode(const float cost, NodePtr & node);

  /**
   * @brief Retrieve all valid neighbors of a node.
   * @param node Pointer to the node we are currently exploring in A*
   * @param neighbors Vector of neighbors to be filled
   */
  inline void getNeighbors(NodePtr & node, NodeVector & neighbors);

  /**
   * @brief Initialize the neighborhood to be used in A*
   * For now we support 4-connect (VON_NEUMANN) and 8-connect (MOORE)
   * @param x_size The size of the underlying grid
   * @param neighborhood The desired neighborhood type
   */
  inline void initNeighborhoods(const int & x_size, const Neighborhood & neighborhood);

  /**
   * @brief Check if this node is the goal node
   * @param node Node pointer to check if its the goal node
   * @return if node is goal
   */
  inline bool isGoal(NodePtr & node);

  /**
   * @brief Get cost of traversal between nodes
   * @param current_node Pointer to current node
   * @param new_node Pointer to new node
   * @return Reference traversal cost between the nodes
   */
  inline float getTraversalCost(NodePtr & current_node, NodePtr & new_node);

  /**
   * @brief Get cost of heuristic of node
   * @param node Node index current
   * @param node Node index of new
   * @return Heuristic cost between the nodes
   */
  inline float getHeuristicCost(const unsigned int & node);

  /**
   * @brief Check if inputs to planner are valid
   * @return Are valid
   */
  inline bool areInputsValid();

  /**
   * @brief Get maximum number of on-approach iterations after within threshold 
   * @return Reference to Maximum on-appraoch iterations parameter
   */
  inline int & getOnApproachMaxIterations();

  /**
   * @brief Get tolerance, in node nodes
   * @return Reference to tolerance parameter
   */
  inline float & getToleranceHeuristic();

  /**
   * @brief Get size of graph in X
   * @return Size in X
   */
  inline unsigned int & getSizeX();

  /**
   * @brief Get size of graph in Y
   * @return Size in Y
   */
  inline unsigned int & getSizeY();

  /**
   * @brief Get node coordinates from index
   * @param index Index of node
   * @return pair of XY coordinates
   */
  inline Coordinates getCoords(const unsigned int & index);

  /**
   * @brief Clear hueristic queue of nodes to search
   */
  inline void clearQueue();

  float _travel_cost_scale;
  float _neutral_cost;
  bool _traverse_unknown;
  int _max_iterations;
  int _max_on_approach_iterations;
  float _tolerance;
  unsigned int _x_size;
  unsigned int _y_size;

  Coordinates _goal_coordinates;
  NodePtr _start;
  NodePtr _goal;

  std::unique_ptr<Graph> _graph;
  std::unique_ptr<NodeQueue> _queue;

  Neighborhood _neighborhood;
  std::vector<int> _neighbors_grid_offsets;
  NodeHeuristicPair _best_heuristic_node;
};

}  // namespace smac_planner

#endif  // SMAC_PLANNER__A_STAR_HPP_
