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

#include "nav2_route/node.hpp"

namespace nav2_route
{

class BreadthFirstSearch
{
public:
  typedef std::unordered_map<unsigned int, Node> Graph;
  typedef Node::NodePtr NodePtr;
  typedef Node::NodeVector NodeVector;
  typedef Node::NodeVector::iterator NeighborIterator;
  typedef Node::Coordinates Coordinates;
  typedef Node::CoordinateVector CoordinateVector;
  typedef Node::NodeGetter NodeGetter;
  typedef std::queue<NodePtr> NodeQueue;

  /**
   * @brief A constructor for nav2_route::BreadthFirstSearch
   */
  BreadthFirstSearch() = default;

  /**
   * @brief Set the starting pose for the search
   * @param mx The node X index of the start
   * @param my The node Y index of the start
   */
  void setStart(
    const unsigned int & mx,
    const unsigned int & my);

 /**
  * @brief Set the goal for the search
  * @param mx The node X index of the start
  * @param my The node Y index of the start
  */
  void setGoal(
    const unsigned int & mx,
    const unsigned int & my);

  /**
   * @brief Create a path from the given costmap, start and goal
   * @param path The output path if the search was successful
   * @return True if a plan was successfully calculated
   */
  bool search(CoordinateVector & path);

  void initialize(unsigned int x_size, unsigned int y_size);

  void setCollisionChecker(CollisionChecker * collision_checker);

private:

  /**
   * @brief Checks if node is the goal node
   * @param node The node to check
   * @return True if the node is the goal node
   */
  inline bool isGoal(NodePtr & node);

  /**
   * @brief Adds a node to the graph
   * @param index The index of the node to add
   * @return The node added to the graph
   */
  NodePtr addToGraph(const unsigned int & index);

  /**
   * @brief Adds a node to the queue
   * @param node The node to add
   */
  void addToQueue(NodePtr & node);

  /**
   * @brief Gets the next node in the queue
   * @return The next node in the queue
   */
  NodePtr getNextNode();

  /**
   * @brief Clear the queue
   */
  void clearQueue();

  /**
   * @brief Clear the graph
   */
  void clearGraph();

  Graph graph_;
  NodePtr start_{nullptr};
  NodePtr goal_{nullptr};
  NodeQueue queue_;

  unsigned int x_size_{0};
  unsigned int y_size_{0};

  CollisionChecker *collision_checker_{nullptr};
};
}  // namespace nav2_route

#endif  // NAV2_ROUTE__BREADTH_FIRST_SEARCH_HPP_
