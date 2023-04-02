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
  typedef Node::Coordinates Coordinates;
  typedef Node::CoordinateVector CoordinateVector;
  typedef std::queue<NodePtr> NodeQueue;

  BreadthFirstSearch();

  void setStart(
    const unsigned int & mx,
    const unsigned int & my);

  // TODO(jwallace): should pass a list of goals
  void setGoal(
    const unsigned int & mx,
    const unsigned int & my);


  Coordinates search(const Coordinates & start, const Coordinates & goal);

  void initialize(unsigned int x_size, unsigned int y_size);

private:
  inline bool isGoal(NodePtr & node);

  Graph graph_;
  NodePtr start_;
  NodePtr goal_;
  NodeQueue queue_;

  unsigned int x_size_;
  unsigned int y_size_;

  NodePtr addToGraph(const unsigned int & index);
};
}  // namespace nav2_route

#endif  // NAV2_ROUTE__BREADTH_FIRST_SEARCH_HPP_
