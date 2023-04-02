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

#include "nav2_route/breadth_first_search.hpp"
#include "nav2_route/node.hpp"

namespace nav2_route
{

BreadthFirstSearch::BreadthFirstSearch()
: start_(nullptr),
  goal_(nullptr),
  x_size_(0),
  y_size_(0)
{}


void BreadthFirstSearch::setStart(const unsigned int & mx, const unsigned int & my)
{
  start_ = addToGraph(Node::getIndex(mx, my, x_size_));
}

void BreadthFirstSearch::setGoal(const unsigned int & mx, const unsigned int & my)
{
  goal_ = addToGraph(Node::getIndex(mx, my, x_size_));
}

BreadthFirstSearch::NodePtr BreadthFirstSearch::addToGraph(const unsigned int & index)
{
  auto iter = graph_.find(index);

  if (iter != graph_.end()) {
    return &(iter->second);
  }

  return &(graph_.emplace(index, Node(index)).first->second);
}

void BreadthFirstSearch::initialize(unsigned int x_size, unsigned int y_size)
{
  x_size_ = x_size;
  y_size_ = y_size;
}

}  // namespace nav2_route
