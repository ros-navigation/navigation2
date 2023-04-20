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
  // Add in iterations and max time parameters
  x_size_ = x_size;
  y_size_ = y_size;
}

void BreadthFirstSearch::setCollisionChecker(nav2_route::CollisionChecker *collision_checker)
{
  collision_checker_ = collision_checker;
  unsigned int x_size = collision_checker_->getCostmap()->getSizeInCellsX();
  unsigned int y_size = collision_checker->getCostmap()->getSizeInCellsY();

  clearGraph();

  if (x_size_ != x_size || y_size_ != y_size) {
    x_size_ = x_size;
    y_size_ = y_size;
    Node::initMotionModel(static_cast<int>(x_size_), static_cast<int>(y_size_));
  }
}

bool BreadthFirstSearch::search(CoordinateVector & path)
{
  clearQueue();

  NodePtr current_node = nullptr;
  NodePtr neighbor = nullptr;
  NeighborIterator neighbor_iterator;
  NodeVector neighbors;

  const unsigned int max_index = x_size_ * y_size_;
  NodeGetter node_getter =
      [&, this](const unsigned int & index, NodePtr & neighbor_rtn) -> bool
      {
        if(index >= max_index) {
          return false;
        }

        neighbor_rtn = addToGraph(index);
        return true;
      };

  addToQueue(start_);
  while(!queue_.empty()) {
    current_node = getNextNode();

    current_node->visit();

    if(isGoal(current_node)) {
      return current_node->backtracePath(path);
    }

    neighbors.clear();
    neighbor = nullptr;

    current_node->getNeighbors(node_getter, collision_checker_, false, neighbors);

    for (neighbor_iterator = neighbors.begin();
         neighbor_iterator != neighbors.end(); ++neighbor_iterator)
    {
      neighbor = *neighbor_iterator;

      if (!neighbor->wasVisited() || !neighbor->isQueued()) {
        neighbor->parent = current_node;
        addToQueue(neighbor);
      }
    }
 }
 return false;
}

void BreadthFirstSearch::addToQueue(NodePtr &node) {
  if(node->isQueued()) {
    return;
  }

  node->queue();
  queue_.emplace(node);
}

BreadthFirstSearch::NodePtr BreadthFirstSearch::getNextNode()
{
  NodePtr next = queue_.front();
  queue_.pop();
  return next;
}

bool BreadthFirstSearch::isGoal(NodePtr &node) {
  return node == goal_;
}

void BreadthFirstSearch::clearQueue()
{
  NodeQueue q;
  std::swap(queue_, q);
}

void BreadthFirstSearch::clearGraph()
{
  Graph g;
  std::swap(graph_, g);
}

}  // namespace nav2_route
