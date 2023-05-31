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

namespace nav2_route
{

struct Coordinates
{
  Coordinates() = default;

  Coordinates(const float & x_in, const float & y_in)
  : x(x_in), y(y_in) {}

  float x, y;
};

struct Node
{
  unsigned int index;
  bool explored{false};
};

class BreadthFirstSearch
{
public:
  typedef Node * NodePtr;
  typedef std::vector<NodePtr> NodeVector;

  void setCostmap(nav2_costmap_2d::Costmap2D * costmap);

  void setStart(unsigned int mx, unsigned int my);

  void setGoals(std::vector<unsigned int> mxs, std::vector<unsigned int> mys);

  bool search(Coordinates & closest_goal);

private:
  inline Coordinates getCoords(
    const unsigned int & index) const
  {
    const unsigned int & width = x_size_;
    return {static_cast<float>(index % width), static_cast<float>(index / width)};
  }

  NodePtr addToGraph(const unsigned int index);

  bool getNeighbors(unsigned int current, NodeVector & neighbors);

  bool inCollision(unsigned int index);

  std::unordered_map<unsigned int, Node> graph_;

  NodePtr start_;
  NodeVector goals_;

  unsigned int x_size_;
  unsigned int y_size_;
  unsigned int max_index_;
  std::vector<int> neighbors_grid_offsets_;
  nav2_costmap_2d::Costmap2D * costmap_;
};
}  // namespace nav2_route

#endif  // NAV2_ROUTE__BREADTH_FIRST_SEARCH_HPP_
