// Copyright 2020 Anshumaan Singh
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef LAZY_THETA_STAR_B__LAZY_THETA_STAR_H_
#define LAZY_THETA_STAR_B__LAZY_THETA_STAR_H_

#include <cmath>
#include <vector>
#include <queue>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

typedef int id;
typedef double cost;
typedef int map_pts;
typedef double world_pts;

#define INF_COST DBL_MAX
#define LETHAL_COST 252

template<typename coordsT>
struct coords
{
  coordsT x, y;
};

struct pos
{
  id pos_id;
  cost * f;
};

struct tree_node
{
  map_pts x, y;
  cost g = INF_COST;
  cost h = INF_COST;
  id parent_id;
  int closed = 0;
  cost f = INF_COST;
};

struct comp
{
  bool operator()(pos & p1, pos & p2)
  {
    return *(p1.f) > *(p2.f);
  }
};
namespace lazyThetaStar
{
class LazyThetaStar
{
public:
  nav2_costmap_2d::Costmap2D * costmap_;
  nav2_util::LifecycleNode::SharedPtr node_;

  std::vector<id> posn;
  std::vector<tree_node> data;
  std::priority_queue<pos, std::vector<pos>, comp> pq;
  int sizeX, sizeY;
  int how_many_corners;
  id id_gen;
  coords<map_pts> src{}, dst{};
  int lethal_cost = LETHAL_COST;

  const map_pts moves[8][2] = {{0, 1},
    {0, -1},
    {1, 0},
    {-1, 0},
    {1, -1},
    {-1, 1},
    {1, 1},
    {-1, -1}};

  LazyThetaStar();

  bool getPath(std::vector<coords<world_pts>> & raw_path);

  bool losCheck(map_pts & x0, map_pts & y0, map_pts & x1, map_pts & y1) const;

  cost dist(map_pts & ax, map_pts & ay, map_pts & bx, map_pts & by)
  {
    return sqrt(pow(ax - bx, 2) + pow(ay - by, 2));
  }

  // TODO (Anshu-man567) : FIND A WAY TO ADD REFERENCES TO THIS FUNCTION THAT IS IF POSSIBLE
  bool isSafe(map_pts cx, map_pts cy) const
  {
    return costmap_->getCost(cx, cy) < lethal_cost;
  }

  bool withinLimits(map_pts & cx, map_pts & cy) const
  {
    return cx >= 0 && cx < sizeX && cy >= 0 && cy < sizeY;
  }

  void addIndex(map_pts cx, map_pts cy, id id_this)
  {
    posn[sizeX * cy + cx] = id_this;
  }

  void getIndex(map_pts & cx, map_pts & cy, id & id_this)
  {
    id_this = posn[sizeX * cy + cx];
  }
  void initializePosn();

  void backtrace(std::vector<coords<world_pts>> & raw_points, id & curr_id);

  bool isGoal(map_pts & cx, map_pts & cy);

  void initializeStuff();

  void clearStuff();

  void resetParent(tree_node & curr_data);

  void setNeighbors(tree_node & curr_data);
};
}   // namespace lazyThetaStar

#endif   // LAZY_THETA_STAR_B_LAZY_THETA_STAR_H_
