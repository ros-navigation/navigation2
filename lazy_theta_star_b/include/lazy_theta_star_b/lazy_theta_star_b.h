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

#ifndef LAZY_THETA_STAR_B__LAZY_THETA_STAR_B_H_
#define LAZY_THETA_STAR_B__LAZY_THETA_STAR_B_H_

#include <queue>
#include <algorithm>
#include <iostream>
#include <chrono>
#include <vector>
#include <cstring>
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

typedef int id;

#define INF_COST 10000000.0
#define LETHAL_COST 127

template<typename ptsT>
struct pts
{
  ptsT x, y;
};

struct pos
{
  id pos_id;
  double * g;
  double * f;
};

struct tree_node
{
  int x, y;
  double g = INF_COST;
  double h = INF_COST;
  id parentId;
  int closed = 0;             // 0 - unexplored, 1 - closed, -1 - open
  double f = INF_COST;
  int got_here = 0;
};

namespace lazyThetaStarB
{

class LazyThetaStarB
{
public:
  int lethal_cost = LETHAL_COST;

  nav2_util::LifecycleNode::SharedPtr node_;
  nav2_costmap_2d::Costmap2D * costmap_;

  // stores the cell data
  std::vector<tree_node> data;

  // stores the raw path given by the planner on the basis of their index in the std::vector data
  std::vector<id> path;

  // is the priority queue
  std::vector<pos> pq;
  int sizeX = 0;
  int sizeY = 0;
  int how_many_corners;
  pts<int> src{}, dst{};

  // The part containing the algorithm
  void makePlan(std::vector<float> & x_path, std::vector<float> & y_path);

  // The Line of Sight checking algorithm, takes in the points, and follows the line joining points
  bool losCheck(int x0, int y0, int x1, int y1);

  // to pop the minimum value of the queue, it is ever so slightly faster than the stl one,
  // based on the implementation by Peter Sanders (2000)
  void binaryHeapDelMin();

  // to compare between values in the priority queue
  static bool comp(pos p1, pos p2)
  {
    return (*(p1.f) != *(p2.f)) ? (*(p1.f) > *(p2.f)) : (*(p1.g) > *(p2.g));
  }

  double dist(int & ax, int & ay, int & bx, int & by)
  {
    return sqrt(pow(ax - bx, 2) + pow(ay - by, 2));
  }

  bool withinLimits(const int & x, const int & y);
  bool isSafe(const int & cx, const int & cy);
  void pushToPq(id & idThis);
  void clearRobotCell(int mx, int my);
  void backtrace(std::vector<id> * waypt, id curr_id);

  // stores the index at which the node data is stored for a particular co-ordinate
  std::vector<id> posn;

  // it sets/resets all the values within posn to 0
  void initializePosn();

  // functions used to maintain indices
  void addIndex(const int & cx, const int & cy, const id & index);
  id getIndex(const int & cx, const int & cy);

  bool isSafe2(int cx, int cy);
};

}  // namespace lazyThetaStarB
#endif  // LAZY_THETA_STAR_B__LAZY_THETA_STAR_B_H_
