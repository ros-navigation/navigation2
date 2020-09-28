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

#ifndef LAZY_THETA_STAR_P_PLANNER__LAZY_THETA_STAR_P_HPP_
#define LAZY_THETA_STAR_P_PLANNER__LAZY_THETA_STAR_P_HPP_

#include <cmath>
#include <vector>
#include <queue>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

typedef int id;
typedef double cost;
typedef int map_pts;
typedef double world_pts;
const double INF_COST = DBL_MAX;
const int LETHAL_COST = 252;

struct coordsM
{
  map_pts x, y;
};

struct coordsW
{
  world_pts x, y;
};

struct pos
{
  id pos_id;
  cost f;
};

struct tree_node
{
  map_pts x, y;
  cost g = INF_COST;
  cost h = INF_COST;
  id parent_id;
  bool is_in_queue = false;
  cost f = INF_COST;
};

struct comp
{
  bool operator()(pos & p1, pos & p2)
  {
    return (p1.f) > (p2.f);
  }
};

namespace lazyThetaStarP
{
class LazyThetaStarP
{
private:
  // for the coordinates (x,y), we store at node_position[size_x * y + x],
  // the index at which the data of the node is present in nodes_data
  std::vector<id> node_position;
  
  // the vector nodes_data stores the coordinates, costs and index of the parent node,
  // and whether or not the node is present in queue_
  std::vector<tree_node> nodes_data;
  
  // this is the priority queue to select the next node for expansion
  std::priority_queue<pos, std::vector<pos>, comp> queue_;


  // it is a counter like variable used to give out the index
  // at which data will be stored for a node that is being expanded
  id index_generated;
  
  /// CAN BE ADDED :
  /// could make it a linear array of id's, if the coordinates are also going to be
  /// denoted by an index -> size_x * y_coord + x_coord
  /// this would help to reduce the number of elements in the tree_node struct and slightly faster
  /// resetting of the values
  const coordsM moves[8] = {{0, 1},
    {0, -1},
    {1, 0},
    {-1, 0},
    {1, -1},
    {-1, 1},
    {1, 1},
    {-1, -1}};

  /** @brief it does a line of sight (los) check between the current node and the parent of its parent node
   * 		if an los is found and the new costs calculated are lesser then the cost and parent node of the current node
   * 		is updated
   * @param data of the current node
  */
  void resetParent(tree_node & curr_data);

  /**
   * @brief this function expands the neighbors of the current node
   * @param curr_data used to send the data of the current node
   * @param curr_id used to send the index of the current node as stored in nodes_position
   */
  void setNeighbors(const tree_node & curr_data, const id & curr_id);

  /**
   * @brief it returns the path by backtracing from the goal to the start, by using their parent nodes
   * @param raw_points used to return the path  thus found
   * @param curr_id sends in the index of the goal coordinate, as stored in nodes_position
   */
  void backtrace(std::vector<coordsW> & raw_points, id curr_id);

  /**
   * @brief performs the line of sight check using Bresenham's Algorithm,
   * 		and has been modified to calculate the traversal cost incurred in a straight line path between
   * 		the two points whose coordinates are (x0, y0) and (x1, y1)
   * @param sl_cost is used to return the cost thus incurred
   * @return true if a line of sight exists between the points
   */
  bool losCheck(
    const map_pts & x0, const map_pts & y0, const map_pts & x1, const map_pts & y1,
    cost & sl_cost) const;

  cost dist(const map_pts & ax, const map_pts & ay, const map_pts & bx, const map_pts & by)
  {
    return std::hypot(ax - bx, ay - by);
  }

  /**
   * @brief for the point(cx, cy) its traversal cost is calculated by <parameter>*(<actual_traversal_cost_from_costmap>)^2/(<max_cost>)^2
   * @return the traversal cost thus calculated
   */
  cost getCost(const cost & cx, const cost & cy) const
  {
    return costmap_tolerance_ *
           costmap_->getCost(cx, cy) * costmap_->getCost(cx, cy) / LETHAL_COST / LETHAL_COST;
  }

  void addIndex(const map_pts & cx, const map_pts & cy, const id & id_this)
  {
    node_position[size_x * cy + cx] = id_this;
  }

  void getIndex(const map_pts & cx, const map_pts & cy, id & id_this)
  {
    id_this = node_position[size_x * cy + cx];
  }

  bool withinLimits(const map_pts & cx, const map_pts & cy) const
  {
    return cx >= 0 && cx < size_x && cy >= 0 && cy < size_y;
  }

  bool isGoal(const map_pts & cx, const map_pts & cy) const
  {
    return cx == dst.x && cy == dst.y;
  }

  cost maxCost(const cost & a, const cost & b) const
  {
    if (a > b) {return a; } else {return b; }
  }

  void addToNodesData(const id & id_this)
  {
    if (nodes_data.size() <= static_cast<unsigned int>(id_this)) {
      nodes_data.push_back({});
    } else {
      nodes_data[id_this] = {};
    }
  }

  void clearQueue()
  {
    while (!queue_.empty()) {
      queue_.pop();
    }
  }

  void initializePosn(int size_inc = 0)
  {
    int i = 0;
    if (!node_position.empty()) {
      for (; i < size_x * size_y; i++) {
        node_position[i] = -1;
      }
    }
    for (; i < size_inc; i++) {
      node_position.emplace_back(-1);
    }
  }

  void setContainers();

public:

  coordsM src{}, dst{};
  nav2_costmap_2d::Costmap2D * costmap_{};
  nav2_util::LifecycleNode::SharedPtr node_;

  int how_many_corners_;
  int size_x, size_y;

  // parameter for cost of costmap traversal
  cost costmap_tolerance_;
  // parameter for distance function used for heurestic
  cost euc_tolerance_2_;
  // parameter for distance function used for distance function cost from the source to the current node
  cost euc_tolerance_;

  LazyThetaStarP();

  /**
   * @brief the function that iteratively searces upon the nodes in the queue (open list) until the
   * 		current node is the goal
   * @param raw_path is used to return the path obtained on exectuing the algorithm
   * @return true if a path is found, false if no path is found
   */
  bool generatePath(std::vector<coordsW> & raw_path);

  bool isSafe(const map_pts & cx, const map_pts & cy) const
  {
    return costmap_->getCost(cx, cy) < LETHAL_COST;
  }
};
}   //  namespace lazyThetaStarP

#endif  //  LAZY_THETA_STAR_P_PLANNER__LAZY_THETA_STAR_P_HPP_
