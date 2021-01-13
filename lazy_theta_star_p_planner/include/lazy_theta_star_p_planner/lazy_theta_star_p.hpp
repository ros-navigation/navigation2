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
#include <chrono>
#include <vector>
#include <queue>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

const double INF_COST = DBL_MAX;
const int LETHAL_COST = 252;

struct coordsM
{
  int x, y;
};

struct coordsW
{
  double x, y;
};

struct pos
{
  int pos_id;
  double f;
};

struct tree_node
{
  int x, y;
  double g = INF_COST;
  double h = INF_COST;
  int parent_id;
  bool is_in_queue = false;
  double f = INF_COST;
};

struct comp
{
  bool operator()(pos & p1, pos & p2)
  {
    return (p1.f) > (p2.f);
  }
};

/// TODO(ANSHU-MAN567): try using a pointer to the tree_node vector instead of directly using it

namespace lazyThetaStarP
{
class LazyThetaStarP
{
public:
  coordsM src{}, dst{};
  nav2_costmap_2d::Costmap2D * costmap_{};
  nav2_util::LifecycleNode::SharedPtr node_;

  /// weight for the costmap traversal cost
  double costmap_tolerance_;
  /// weight for the euclidean distance cost (as of now used for calculation of gCost and hCost)
  double euc_tolerance_;

  int how_many_corners_;
  int size_x_, size_y_;

  LazyThetaStarP();

  /**
   * @brief the function that iteratively searches upon the nodes in the queue (open list) until the
   *            current node is the goal pose or if size of queue > 0
   * @param raw_path is used to return the path obtained on exectuing the algorithm
   * @return true if a path is found, false if no path is found between the start and goal pose
   */
  bool generatePath(std::vector<coordsW> & raw_path);

  bool isSafe(const int & cx, const int & cy) const
  {
    return costmap_->getCost(cx + 1, cy + 1) < LETHAL_COST;
  }

private:
  /// for the coordinates (x,y), we store at node_position[size_x_ * y + x],
  /// the index at which the data of the node is present in nodes_data
  std::vector<int> node_position;

  /// the vector nodes_data stores the coordinates, costs and index of the parent node,
  /// and whether or not the node is present in queue_
  std::vector<tree_node> nodes_data;

  /// this is the priority queue (open_list) to select the next node for expansion
  std::priority_queue<pos, std::vector<pos>, comp> queue_;

  /// it is a counter like variable used to generate consecutive indices
  /// such that the data for all the nodes (in open and closed lists) could be stored
  /// consecutively in nodes_data
  int index_generated;

  const coordsM moves[8] = {{0, 1},
    {0, -1},
    {1, 0},
    {-1, 0},
    {1, -1},
    {-1, 1},
    {1, 1},
    {-1, -1}};

  tree_node * curr_node = new tree_node;
  /** @brief it does a line of sight (los) check between the current node and the parent of its parent node
   *            if an los is found and the new costs calculated are lesser then the cost and parent node of the current node
   *            is updated
   * @param data of the current node
  */
  void resetParent(tree_node & curr_data);

  /**
   * @brief this function expands the neighbors of the current node
   * @param curr_data used to send the data of the current node
   * @param curr_id used to send the index of the current node as stored in nodes_position
   */
  void setNeighbors(const tree_node & curr_data, const int & curr_int);

  /**
   * @brief it returns the path by backtracing from the goal to the start, by using their parent nodes
   * @param raw_points used to return the path  thus found
   * @param curr_id sends in the index of the goal coordinate, as stored in nodes_position
   */
  void backtrace(std::vector<coordsW> & raw_points, int curr_id);

  /**
   * @brief performs the line of sight check using Bresenham's Algorithm,
   *            and has been modified to calculate the traversal cost incurred in a straight line path between
   *            the two points whose coordinates are (x0, y0) and (x1, y1)
   * @param sl_cost is used to return the cost thus incurred
   * @return true if a line of sight exists between the points
   */
  bool losCheck(
    const int & x0, const int & y0, const int & x1, const int & y1,
    double & sl_cost);

  void initializePosn(int size_inc = 0);

  void setContainers();

  double dist(const int & ax, const int & ay, const int & bx, const int & by)
  {
    return std::hypot(ax - bx, ay - by);
  }

  double getEucledianCost(const int & ax, const int & ay, const int & bx, const int & by)
  {
    return euc_tolerance_ * dist(ax, ay, bx, by);
  }

  double getCellCost(const int & cx, const int & cy) const
  {
    return costmap_->getCost(cx + 1, cy + 1);
  }

  /**
   * @brief for the point(cx, cy) its traversal cost is calculated by <parameter>*(<actual_traversal_cost_from_costmap>)^2/(<max_cost>)^2
   * @return the traversal cost thus calculated
   */
  double getTraversalCost(const int & cx, const int & cy)
  {
    double curr_cost = getCellCost(cx, cy);
    return costmap_tolerance_ * curr_cost * curr_cost / LETHAL_COST / LETHAL_COST;
  }
  /**
   * @brief it is an overloaded function to ease in cost calculations while performing the LOS check
   * @param cx
   * @param cy
   * @param cost denotes the total straight line traversal cost, adds the traversal cost for the node (cx, cy) at every instance
   */
  bool isSafe(const int & cx, const int & cy, double & cost) const
  {
    double curr_cost = getCellCost(cx, cy);
    if (curr_cost < LETHAL_COST) {
      cost += costmap_tolerance_ * curr_cost * curr_cost / LETHAL_COST / LETHAL_COST;
      return true;
    } else {
      return false;
    }
  }

  void addIndex(const int & cx, const int & cy, const int & id_this)
  {
    node_position[size_x_ * cy + cx] = id_this;
  }

  void getIndex(const int & cx, const int & cy, int & id_this)
  {
    id_this = node_position[size_x_ * cy + cx];
  }

  bool withinLimits(const int & cx, const int & cy) const
  {
    return cx >= 0 && cx < size_x_ && cy >= 0 && cy < size_y_;
  }

  bool isGoal(const int & cx, const int & cy) const
  {
    return cx == dst.x && cy == dst.y;
  }

  void addToNodesData(const int & id_this)
  {
    if (nodes_data.size() <= static_cast<unsigned int>(id_this)) {
      nodes_data.push_back({});
    } else {
      nodes_data[id_this] = {};
    }
  }

  void clearQueue()
  {
    while (!queue_.empty())
    {queue_.pop();}
  }
};
}   //  namespace lazyThetaStarP

#endif  //  LAZY_THETA_STAR_P_PLANNER__LAZY_THETA_STAR_P_HPP_
