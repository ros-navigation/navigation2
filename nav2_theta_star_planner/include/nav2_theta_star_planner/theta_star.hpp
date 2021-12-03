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

#ifndef NAV2_THETA_STAR_PLANNER__THETA_STAR_HPP_
#define NAV2_THETA_STAR_PLANNER__THETA_STAR_HPP_

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

struct tree_node
{
  int x, y;
  double g = INF_COST;
  double h = INF_COST;
  const tree_node * parent_id = nullptr;
  bool is_in_queue = false;
  double f = INF_COST;
};

struct comp
{
  bool operator()(const tree_node * p1, const tree_node * p2)
  {
    return (p1->f) > (p2->f);
  }
};

namespace theta_star
{
class ThetaStar
{
public:
  coordsM src_{}, dst_{};
  nav2_costmap_2d::Costmap2D * costmap_{};

  /// weight on the costmap traversal cost
  double w_traversal_cost_;
  /// weight on the euclidean distance cost (used for calculations of g_cost)
  double w_euc_cost_;
  /// weight on the heuristic cost (used for h_cost calculations)
  double w_heuristic_cost_;
  /// parameter to set the number of adjacent nodes to be searched on
  int how_many_corners_;
  /// the x-directional and y-directional lengths of the map respectively
  int size_x_, size_y_;

  ThetaStar();

  ~ThetaStar() = default;

  /**
   * @brief it iteratively searches upon the nodes in the queue (open list) until the
   *            current node is the goal pose or the size of queue becomes 0
   * @param raw_path is used to return the path obtained by executing the algorithm
   * @return true if a path is found, false if no path is found in between the start and goal pose
   */
  bool generatePath(std::vector<coordsW> & raw_path);

  /**
   * @brief this function checks whether the cost of a point(cx, cy) on the costmap is less than the LETHAL_COST
   * @return the result of the check
   */
  inline bool isSafe(const int & cx, const int & cy) const
  {
    return costmap_->getCost(cx, cy) < LETHAL_COST;
  }

  /**
   * @brief initialises the values of the start and goal points
   */
  void setStartAndGoal(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal);

  /**
   * @brief checks whether the start and goal points have costmap costs greater than LETHAL_COST
   * @return true if the cost of any one of the points is greater than LETHAL_COST
   */
  bool isUnsafeToPlan() const
  {
    return !(isSafe(src_.x, src_.y)) || !(isSafe(dst_.x, dst_.y));
  }

  int nodes_opened = 0;

protected:
  /// for the coordinates (x,y), it stores at node_position_[size_x_ * y + x],
  /// the pointer to the location at which the data of the node is present in nodes_data_
  /// it is initialised with size_x_ * size_y_ elements
  /// and its number of elements increases to account for a change in map size
  std::vector<tree_node *> node_position_;

  /// the vector nodes_data_ stores the coordinates, costs and index of the parent node,
  /// and whether or not the node is present in queue_, for all the nodes searched
  /// it is initialised with no elements
  /// and its size increases depending on the number of nodes searched
  std::vector<tree_node> nodes_data_;

  /// this is the priority queue (open_list) to select the next node to be expanded
  std::priority_queue<tree_node *, std::vector<tree_node *>, comp> queue_;

  /// it is a counter like variable used to generate consecutive indices
  /// such that the data for all the nodes (in open and closed lists) could be stored
  /// consecutively in nodes_data_
  int index_generated_;

  const coordsM moves[8] = {{0, 1},
    {0, -1},
    {1, 0},
    {-1, 0},
    {1, -1},
    {-1, 1},
    {1, 1},
    {-1, -1}};

  tree_node * exp_node;


  /** @brief it performs a line of sight (los) check between the current node and the parent node of its parent node;
   *            if an los is found and the new costs calculated are lesser, then the cost and parent node
   *            of the current node is updated
   * @param data of the current node
   */
  void resetParent(tree_node * curr_data);

  /**
   * @brief this function expands the current node
   * @param curr_data used to send the data of the current node
   * @param curr_id used to send the index of the current node as stored in nodes_position_
   */
  void setNeighbors(const tree_node * curr_data);

  /**
   * @brief performs the line of sight check using Bresenham's Algorithm,
   *            and has been modified to calculate the traversal cost incurred in a straight line path between
   *            the two points whose coordinates are (x0, y0) and (x1, y1)
   * @param sl_cost is used to return the cost thus incurred
   * @return true if a line of sight exists between the points
   */
  bool losCheck(
    const int & x0, const int & y0, const int & x1, const int & y1,
    double & sl_cost) const;

  /**
   * @brief it returns the path by backtracking from the goal to the start, by using their parent nodes
   * @param raw_points used to return the path  thus found
   * @param curr_id sends in the index of the goal coordinate, as stored in nodes_position
   */
  void backtrace(std::vector<coordsW> & raw_points, const tree_node * curr_n) const;

  /**
   * @brief it is an overloaded function to ease the cost calculations while performing the LOS check
   * @param cost denotes the total straight line traversal cost; it adds the traversal cost for the node (cx, cy) at every instance; it is also being returned
   * @return false if the traversal cost is greater than / equal to the LETHAL_COST and true otherwise
   */
  bool isSafe(const int & cx, const int & cy, double & cost) const
  {
    double curr_cost = getCost(cx, cy);
    if (curr_cost < LETHAL_COST) {
      cost += w_traversal_cost_ * curr_cost * curr_cost / LETHAL_COST / LETHAL_COST;
      return true;
    } else {
      return false;
    }
  }

  /*
   * @brief this function scales the costmap cost by shifting the origin to 25 and then multiply
   *           the actual costmap cost by 0.9 to keep the output in the range of [25, 255)
   */
  inline double getCost(const int & cx, const int & cy) const
  {
    return 26 + 0.9 * costmap_->getCost(cx, cy);
  }

  /**
   * @brief for the point(cx, cy), its traversal cost is calculated by
   *                    <parameter>*(<actual_traversal_cost_from_costmap>)^2/(<max_cost>)^2
   * @return the traversal cost thus calculated
   */
  inline double getTraversalCost(const int & cx, const int & cy)
  {
    double curr_cost = getCost(cx, cy);
    return w_traversal_cost_ * curr_cost * curr_cost / LETHAL_COST / LETHAL_COST;
  }

  /**
   * @brief calculates the piecewise straight line euclidean distances by
   *                    <euc_cost_parameter>*<euclidean distance between the points (ax, ay) and (bx, by)>
   * @return the distance thus calculated
   */
  inline double getEuclideanCost(const int & ax, const int & ay, const int & bx, const int & by)
  {
    return w_euc_cost_ * std::hypot(ax - bx, ay - by);
  }

  /**
   * @brief for the point(cx, cy), its heuristic cost is calculated by
   *                    <heuristic_cost_parameter>*<euclidean distance between the point and goal>
   * @return the heuristic cost
   */
  inline double getHCost(const int & cx, const int & cy)
  {
    return w_heuristic_cost_ * std::hypot(cx - dst_.x, cy - dst_.y);
  }

  /**
   * @brief checks if the given coordinates(cx, cy) lies within the map
   * @return the result of the check
   */
  inline bool withinLimits(const int & cx, const int & cy) const
  {
    return cx >= 0 && cx < size_x_ && cy >= 0 && cy < size_y_;
  }

  /**
   * @brief checks if the coordinates of a node is the goal or not
   * @return the result of the check
   */
  inline bool isGoal(const tree_node & this_node) const
  {
    return this_node.x == dst_.x && this_node.y == dst_.y;
  }

  /**
   * @brief initialises the node_position_ vector by storing -1 as index for all points(x, y) within the limits of the map
   * @param size_inc is used to increase the number of elements in node_position_ in case the size of the map increases
   */
  void initializePosn(int size_inc = 0);

  /**
   * @brief it stores id_this in node_position_ at the index [ size_x_*cy + cx ]
   * @param id_this a pointer to the location at which the data of the point(cx, cy) is stored in nodes_data_
   */
  inline void addIndex(const int & cx, const int & cy, tree_node * node_this)
  {
    node_position_[size_x_ * cy + cx] = node_this;
  }

  /**
   * @brief retrieves the pointer of the location at which the data of the point(cx, cy) is stored in nodes_data
   * @return id_this is the pointer to that location
   */
  inline tree_node * getIndex(const int & cx, const int & cy)
  {
    return node_position_[size_x_ * cy + cx];
  }

  /**
   * @brief this function depending on the size of the nodes_data_ vector allots space to store the data for a node(x, y)
   * @param id_this is the index at which the data is stored/has to be stored for that node
   */
  void addToNodesData(const int & id_this)
  {
    if (static_cast<int>(nodes_data_.size()) <= id_this) {
      nodes_data_.push_back({});
    } else {
      nodes_data_[id_this] = {};
    }
  }

  /**
   * @brief initialises the values of global variables at beginning of the execution of the generatePath function
   */
  void resetContainers();

  /**
   * @brief clears the priority queue after each execution of the generatePath function
   */
  void clearQueue()
  {
    queue_ = std::priority_queue<tree_node *, std::vector<tree_node *>, comp>();
  }
};
}   //  namespace theta_star

#endif  //  NAV2_THETA_STAR_PLANNER__THETA_STAR_HPP_
