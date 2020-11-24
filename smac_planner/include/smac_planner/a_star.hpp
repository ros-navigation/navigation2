// Copyright (c) 2020, Samsung Research America
// Copyright (c) 2020, Applied Electric Vehicles Pty Ltd
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
// limitations under the License. Reserved.

#ifndef SMAC_PLANNER__A_STAR_HPP_
#define SMAC_PLANNER__A_STAR_HPP_

#include <omp.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <stdexcept>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "smac_planner/constants.hpp"
#include "smac_planner/node_2d.hpp"
#include "smac_planner/node_basic.hpp"
#include "smac_planner/node_se2.hpp"
#include "smac_planner/types.hpp"
using namespace std::chrono;  // NOLINT

namespace smac_planner
{
template<typename>
struct IsNode2DInstance : public std::false_type {};

template<typename>
struct IsNodeSE2Instance : public std::false_type {};

template<typename T>
struct IsNode2DInstance<Node2D<T>>: public std::true_type {};

template<typename T>
struct IsNodeSE2Instance<NodeSE2<T>>: public std::true_type {};

inline double squaredDistance(const Eigen::Vector2d & p1, const Eigen::Vector2d & p2)
{
  const double & dx = p1[0] - p2[0];
  const double & dy = p1[1] - p2[1];
  return hypot(dx, dy);
}

/**
 * @class smac_planner::AStarAlgorithm
 * @brief An A* implementation for planning in a costmap. Templated based on the Node type.
 */
template<typename NodeT, typename GridCollisionCheckerT, typename Costmap2DT, typename FootprintT>
class AStarAlgorithm
{
public:
  typedef NodeT * NodePtr;
  typedef std::unordered_map<unsigned int, NodeT> Graph;
  typedef std::vector<NodePtr> NodeVector;
  typedef std::pair<float, NodeBasic<NodeT>> NodeElement;
  typedef typename NodeT::Coordinates Coordinates;
  typedef typename NodeT::CoordinateVector CoordinateVector;
  typedef typename NodeVector::iterator NeighborIterator;
  typedef std::function<bool (const unsigned int &, NodeT * &)> NodeGetter;

  /**
   * @struct smac_planner::NodeComparator
   * @brief Node comparison for priority queue sorting
   */
  struct NodeComparator
  {
    bool operator()(const NodeElement & a, const NodeElement & b) const
    {
      return a.first > b.first;
    }
  };

  typedef std::priority_queue<NodeElement, std::vector<NodeElement>, NodeComparator> NodeQueue;

  /**
   * @brief A constructor for smac_planner::PlannerServer
   * @param neighborhood The type of neighborhood to use for search (4 or 8 connected)
   */
  AStarAlgorithm(const MotionModel & motion_model, const SearchInfo & search_info)
  : _traverse_unknown(true),
    _max_iterations(0),
    _x_size(0),
    _y_size(0),
    _search_info(search_info),
    _goal_coordinates(Coordinates()),
    _start(nullptr),
    _goal(nullptr),
    _motion_model(motion_model),
    _collision_checker(nullptr)
  {
    _graph.reserve(100000);
  }

  /**
   * @brief A destructor for smac_planner::AStarAlgorithm
   */
  ~AStarAlgorithm() {}

  /**
   * @brief Initialization of the planner with defaults
   * @param allow_unknown Allow search in unknown space, good for navigation while mapping
   * @param max_iterations Maximum number of iterations to use while expanding search
   * @param max_on_approach_iterations Maximum number of iterations before returning a valid
   * path once within thresholds to refine path
   * comes at more compute time but smoother paths.
   */
  void initialize(
    const bool & allow_unknown, int & max_iterations, const int & max_on_approach_iterations)
  {
    _traverse_unknown = allow_unknown;
    _max_iterations = max_iterations;
    _max_on_approach_iterations = max_on_approach_iterations;
  }

  /**
   * @brief Creating path from given costmap, start, and goal
   * @param path Reference to a vector of indicies of generated path
   * @param iterations Reference to number of iterations to create plan
   * @param tolerance Reference to tolerance in costmap nodes
   * @return if plan was successful
   */
  bool createPath(CoordinateVector & path, int & iterations, const float & tolerance)
  {
    _tolerance = tolerance * NodeT::neutral_cost;
    _best_heuristic_node = {std::numeric_limits<float>::max(), 0};
    clearQueue();

    if (!areInputsValid()) {
      return false;
    }

    // 0) Add starting point to the open set
    addNode(0.0, getStart());
    getStart()->setAccumulatedCost(0.0);

    // Optimization: preallocate all variables
    NodePtr current_node = nullptr;
    NodePtr neighbor = nullptr;
    float g_cost = 0.0;
    NodeVector neighbors;
    int approach_iterations = 0;
    NeighborIterator neighbor_iterator;
    int analytic_iterations = 0;
    int closest_distance = std::numeric_limits<int>::max();

    // Given an index, return a node ptr reference if its collision-free and valid
    const unsigned int max_index = getSizeX() * getSizeY() * getSizeDim3();
    NodeGetter neighborGetter = [&, this](
      const unsigned int & index, NodePtr & neighbor_rtn) -> bool {
        if (index < 0 || index >= max_index) {
          return false;
        }

        neighbor_rtn = addToGraph(index);
        return true;
      };

    while (iterations < getMaxIterations() && !_queue.empty()) {
      // 1) Pick Nbest from O s.t. min(f(Nbest)), remove from queue
      current_node = getNextNode();

      // We allow for nodes to be queued multiple times in case
      // shorter paths result in it, but we can visit only once
      if (current_node->wasVisited()) {
        continue;
      }

      iterations++;

      // 2) Mark Nbest as visited
      current_node->visited();

      // 2.a) Use an analytic expansion (if available) to generate a path
      // to the goal.
      NodePtr result =
        tryAnalyticExpansion(current_node, neighborGetter, analytic_iterations, closest_distance);
      if (result != nullptr) {
        current_node = result;
      }

      // 3) Check if we're at the goal, backtrace if required
      if (isGoal(current_node)) {
        return backtracePath(current_node, path);
      } else if (_best_heuristic_node.first < getToleranceHeuristic()) {
        // Optimization: Let us find when in tolerance and refine within reason
        approach_iterations++;
        if (
          approach_iterations > getOnApproachMaxIterations() ||
          iterations + 1 == getMaxIterations())
        {
          NodePtr node = &_graph.at(_best_heuristic_node.second);
          return backtracePath(node, path);
        }
      }

      // 4) Expand neighbors of Nbest not visited
      neighbors.clear();
      NodeT::getNeighbors(
        current_node, neighborGetter, _collision_checker, _traverse_unknown, neighbors);

      for (neighbor_iterator = neighbors.begin(); neighbor_iterator != neighbors.end();
        ++neighbor_iterator)
      {
        neighbor = *neighbor_iterator;

        // 4.1) Compute the cost to go to this node
        g_cost = getAccumulatedCost(current_node) + getTraversalCost(current_node, neighbor);

        // 4.2) If this is a lower cost than prior, we set this as the new cost and new approach
        if (g_cost < getAccumulatedCost(neighbor)) {
          neighbor->setAccumulatedCost(g_cost);
          neighbor->parent = current_node;

          // 4.3) If not in queue or visited, add it, `getNeighbors()` handles
          neighbor->queued();
          addNode(g_cost + getHeuristicCost(neighbor), neighbor);
        }
      }
    }

    return false;
  }

  /**
   * @brief Node2D, 2D template. Create the graph based on the node type. For 2D nodes, a cost grid.
   *   For 3D nodes, a SE2 grid without cost info as needs collision detector for footprint.
   * @param x The total number of nodes in the X direction
   * @param y The total number of nodes in the X direction
   * @param dim_3 The total number of nodes in the theta or Z direction
   * @param costmap Costmap to convert into the graph
   */
  template<typename U = NodeT,
    std::enable_if_t<!IsNodeSE2Instance<U>::value, bool> = true
  >
  void createGraph(
    const unsigned int & x_size, const unsigned int & y_size, const unsigned int & dim_3_size,
    Costmap2DT * & costmap)
  {
    if (dim_3_size != 1) {
      throw std::runtime_error("Node type Node2D cannot be given non-1 dim 3 quantization.");
    }
    _costmap = costmap;
    _dim3_size = dim_3_size;    // 2D search MUST be 2D, not 3D or SE2.
    clearGraph();

    if (getSizeX() != x_size || getSizeY() != y_size) {
      _x_size = x_size;
      _y_size = y_size;
      NodeT::initNeighborhood(_x_size, _motion_model);
    }
  }

  /**
   * @brief NodeSE2, 3D template. Create the graph based on the node type. For 2D nodes, a cost grid.
   *   For 3D nodes, a SE2 grid without cost info as needs collision detector for footprint.
   * @param x The total number of nodes in the X direction
   * @param y The total number of nodes in the X direction
   * @param dim_3 The total number of nodes in the theta or Z direction
   * @param costmap Costmap to convert into the graph
   */
  template<typename U = NodeT,
    std::enable_if_t<IsNodeSE2Instance<U>::value, bool> = true
  >
  void createGraph(
    const unsigned int & x_size, const unsigned int & y_size, const unsigned int & dim_3_size,
    Costmap2DT * & costmap)
  {
    _costmap = costmap;
    _collision_checker = GridCollisionCheckerT(costmap);
    _collision_checker.setFootprint(_footprint, _is_radius_footprint);

    _dim3_size = dim_3_size;
    unsigned int index;
    clearGraph();

    if (getSizeX() != x_size || getSizeY() != y_size) {
      _x_size = x_size;
      _y_size = y_size;
      NodeT::initMotionModel(_motion_model, _x_size, _y_size, _dim3_size, _search_info);
    }
  }

  /**
   * @brief Set the goal for planning, as a node (Node2D) index
   * @param mx The node X index of the goal
   * @param my The node Y index of the goal
   * @param dim_3 The node dim_3 index of the goal
   */
  template<typename U = NodeT,
    std::enable_if_t<!IsNodeSE2Instance<U>::value, bool> = true
  >
  void setGoal(const unsigned int & mx, const unsigned int & my, const unsigned int & dim_3)
  {
    if (dim_3 != 0) {
      throw std::runtime_error("Node type Node2D cannot be given non-zero goal dim 3.");
    }

    _goal = addToGraph(NodeT::getIndex(mx, my, getSizeX()));
    _goal_coordinates = typename NodeT::Coordinates(mx, my);
  }

  /**
   * @brief Set the goal for planning, as a node (NodeSE2) index
   * @param mx The node X index of the goal
   * @param my The node Y index of the goal
   * @param dim_3 The node dim_3 index of the goal
   */
  template<typename U = NodeT,
    std::enable_if_t<IsNodeSE2Instance<U>::value, bool> = true
  >
  void setGoal(const unsigned int & mx, const unsigned int & my, const unsigned int & dim_3)
  {
    _goal = addToGraph(NodeT::getIndex(mx, my, dim_3, getSizeX(), getSizeDim3()));
    _goal_coordinates = typename NodeT::Coordinates(
      static_cast<float>(mx), static_cast<float>(my), static_cast<float>(dim_3));
    _goal->setPose(_goal_coordinates);

    NodeT::computeWavefrontHeuristic(
      _costmap, static_cast<unsigned int>(getStart()->pose.x),
      static_cast<unsigned int>(getStart()->pose.y), mx, my);
  }

  /**
   * @brief Set the starting pose for planning, as a node (Node2D) index
   * @param mx The node X index of the goal
   * @param my The node Y index of the goal
   * @param dim_3 The node dim_3 index of the goal
   */
  template<typename U = NodeT,
    std::enable_if_t<!IsNodeSE2Instance<U>::value, bool> = true
  >
  void setStart(const unsigned int & mx, const unsigned int & my, const unsigned int & dim_3)
  {
    if (dim_3 != 0) {
      throw std::runtime_error("Node type Node2D cannot be given non-zero starting dim 3.");
    }
    _start = addToGraph(NodeT::getIndex(mx, my, getSizeX()));
  }

  /**
   * @brief Set the starting pose for planning, as a node (NodeSE2) index
   * @param mx The node X index of the goal
   * @param my The node Y index of the goal
   * @param dim_3 The node dim_3 index of the goal
   */
  template<typename U = NodeT,
    std::enable_if_t<IsNodeSE2Instance<U>::value, bool> = true
  >
  void setStart(const unsigned int & mx, const unsigned int & my, const unsigned int & dim_3)
  {
    _start = addToGraph(NodeT::getIndex(mx, my, dim_3, getSizeX(), getSizeDim3()));
    _start->setPose(
      Coordinates(static_cast<float>(mx), static_cast<float>(my), static_cast<float>(dim_3)));
  }

  /**
   * @brief Set the footprint
   * @param footprint footprint of robot
   * @param use_radius Whether this footprint is a circle with radius
   */
  void setFootprint(FootprintT footprint, bool use_radius)
  {
    _footprint = footprint;
    _is_radius_footprint = use_radius;
  }

  /**
   * @brief Perform an analytic path expansion to the goal
   * @param node The node (Node2D) to start the analytic path from
   * @param getter The function object that gets valid nodes from the graph
   * @return Node pointer to goal node if successful, else return nullptr
   */
  template<typename U = NodeT,
    std::enable_if_t<!IsNodeSE2Instance<U>::value, bool> = true
  >
  NodePtr getAnalyticPath(const NodePtr & node, const NodeGetter & node_getter)
  {
    return NodePtr(nullptr);
  }

  /**
   * @brief Perform an analytic path expansion to the goal
   * @param node The node (NodeSE2) to start the analytic path from
   * @param getter The function object that gets valid nodes from the graph
   * @return Node pointer to goal node if successful, else return nullptr
   */
  template<typename U = NodeT,
    std::enable_if_t<IsNodeSE2Instance<U>::value, bool> = true
  >
  NodePtr getAnalyticPath(const NodePtr & node, const NodeGetter & node_getter)
  {
    ompl::base::ScopedState<> from(node->motion_table.state_space),
    to(node->motion_table.state_space), s(node->motion_table.state_space);
    const typename NodeT::Coordinates & node_coords = node->pose;
    from[0] = node_coords.x;
    from[1] = node_coords.y;
    from[2] = node_coords.theta * node->motion_table.bin_size;
    to[0] = _goal_coordinates.x;
    to[1] = _goal_coordinates.y;
    to[2] = _goal_coordinates.theta * node->motion_table.bin_size;

    float d = node->motion_table.state_space->distance(from(), to());
    NodePtr prev(node);
    // A move of sqrt(2) is guaranteed to be in a new cell
    static const float sqrt_2 = std::sqrt(2.);
    unsigned int num_intervals = std::floor(d / sqrt_2);

    using PossibleNode = std::pair<NodePtr, Coordinates>;
    std::vector<PossibleNode> possible_nodes;
    if (num_intervals > 1) {
      possible_nodes.reserve(num_intervals - 1);  // We won't store this node or the goal
    }
    std::vector<double> reals;
    // Pre-allocate
    unsigned int index = 0;
    NodePtr next(nullptr);
    float angle = 0.0;
    Coordinates proposed_coordinates;
    // Don't generate the first point because we are already there!
    // And the last point is the goal, so ignore it too!
    for (float i = 1; i < num_intervals; i++) {
      node->motion_table.state_space->interpolate(from(), to(), i / num_intervals, s());
      reals = s.reals();
      angle = reals[2] / node->motion_table.bin_size;
      while (angle >= node->motion_table.num_angle_quantization_float) {
        angle -= node->motion_table.num_angle_quantization_float;
      }
      while (angle < 0.0) {
        angle += node->motion_table.num_angle_quantization_float;
      }
      // Turn the pose into a node, and check if it is valid
      index = NodeT::getIndex(
        static_cast<unsigned int>(reals[0]), static_cast<unsigned int>(reals[1]),
        static_cast<unsigned int>(angle));
      // Get the node from the graph
      if (node_getter(index, next)) {
        Coordinates initial_node_coords = next->pose;
        proposed_coordinates = {static_cast<float>(reals[0]), static_cast<float>(reals[1]), angle};
        next->setPose(proposed_coordinates);
        if (next->isNodeValid(_traverse_unknown, _collision_checker) && next != prev) {
          // Save the node, and its previous coordinates in case we need to abort
          possible_nodes.emplace_back(next, initial_node_coords);
          prev = next;
        } else {
          next->setPose(initial_node_coords);
          for (const auto & node_pose : possible_nodes) {
            const auto & n = node_pose.first;
            n->setPose(node_pose.second);
          }
          return NodePtr(nullptr);
        }
      } else {
        // Abort
        for (const auto & node_pose : possible_nodes) {
          const auto & n = node_pose.first;
          n->setPose(node_pose.second);
        }
        return NodePtr(nullptr);
      }
    }
    // Legitimate path - set the parent relationships - poses already set
    prev = node;
    for (const auto & node_pose : possible_nodes) {
      const auto & n = node_pose.first;
      if (!n->wasVisited()) {
        // Make sure this node has not been visited by the regular algorithm.
        // If it has been, there is the (slight) chance that it is in the path we are expanding
        // from, so we should skip it.
        // Skipping to the next node will still create a kinematically feasible path.
        n->parent = prev;
        n->visited();
        prev = n;
      }
    }
    if (_goal != prev) {
      _goal->parent = prev;
      _goal->visited();
    }
    return _goal;
  }

  /**
   * @brief Set the starting pose for planning, as a node (Node2D) index
   * @param node Node pointer to the goal node to backtrace
   * @param path Reference to a vector of indicies of generated path
   * @return whether the path was able to be backtraced
   */
  template<typename U = NodeT,
    std::enable_if_t<!IsNodeSE2Instance<U>::value, bool> = true
  >
  bool backtracePath(NodePtr & node, CoordinateVector & path)
  {
    if (!node->parent) {
      return false;
    }

    NodePtr current_node = node;

    while (current_node->parent) {
      path.push_back(NodeT::getCoords(current_node->getIndex(), getSizeX(), getSizeDim3()));
      current_node = current_node->parent;
    }

    return path.size() > 1;
  }

  /**
   * @brief Set the starting pose for planning, as a node (NodeSE2) index
   * @param node Node pointer to the goal node to backtrace
   * @param path Reference to a vector of indicies of generated path
   * @return whether the path was able to be backtraced
   */
  template<typename U = NodeT,
    std::enable_if_t<IsNodeSE2Instance<U>::value, bool> = true
  >
  bool backtracePath(NodePtr & node, CoordinateVector & path)
  {
    if (!node->parent) {
      return false;
    }

    NodePtr current_node = node;

    while (current_node->parent) {
      path.push_back(current_node->pose);
      current_node = current_node->parent;
    }

    return path.size() > 1;
  }

  /**
   * @brief Get maximum number of iterations to plan
   * @return Reference to Maximum iterations parameter
   */
  int & getMaxIterations() {return _max_iterations;}

  /**
   * @brief Get pointer reference to starting node
   * @return Node pointer reference to starting node
   */
  NodePtr & getStart() {return _start;}

  /**
   * @brief Get pointer reference to goal node
   * @return Node pointer reference to goal node
   */
  NodePtr & getGoal() {return _goal;}

  /**
   * @brief Get maximum number of on-approach iterations after within threshold
   * @return Reference to Maximum on-appraoch iterations parameter
   */
  int & getOnApproachMaxIterations() {return _max_on_approach_iterations;}

  /**
   * @brief Get tolerance, in node nodes
   * @return Reference to tolerance parameter
   */
  float & getToleranceHeuristic() {return _tolerance;}

  /**
   * @brief Get size of graph in X
   * @return Size in X
   */
  unsigned int & getSizeX() {return _x_size;}

  /**
   * @brief Get size of graph in Y
   * @return Size in Y
   */
  unsigned int & getSizeY() {return _y_size;}

  /**
   * @brief Get number of angle quantization bins (SE2) or Z coordinate  (XYZ)
   * @return Number of angle bins / Z dimension
   */
  unsigned int & getSizeDim3() {return _dim3_size;}

protected:
  /**
   * @brief Get pointer to next goal in open set
   * @return Node (Node2D) pointer reference to next heuristically scored node
   */
  template<typename U = NodeT,
    std::enable_if_t<!IsNodeSE2Instance<U>::value, bool> = true
  >
  NodePtr getNextNode()
  {
    NodeBasic<NodeT> node = _queue.top().second;
    _queue.pop();
    return node.graph_node_ptr;
  }

  /**
   * @brief Get pointer to next goal in open set
   * @return Node (NodeSE2) pointer reference to next heuristically scored node
   */
  template<typename U = NodeT,
    std::enable_if_t<IsNodeSE2Instance<U>::value, bool> = true
  >
  NodePtr getNextNode()
  {
    NodeBasic<NodeT> node = _queue.top().second;
    _queue.pop();

    if (!node.graph_node_ptr->wasVisited()) {
      node.graph_node_ptr->pose = node.pose;
    }

    return node.graph_node_ptr;
  }

  /**
   * @brief Get pointer to next goal in open set
   * @param cost The cost to sort into the open set of the node
   * @param node Node (Node2D) pointer reference to add to open set
   */
  template<typename U = NodeT,
    std::enable_if_t<!IsNodeSE2Instance<U>::value, bool> = true
  >
  void addNode(const float cost, NodePtr & node)
  {
    NodeBasic<NodeT> queued_node(node->getIndex());
    queued_node.graph_node_ptr = node;
    _queue.emplace(cost, queued_node);
  }

  /**
   * @brief Get pointer to next goal in open set
   * @param cost The cost to sort into the open set of the node
   * @param node Node (NodeSE2) pointer reference to add to open set
   */
  template<typename U = NodeT,
    std::enable_if_t<IsNodeSE2Instance<U>::value, bool> = true
  >
  void addNode(const float cost, NodePtr & node)
  {
    NodeBasic<NodeT> queued_node(node->getIndex());
    queued_node.pose = node->pose;
    queued_node.graph_node_ptr = node;
    _queue.emplace(cost, queued_node);
  }

  /**
   * @brief Adds a node (Node2D) to the graph
   * @param cost The cost to sort into the open set of the node
   * @param node Node pointer reference to add to open set
   */
  template<typename U = NodeT,
    std::enable_if_t<!IsNodeSE2Instance<U>::value, bool> = true
  >
  NodePtr addToGraph(const unsigned int & index)
  {
    return &(_graph.emplace(index, NodeT(_costmap->getCharMap()[index], index)).first->second);
  }

  /**
   * @brief Adds a node (NodeSE2) to the graph
   * @param cost The cost to sort into the open set of the node
   * @param node Node pointer reference to add to open set
   */
  template<typename U = NodeT,
    std::enable_if_t<IsNodeSE2Instance<U>::value, bool> = true
  >
  NodePtr addToGraph(const unsigned int & index)
  {
    return &(_graph.emplace(index, NodeT(index)).first->second);
  }

  /**
   * @brief Check if this node is the goal node
   * @param node Node pointer to check if its the goal node
   * @return if node is goal
   */
  inline bool isGoal(NodePtr & node) {return node == getGoal();}

  /**
   * @brief Get cost of traversal between nodes
   * @param current_node Pointer to current node
   * @param new_node Pointer to new node
   * @return Reference traversal cost between the nodes
   */
  inline float getTraversalCost(NodePtr & current_node, NodePtr & new_node)
  {
    return current_node->getTraversalCost(new_node);
  }

  /**
   * @brief Get total cost of traversal for a node
   * @param node Pointer to current node
   * @return Reference accumulated cost between the nodes
   */
  inline float & getAccumulatedCost(NodePtr & node) {return node->getAccumulatedCost();}

  /**
   * @brief Get cost of heuristic of node
   * @param node Node index current
   * @param node Node index of new
   * @return Heuristic cost between the nodes
   */
  inline float getHeuristicCost(const NodePtr & node)
  {
    const Coordinates node_coords = NodeT::getCoords(node->getIndex(), getSizeX(), getSizeDim3());
    float heuristic = NodeT::getHeuristicCost(node_coords, _goal_coordinates);

    if (heuristic < _best_heuristic_node.first) {
      _best_heuristic_node = {heuristic, node->getIndex()};
    }

    return heuristic;
  }

  /**
   * @brief Check if inputs to planner are valid
   * @return Are valid
   */
  inline bool areInputsValid()
  {
    // Check if graph was filled in
    if (_graph.empty()) {
      throw std::runtime_error("Failed to compute path, no costmap given.");
    }

    // Check if points were filled in
    if (!_start || !_goal) {
      throw std::runtime_error("Failed to compute path, no valid start or goal given.");
    }

    // Check if ending point is valid
    if (
      getToleranceHeuristic() < 0.001 &&
      !_goal->isNodeValid(_traverse_unknown, _collision_checker))
    {
      throw std::runtime_error("Failed to compute path, goal is occupied with no tolerance.");
    }

    // Check if starting point is valid
    if (!_start->isNodeValid(_traverse_unknown, _collision_checker)) {
      throw std::runtime_error("Starting point in lethal space! Cannot create feasible plan.");
    }

    return true;
  }

  /**
   * @brief Clear hueristic queue of nodes to search
   */
  inline void clearQueue()
  {
    NodeQueue q;
    std::swap(_queue, q);
  }

  /**
   * @brief Clear graph of nodes searched
   */
  inline void clearGraph()
  {
    Graph g;
    g.reserve(100000);
    std::swap(_graph, g);
  }

  /**
   * @brief Attempt an analytic path completion
   * @return Node pointer reference to goal node if successful, else
   * return nullptr
   */
  inline NodePtr tryAnalyticExpansion(
    const NodePtr & current_node, const NodeGetter & getter, int & analytic_iterations,
    int & closest_distance)
  {
    if (_motion_model == MotionModel::DUBIN || _motion_model == MotionModel::REEDS_SHEPP) {
      // This must be a NodeSE2 node if we are using these motion models

      // See if we are closer and should be expanding more often
      const Coordinates node_coords =
        NodeT::getCoords(current_node->getIndex(), getSizeX(), getSizeDim3());
      closest_distance = std::min(
        closest_distance,
        static_cast<int>(
          NodeT::getHeuristicCost(node_coords, _goal_coordinates) / NodeT::neutral_cost));
      // We want to expand at a rate of d/expansion_ratio,
      // but check to see if we are so close that we would be expanding every iteration
      // If so, limit it to the expansion ratio (rounded up)
      int desired_iterations = std::max(
        static_cast<int>(closest_distance / _search_info.analytic_expansion_ratio),
        static_cast<int>(std::ceil(_search_info.analytic_expansion_ratio)));
      // If we are closer now, we should update the target number of iterations to go
      analytic_iterations = std::min(analytic_iterations, desired_iterations);

      // Always run the expansion on the first run in case there is a
      // trivial path to be found
      if (analytic_iterations <= 0) {
        // Reset the counter, and try the analytic path expansion
        analytic_iterations = desired_iterations;
        return getAnalyticPath(current_node, getter);
      }
      analytic_iterations--;
    }
    // No valid motion model - return nullptr
    return NodePtr(nullptr);
  }

  bool _traverse_unknown;
  int _max_iterations;
  int _max_on_approach_iterations;
  float _tolerance;
  unsigned int _x_size;
  unsigned int _y_size;
  unsigned int _dim3_size;
  SearchInfo _search_info;

  Coordinates _goal_coordinates;
  NodePtr _start;
  NodePtr _goal;

  Graph _graph;
  NodeQueue _queue;

  MotionModel _motion_model;
  NodeHeuristicPair _best_heuristic_node;

  GridCollisionCheckerT _collision_checker;
  FootprintT _footprint;
  bool _is_radius_footprint;
  Costmap2DT * _costmap;
};

}  // namespace smac_planner

#endif  // SMAC_PLANNER__A_STAR_HPP_
