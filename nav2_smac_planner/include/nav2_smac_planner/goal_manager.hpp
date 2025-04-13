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

#ifndef NAV2_SMAC_PLANNER__GOAL_MANAGER_HPP_
#define NAV2_SMAC_PLANNER__GOAL_MANAGER_HPP_

#include <unordered_set>
#include <vector>

#include "nav2_smac_planner/types.hpp"
#include "nav2_smac_planner/node_2d.hpp"
#include "nav2_smac_planner/node_hybrid.hpp"
#include "nav2_smac_planner/node_lattice.hpp"
#include "nav2_smac_planner/node_basic.hpp"

namespace nav2_smac_planner
{

/**
* @class nav2_smac_planner::GoalManager
* @brief Responsible for managing multiple varibales storing information on the goal
*/
template<typename NodeT>
class GoalManager{
public:
  typedef NodeT * NodePtr;
  typedef std::vector<NodePtr> NodeVector;
  typedef std::unordered_set<NodePtr> NodeSet;
  typedef std::vector<GoalState<NodeT>> GoalStateVector;
  typedef typename NodeT::Coordinates Coordinates;
  typedef typename NodeT::CoordinateVector CoordinateVector;

   /**
   * @brief Constructor for the GoalManager
   */
  GoalManager();

  /**
   * @brief Destructor for the GoalManager
   */
  ~GoalManager();

  /**
   * @brief Checks if the goals set is empty
   * @return true if the goals set is empty
   */
  bool goalsEmpty();

  /**
   * @brief Populates the goals set with the given goals and coordinates
   * @param goals Vector of goals to populate
   * @param goals_coordinates Vector of coordinates to populate
   */

  void populate(
    NodeVector & goals,
    CoordinateVector & goals_coordinates);

  /**
   * @brief Clears all goal-related data.
   */
  void clear();

  /**
   * @brief Creates the coarse and fine lists of goals to expand
   * @param coarse_list List of goals to expand
   * @param fine_list List of goals to refine
   */
  void prepareGoalsForExpansion(
    NodeVector & coarse_list, NodeVector & fine_list,
    int coarse_search_resolution);

  /**
   * @brief Remove invalid goals from the goal set, goal coordinates, and goal state.
   *        In addition, it sets the all_nodes_invalid flag to true if all goals are invalid.
   * @param isValidFn Function to check if a node is valid
   * @param all_nodes_invalid Returns true if all nodes are invalid.
   */

  void removeInvalidGoals(
    const std::function<bool(const NodePtr &)> & isValidFn,
    bool & all_nodes_invalid);

  /**
   * @brief Check if this node is the goal node
   * @param node Node pointer to check if its the goal node
   * @return if node is goal
   */
  bool isGoal(NodePtr & node);

  /**
   * @brief Get pointer reference to goals node
   * @return unordered_set of node pointers reference to the goals nodes
   */
  NodeSet & getGoals();

  /**
   * @brief Get pointer reference to goals state
   * @return vector of node pointers reference to the goals state
   */
  GoalStateVector & getGoalsState();

  /**
   * @brief Get pointer reference to goals coordinates
   * @return vector of goals coordinates reference to the goals coordinates
   */
  CoordinateVector & getGoalsCoordinates();

protected:
  NodeSet _goals_set;
  GoalStateVector _goals_state;
  CoordinateVector _goals_coordinate;
};

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__GOAL_MANAGER_HPP_
