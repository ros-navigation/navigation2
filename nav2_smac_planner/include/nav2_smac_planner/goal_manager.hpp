// Copyright (c) 2020, Samsung Research America
// Copyright (c) 2020, Applied Electric Vehicles Pty Ltd
// Copyright (c) 2024, Stevedan Ogochukwu Omodolor Omodia
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
#include <functional>

#include "nav2_smac_planner/types.hpp"
#include "nav2_smac_planner/node_2d.hpp"
#include "nav2_smac_planner/node_hybrid.hpp"
#include "nav2_smac_planner/node_lattice.hpp"
#include "nav2_smac_planner/node_basic.hpp"
#include "nav2_smac_planner/collision_checker.hpp"


namespace nav2_smac_planner
{

/**
* @class nav2_smac_planner::GoalManager
* @brief Responsible for managing multiple variables storing information on the goal
*/
template<typename NodeT>
class GoalManager
{
public:
  typedef NodeT * NodePtr;
  typedef std::vector<NodePtr> NodeVector;
  typedef std::unordered_set<NodePtr> NodeSet;
  typedef std::vector<GoalState<NodeT>> GoalStateVector;
  typedef typename NodeT::Coordinates Coordinates;
  typedef typename NodeT::CoordinateVector CoordinateVector;

   /**
   * @brief Constructor: Initializes empty goal state. sets and coordinate lists.
   */
  GoalManager()
  : _goals_set(NodeSet()),
    _goals_state(GoalStateVector()),
    _goals_coordinate(CoordinateVector()),
    _ref_goal_coord(Coordinates())
  {
  }

  /**
   * @brief Destructor for the GoalManager
   */
  ~GoalManager() = default;

  /**
   * @brief Checks if the goals set is empty
   * @return true if the goals set is empty
   */
  bool goalsIsEmpty()
  {
    return _goals_state.empty();
  }

  /**
   * @brief Sets the internal goals' state list to the provided goals.
   * @param goals Vector of goals state to set,
   */
  void setGoalStates(
    GoalStateVector & goals_state)
  {
    clear();
    _goals_state = goals_state;
  }

  /**
   * @brief Clears all internal goal data, including goals, states, and coordinates.
   */
  void clear()
  {
    _goals_set.clear();
    _goals_state.clear();
    _goals_coordinate.clear();
  }

  /**
   * @brief Populates coarse and fine goal lists for analytic expansion.
   * @param coarse_check_goals Output list of goals for coarse search expansion.
   * @param fine_check_goals Output list of goals for fine search refinement.
   * @param coarse_search_resolution Number of fine goals per coarse goal.
   */
  void prepareGoalsForAnalyticExpansion(
    NodeVector & coarse_check_goals, NodeVector & fine_check_goals,
    int coarse_search_resolution)
  {
    for (unsigned int i = 0; i < _goals_state.size(); i++) {
      if (_goals_state[i].is_valid) {
        if (i % coarse_search_resolution == 0) {
          coarse_check_goals.push_back(_goals_state[i].goal);
        } else {
          fine_check_goals.push_back(_goals_state[i].goal);
        }
      }
    }
  }

  /**
   * @brief Filters and marks invalid goals based on collision checking and tolerance thresholds.
   *
   * Stores only valid (or tolerably infeasible) goals into internal goal sets and coordinates.
   *
   * @param tolerance Heuristic tolerance allowed for infeasible goals.
   * @param collision_checker Collision checker to validate goal positions.
   * @param traverse_unknown Flag whether traversal through unknown space is allowed.
   */
  void removeInvalidGoals(
    const float & tolerance,
    GridCollisionChecker * collision_checker,
    const bool & traverse_unknown)
  {
    for (unsigned int i = 0; i < _goals_state.size(); i++) {
      if (_goals_state[i].goal->isNodeValid(traverse_unknown, collision_checker) ||
        tolerance > 0.001)
      {
        _goals_state[i].is_valid = true;
        _goals_set.insert(_goals_state[i].goal);
        _goals_coordinate.push_back(_goals_state[i].goal->pose);
      } else {
        _goals_state[i].is_valid = false;
      }
    }
  }

  /**
   * @brief Check if a given node is part of the goal set.
   * @param node Node pointer to check.
   * @return if node matches any goal in the goal set.
   */
  inline bool isGoal(NodePtr & node)
  {
    return _goals_set.find(node) != _goals_set.end();
  }

  /**
   * @brief Get pointer reference to goals set vector
   * @return unordered_set of node pointers reference to the goals nodes
   */
  inline NodeSet & getGoalsSet()
  {
    return _goals_set;
  }

  /**
   * @brief Get pointer reference to goals state
   * @return vector of node pointers reference to the goals state
   */
  inline GoalStateVector & getGoalsState()
  {
    return _goals_state;
  }

  /**
   * @brief Get pointer reference to goals coordinates
   * @return vector of goals coordinates reference to the goals coordinates
   */
  inline CoordinateVector & getGoalsCoordinates()
  {
    return _goals_coordinate;
  }

  /**
   * @brief Set the primary goal coordinate
   * @param coord Coordinates to set as primary goal
   */
  inline void setRefGoalCoordinates(const Coordinates & coord)
  {
    _ref_goal_coord = coord;
  }

  /**
   * @brief Checks whether the primary goal coordinate has changed.
   * @param coord Coordinates to compare with the current primary goal coordinate.
   * @return true if the primary goal coordinate has changed, false otherwise.
   */
  inline bool hasGoalChanged(const Coordinates & coord)
  {
    /**
     * Note: This function checks if the goal has changed. This has to be done with
     * the coordinates not the Node pointer because the Node pointer
     * can be reused for different goals, but the coordinates will always
     * be unique for each goal.
     */
    return _ref_goal_coord != coord;
  }

protected:
  NodeSet _goals_set;
  GoalStateVector _goals_state;
  CoordinateVector _goals_coordinate;
  Coordinates _ref_goal_coord;
};

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__GOAL_MANAGER_HPP_
