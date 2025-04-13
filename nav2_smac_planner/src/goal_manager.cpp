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

#include "nav2_smac_planner/goal_manager.hpp"

namespace nav2_smac_planner
{
template<typename NodeT>
GoalManager<NodeT>::GoalManager()
: _goals_set(NodeSet()),
  _goals_state(GoalStateVector()),
  _goals_coordinate(CoordinateVector())
{
}

template<typename NodeT>
GoalManager<NodeT>::~GoalManager()
{
}

template<typename NodeT>
bool GoalManager<NodeT>::goalsEmpty()
{
  return _goals_set.empty();
}

template<typename NodeT>
void GoalManager<NodeT>::populate(
  NodeVector & goals,
  CoordinateVector & goals_coordinates)
{
  _goals_coordinate.clear();
  _goals_set.clear();
  _goals_state.clear();
  for (unsigned int i = 0; i < goals.size(); i++) {
    goals[i]->setPose(goals_coordinates[i]);
    _goals_state.push_back({goals[i], true});
  }
}

template<typename NodeT>
void GoalManager<NodeT>::clear()
{
  _goals_set.clear();
  _goals_state.clear();
  _goals_coordinate.clear();
}

template<typename NodeT>
void GoalManager<NodeT>::prepareGoalsForExpansion(
  NodeVector & coarse_list, NodeVector & fine_list,
  int coarse_search_resolution)
{
  for (unsigned int i = 0; i < _goals_state.size(); i++) {
    if (_goals_state[i].is_valid) {
      if (i % coarse_search_resolution == 0) {
        coarse_list.push_back(_goals_state[i].goal);
      } else {
        fine_list.push_back(_goals_state[i].goal);
      }
    }
  }
}

template<typename NodeT>
void GoalManager<NodeT>::removeInvalidGoals(
  const std::function<bool(const NodePtr &)> & isValidFn,
  bool & all_nodes_invalid)
{
  all_nodes_invalid = true;

  for (unsigned int i = 0; i < _goals_state.size(); i++) {
    if (!isValidFn(_goals_state[i].goal)) {
      _goals_state[i].is_valid = false;
    } else {
      _goals_state[i].is_valid = true;
      _goals_set.insert(_goals_state[i].goal);
      _goals_coordinate.push_back(_goals_state[i].goal->pose);
    }
  }
}

template<typename NodeT>
bool GoalManager<NodeT>::isGoal(NodePtr & node)
{
  return _goals_set.find(node) != _goals_set.end();
}

template<typename NodeT>
typename GoalManager<NodeT>::NodeSet & GoalManager<NodeT>::getGoals()
{
  return _goals_set;
}

template<typename NodeT>
typename GoalManager<NodeT>::GoalStateVector & GoalManager<NodeT>::getGoalsState()
{
  return _goals_state;
}

template<typename NodeT>
typename GoalManager<NodeT>::CoordinateVector & GoalManager<NodeT>::getGoalsCoordinates()
{
  return _goals_coordinate;
}

template class GoalManager<Node2D>;
template class GoalManager<NodeHybrid>;
template class GoalManager<NodeLattice>;


}  // namespace nav2_smac_planner
