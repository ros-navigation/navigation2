// Copyright (c) 2021, Samsung Research America
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

#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

#include <algorithm>
#include <vector>
#include <memory>

#include "nav2_smac_planner/analytic_expansion.hpp"

namespace nav2_smac_planner
{

template<typename NodeT>
AnalyticExpansion<NodeT>::AnalyticExpansion(
  const MotionModel & motion_model,
  const SearchInfo & search_info,
  const bool & traverse_unknown,
  const unsigned int & dim_3_size)
: _motion_model(motion_model),
  _search_info(search_info),
  _traverse_unknown(traverse_unknown),
  _dim_3_size(dim_3_size),
  _collision_checker(nullptr)
{
}

template<typename NodeT>
void AnalyticExpansion<NodeT>::setCollisionChecker(
  GridCollisionChecker * collision_checker)
{
  _collision_checker = collision_checker;
}

template<typename NodeT>
typename AnalyticExpansion<NodeT>::NodePtr AnalyticExpansion<NodeT>::tryAnalyticExpansion(
  const NodePtr & current_node, const NodePtr & goal_node,
  const NodeGetter & getter, int & analytic_iterations,
  int & closest_distance)
{
  // This must be a valid motion model for analytic expansion to be attempted
  if (_motion_model == MotionModel::DUBIN || _motion_model == MotionModel::REEDS_SHEPP ||
    _motion_model == MotionModel::STATE_LATTICE)
  {
    // See if we are closer and should be expanding more often
    const Coordinates node_coords =
      NodeT::getCoords(
      current_node->getIndex(), _collision_checker->getCostmap()->getSizeInCellsX(), _dim_3_size);
    closest_distance = std::min(
      closest_distance,
      static_cast<int>(NodeT::getHeuristicCost(node_coords, goal_node->pose)));

    // We want to expand at a rate of d/expansion_ratio,
    // but check to see if we are so close that we would be expanding every iteration
    // If so, limit it to the expansion ratio (rounded up)
    int desired_iterations = std::max(
      static_cast<int>(closest_distance / _search_info.analytic_expansion_ratio),
      static_cast<int>(std::ceil(_search_info.analytic_expansion_ratio)));

    // If we are closer now, we should update the target number of iterations to go
    analytic_iterations =
      std::min(analytic_iterations, desired_iterations);

    // Always run the expansion on the first run in case there is a
    // trivial path to be found
    if (analytic_iterations <= 0) {
      // Reset the counter and try the analytic path expansion
      analytic_iterations = desired_iterations;
      AnalyticExpansionNodes analytic_nodes =
        getAnalyticPath(current_node, goal_node, getter, current_node->motion_table.state_space);
      if (!analytic_nodes.empty()) {
        // If we have a valid path, attempt to refine it
        NodePtr node = current_node;
        NodePtr test_node = current_node;
        AnalyticExpansionNodes refined_analytic_nodes;
        for (int i = 0; i < 8; i++) {
          // Attempt to create better paths in 5 node increments, need to make sure
          // they exist for each in order to do so (maximum of 40 points back).
          if (test_node->parent && test_node->parent->parent && test_node->parent->parent->parent &&
            test_node->parent->parent->parent->parent &&
            test_node->parent->parent->parent->parent->parent)
          {
            test_node = test_node->parent->parent->parent->parent->parent;
            refined_analytic_nodes =
              getAnalyticPath(test_node, goal_node, getter, test_node->motion_table.state_space);
            if (refined_analytic_nodes.empty()) {
              break;
            }
            analytic_nodes = refined_analytic_nodes;
            node = test_node;
          } else {
            break;
          }
        }

        // The analytic expansion can short-cut near obstacles when closer to a goal
        // So, we can attempt to refine it more by increasing the possible radius
        // higher than the minimum turning radius and use the best solution based on
        // a scoring function similar to that used in traveral cost estimation.
        auto scoringFn = [&](const AnalyticExpansionNodes & expansion) {
            if (expansion.size() < 2) {
              return std::numeric_limits<float>::max();
            }

            float score = 0.0;
            float normalized_cost = 0.0;
            // Analytic expansions are consistently spaced
            const float distance = hypotf(
              expansion[1].proposed_coords.x - expansion[0].proposed_coords.x,
              expansion[1].proposed_coords.y - expansion[0].proposed_coords.y);
            const float & weight = expansion[0].node->motion_table.cost_penalty;
            for (auto iter = expansion.begin(); iter != expansion.end(); ++iter) {
              normalized_cost = iter->node->getCost() / 252.0f;
              // Search's Traversal Cost Function
              score += distance * (1.0 + weight * normalized_cost);
            }
            return score;
          };

        float best_score = scoringFn(analytic_nodes);
        float score = std::numeric_limits<float>::max();
        float min_turn_rad = node->motion_table.min_turning_radius;
        const float max_min_turn_rad = 4.0 * min_turn_rad;  // Up to 4x the turning radius
        while (min_turn_rad < max_min_turn_rad) {
          min_turn_rad += 0.5;  // In Grid Coords, 1/2 cell steps
          ompl::base::StateSpacePtr state_space;
          if (node->motion_table.motion_model == MotionModel::DUBIN) {
            state_space = std::make_shared<ompl::base::DubinsStateSpace>(min_turn_rad);
          } else {
            state_space = std::make_shared<ompl::base::ReedsSheppStateSpace>(min_turn_rad);
          }
          refined_analytic_nodes = getAnalyticPath(node, goal_node, getter, state_space);
          score = scoringFn(refined_analytic_nodes);
          if (score <= best_score) {
            analytic_nodes = refined_analytic_nodes;
            best_score = score;
          }
        }

        return setAnalyticPath(node, goal_node, analytic_nodes);
      }
    }

    analytic_iterations--;
  }

  // No valid motion model - return nullptr
  return NodePtr(nullptr);
}

template<typename NodeT>
typename AnalyticExpansion<NodeT>::AnalyticExpansionNodes AnalyticExpansion<NodeT>::getAnalyticPath(
  const NodePtr & node,
  const NodePtr & goal,
  const NodeGetter & node_getter,
  const ompl::base::StateSpacePtr & state_space)
{
  static ompl::base::ScopedState<> from(state_space), to(state_space), s(state_space);
  from[0] = node->pose.x;
  from[1] = node->pose.y;
  from[2] = node->motion_table.getAngleFromBin(node->pose.theta);
  to[0] = goal->pose.x;
  to[1] = goal->pose.y;
  to[2] = node->motion_table.getAngleFromBin(goal->pose.theta);

  float d = state_space->distance(from(), to());

  // A move of sqrt(2) is guaranteed to be in a new cell
  static const float sqrt_2 = sqrtf(2.0f);

  // If the length is too far, exit. This prevents unsafe shortcutting of paths
  // into higher cost areas far out from the goal itself, let search to the work of getting
  // close before the analytic expansion brings it home. This should never be smaller than
  // 4-5x the minimum turning radius being used, or planning times will begin to spike.
  if (d > _search_info.analytic_expansion_max_length || d < sqrt_2) {
    return AnalyticExpansionNodes();
  }

  unsigned int num_intervals = static_cast<unsigned int>(std::floor(d / sqrt_2));

  AnalyticExpansionNodes possible_nodes;
  // When "from" and "to" are zero or one cell away,
  // num_intervals == 0
  possible_nodes.reserve(num_intervals);  // We won't store this node or the goal
  std::vector<double> reals;
  double theta;

  // Pre-allocate
  NodePtr prev(node);
  uint64_t index = 0;
  NodePtr next(nullptr);
  float angle = 0.0;
  Coordinates proposed_coordinates;
  bool failure = false;
  std::vector<float> node_costs;
  node_costs.reserve(num_intervals);

  // Check intermediary poses (non-goal, non-start)
  for (float i = 1; i <= num_intervals; i++) {
    state_space->interpolate(from(), to(), i / num_intervals, s());
    reals = s.reals();
    // Make sure in range [0, 2PI)
    theta = (reals[2] < 0.0) ? (reals[2] + 2.0 * M_PI) : reals[2];
    theta = (theta > 2.0 * M_PI) ? (theta - 2.0 * M_PI) : theta;
    angle = node->motion_table.getAngle(theta);

    // Turn the pose into a node, and check if it is valid
    index = NodeT::getIndex(
      static_cast<unsigned int>(reals[0]),
      static_cast<unsigned int>(reals[1]),
      static_cast<unsigned int>(angle));
    // Get the node from the graph
    if (node_getter(index, next)) {
      Coordinates initial_node_coords = next->pose;
      proposed_coordinates = {static_cast<float>(reals[0]), static_cast<float>(reals[1]), angle};
      next->setPose(proposed_coordinates);
      if (next->isNodeValid(_traverse_unknown, _collision_checker) && next != prev) {
        // Save the node, and its previous coordinates in case we need to abort
        possible_nodes.emplace_back(next, initial_node_coords, proposed_coordinates);
        node_costs.emplace_back(next->getCost());
        prev = next;
      } else {
        // Abort
        next->setPose(initial_node_coords);
        failure = true;
        break;
      }
    } else {
      // Abort
      failure = true;
      break;
    }
  }

  if (!failure) {
    // We found 'a' valid expansion. Now to tell if its a quality option...
    const float max_cost = _search_info.analytic_expansion_max_cost;
    auto max_cost_it = std::max_element(node_costs.begin(), node_costs.end());
    if (max_cost_it != node_costs.end() && *max_cost_it > max_cost) {
      // If any element is above the comfortable cost limit, check edge cases:
      // (1) Check if goal is in greater than max_cost space requiring
      //  entering it, but only entering it on final approach, not in-and-out
      // (2) Checks if goal is in normal space, but enters costed space unnecessarily
      //  mid-way through, skirting obstacle or in non-globally confined space
      bool cost_exit_high_cost_region = false;
      for (auto iter = node_costs.rbegin(); iter != node_costs.rend(); ++iter) {
        const float & curr_cost = *iter;
        if (curr_cost <= max_cost) {
          cost_exit_high_cost_region = true;
        } else if (curr_cost > max_cost && cost_exit_high_cost_region) {
          failure = true;
          break;
        }
      }

      // (3) Handle exception: there may be no other option close to goal
      // if max cost is set too low (optional)
      if (failure) {
        if (d < 2.0f * M_PI * goal->motion_table.min_turning_radius &&
          _search_info.analytic_expansion_max_cost_override)
        {
          failure = false;
        }
      }
    }
  }

  // Reset to initial poses to not impact future searches
  for (const auto & node_pose : possible_nodes) {
    const auto & n = node_pose.node;
    n->setPose(node_pose.initial_coords);
  }

  if (failure) {
    return AnalyticExpansionNodes();
  }

  return possible_nodes;
}

template<typename NodeT>
typename AnalyticExpansion<NodeT>::NodePtr AnalyticExpansion<NodeT>::setAnalyticPath(
  const NodePtr & node,
  const NodePtr & goal_node,
  const AnalyticExpansionNodes & expanded_nodes)
{
  _detached_nodes.clear();
  // Legitimate final path - set the parent relationships, states, and poses
  NodePtr prev = node;
  for (const auto & node_pose : expanded_nodes) {
    auto n = node_pose.node;
    cleanNode(n);
    if (n->getIndex() != goal_node->getIndex()) {
      if (n->wasVisited()) {
        _detached_nodes.push_back(std::make_unique<NodeT>(-1));
        n = _detached_nodes.back().get();
      }
      n->parent = prev;
      n->pose = node_pose.proposed_coords;
      n->visited();
      prev = n;
    }
  }
  if (goal_node != prev) {
    goal_node->parent = prev;
    cleanNode(goal_node);
    goal_node->visited();
  }
  return goal_node;
}

template<>
void AnalyticExpansion<NodeLattice>::cleanNode(const NodePtr & node)
{
  node->setMotionPrimitive(nullptr);
}

template<typename NodeT>
void AnalyticExpansion<NodeT>::cleanNode(const NodePtr & /*expanded_nodes*/)
{
}

template<>
typename AnalyticExpansion<Node2D>::AnalyticExpansionNodes AnalyticExpansion<Node2D>::
getAnalyticPath(
  const NodePtr & node,
  const NodePtr & goal,
  const NodeGetter & node_getter,
  const ompl::base::StateSpacePtr & state_space)
{
  return AnalyticExpansionNodes();
}

template<>
typename AnalyticExpansion<Node2D>::NodePtr AnalyticExpansion<Node2D>::setAnalyticPath(
  const NodePtr & node,
  const NodePtr & goal_node,
  const AnalyticExpansionNodes & expanded_nodes)
{
  return NodePtr(nullptr);
}

template<>
typename AnalyticExpansion<Node2D>::NodePtr AnalyticExpansion<Node2D>::tryAnalyticExpansion(
  const NodePtr & current_node, const NodePtr & goal_node,
  const NodeGetter & getter, int & analytic_iterations,
  int & closest_distance)
{
  return NodePtr(nullptr);
}

template class AnalyticExpansion<Node2D>;
template class AnalyticExpansion<NodeHybrid>;
template class AnalyticExpansion<NodeLattice>;

}  // namespace nav2_smac_planner
