// Copyright (c) 2024 Nav2 Contributors
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
// limitations under the License.

#include <functional>
#include <vector>
#include <algorithm>
#include <cmath>
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_core/exceptions.hpp"
#include "nav2_dstar_lite_planner/dstar_lite.hpp"

namespace nav2_dstar_lite_planner
{

DStarLite::DStarLite(Parameters * params)
: params_(params)
{
}

void DStarLite::setStartAndGoal(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  unsigned int s[2], d[2];
  costmap_->worldToMap(start.pose.position.x, start.pose.position.y, s[0], s[1]);
  costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, d[0], d[1]);

  src_ = {static_cast<int>(s[0]), static_cast<int>(s[1])};
  dst_ = {static_cast<int>(d[0]), static_cast<int>(d[1])};
}

bool DStarLite::isUnsafeToPlan() const
{
  return !isSafe(src_) || !isSafe(dst_);
}

bool DStarLite::generatePath(
  std::vector<WorldCoord> & path,
  std::function<bool()> cancel_checker)
{
  size_x_ = static_cast<int>(costmap_->getSizeInCellsX());
  size_y_ = static_cast<int>(costmap_->getSizeInCellsY());

  if (first_run_ || needs_full_replan_) {
    initialize();
    first_run_ = false;
    needs_full_replan_ = false;
    replan_count_ = 0;
  } else if (replan_count_ >= params_->replan_interval) {
    initialize();
    replan_count_ = 0;
  } else {
    std::vector<CellIndex> changed = getChangedCells();
    for (const auto & cell : changed) {
      updateVertex(cell);
      auto preds = getPredecessors(cell);
      for (const auto & p : preds) {
        updateVertex(p);
      }
    }
    replan_count_++;
  }

  nodes_opened = 0;
  computeShortestPath(cancel_checker);

  DStarLiteState & start_state = getOrCreateState(src_);

  if (start_state.rhs >= INF_COST) {
    path.clear();
    return false;
  }

  double cost = extractPath(path);
  if (path.empty()) {
    return false;
  }
  last_path_cost_ = cost;

  snapshotCostmap();

  return true;
}

void DStarLite::initialize()
{
  resetState();

  DStarLiteState & goal_state = getOrCreateState(dst_);
  goal_state.rhs = 0.0;
  goal_state.key = calculateKey(dst_);
  goal_state.in_queue = true;

  PriorityQueueEntry entry;
  entry.index = dst_;
  entry.key = goal_state.key;
  queue_.push(entry);
}

void DStarLite::computeShortestPath(std::function<bool()> cancel_checker)
{
  Key start_key = calculateKey(src_);
  DStarLiteState & start_state = getOrCreateState(src_);

  int iter = 0;
  while (!queue_.empty() &&
         (queue_.top().key < start_key || start_state.rhs != start_state.g))
  {
    nodes_opened++;
    iter++;

    if (iter % params_->terminal_checking_interval == 0 && cancel_checker()) {
      clearQueue();
      throw nav2_core::PlannerException("D* Lite planner was canceled");
    }

    if (params_->max_iterations > 0 && iter >= params_->max_iterations) {
      clearQueue();
      throw nav2_core::PlannerException("D* Lite exceeded max iterations");
    }

    PriorityQueueEntry top_entry = queue_.top();
    queue_.pop();

    CellIndex s = top_entry.index;
    DStarLiteState & s_state = getOrCreateState(s);

    Key current_key = calculateKey(s);
    if (top_entry.key > current_key) {
      s_state.key = current_key;
      PriorityQueueEntry re_entry;
      re_entry.index = s;
      re_entry.key = current_key;
      queue_.push(re_entry);
      continue;
    }

    if (s_state.g == s_state.rhs) {
      s_state.in_queue = false;
      continue;
    }

    start_key = calculateKey(src_);

    if (s_state.g > s_state.rhs) {
      s_state.g = s_state.rhs;
      s_state.in_queue = false;

      auto preds = getPredecessors(s);
      for (const auto & p : preds) {
        updateVertex(p);
      }
    } else {
      s_state.g = INF_COST;
      s_state.in_queue = false;

      auto preds = getPredecessors(s);
      for (const auto & p : preds) {
        updateVertex(p);
      }
      updateVertex(s);
    }

    start_key = calculateKey(src_);
    start_state = getOrCreateState(src_);
  }
}

void DStarLite::updateVertex(const CellIndex & s)
{
  DStarLiteState & s_state = getOrCreateState(s);

  if (s != dst_) {
    double min_rhs = INF_COST;
    auto succs = getSuccessors(s);
    for (const auto & succ : succs) {
      DStarLiteState & succ_state = getOrCreateState(succ);
      if (succ_state.g >= INF_COST) {
        continue;
      }
      double cost = getEdgeCost(s, succ);
      if (cost >= INF_COST) {
        continue;
      }
      min_rhs = std::min(min_rhs, cost + succ_state.g);
    }
    s_state.rhs = min_rhs;
  }

  if (s_state.g != s_state.rhs) {
    Key new_key = calculateKey(s);
    s_state.key = new_key;
    s_state.in_queue = true;
    PriorityQueueEntry entry;
    entry.index = s;
    entry.key = new_key;
    queue_.push(entry);
  } else {
    s_state.in_queue = false;
  }
}

Key DStarLite::calculateKey(const CellIndex & s)
{
  DStarLiteState & s_state = getOrCreateState(s);
  double min_val = std::min(s_state.g, s_state.rhs);
  Key k;
  k.k1 = min_val + getHeuristic(s);
  k.k2 = min_val;
  return k;
}

double DStarLite::getHeuristic(const CellIndex & s) const
{
  double dx = static_cast<double>(s.x - src_.x);
  double dy = static_cast<double>(s.y - src_.y);
  return std::hypot(dx, dy);
}

double DStarLite::getEdgeCost(const CellIndex & from, const CellIndex & to) const
{
  if (!isSafe(to)) {
    return INF_COST;
  }
  double cost = getTraversalCost(to);
  double dx = static_cast<double>(from.x - to.x);
  double dy = static_cast<double>(from.y - to.y);
  double dist = std::hypot(dx, dy);
  return dist * cost;
}

double DStarLite::getTraversalCost(const CellIndex & s) const
{
  unsigned char costmap_cost = costmap_->getCost(
    static_cast<unsigned int>(s.x),
    static_cast<unsigned int>(s.y));

  if (costmap_cost >= LETHAL_OBSTACLE) {
    return INF_COST;
  }

  if (costmap_cost == 255) {
    if (params_->allow_unknown) {
      return 1.0 + 253.0 / static_cast<double>(MAX_NON_OBSTACLE);
    } else {
      return INF_COST;
    }
  }

  // Map to [1.0, 2.0]: free=1.0, max_non_obstacle=2.0
  return 1.0 + static_cast<double>(costmap_cost) / static_cast<double>(MAX_NON_OBSTACLE);
}

bool DStarLite::isSafe(const CellIndex & s) const
{
  unsigned char cost = costmap_->getCost(
    static_cast<unsigned int>(s.x),
    static_cast<unsigned int>(s.y));
  if (cost == 255) {
    return params_->allow_unknown;
  }
  return cost < LETHAL_OBSTACLE;
}

bool DStarLite::withinBounds(const CellIndex & s) const
{
  return s.x >= 0 && s.x < size_x_ && s.y >= 0 && s.y < size_y_;
}

std::vector<CellIndex> DStarLite::getSuccessors(const CellIndex & s) const
{
  std::vector<CellIndex> result;
  result.reserve(8);
  for (int i = 0; i < 8; i++) {
    CellIndex n{s.x + dx_[i], s.y + dy_[i]};
    if (withinBounds(n) && isSafe(n)) {
      result.push_back(n);
    }
  }
  return result;
}

std::vector<CellIndex> DStarLite::getPredecessors(const CellIndex & s) const
{
  return getSuccessors(s);
}

double DStarLite::extractPath(std::vector<WorldCoord> & path)
{
  path.clear();
  double total_cost = 0.0;

  CellIndex current = src_;
  const int max_steps = size_x_ * size_y_;

  for (int step = 0; step < max_steps; step++) {
    double wx, wy;
    costmap_->mapToWorld(
      static_cast<unsigned int>(current.x),
      static_cast<unsigned int>(current.y), wx, wy);
    path.push_back({wx, wy});

    if (current == dst_) {
      break;
    }

    auto succs = getSuccessors(current);
    if (succs.empty()) {
      path.clear();
      return INF_COST;
    }

    CellIndex best = succs[0];
    double best_val = INF_COST;
    for (const auto & succ : succs) {
      DStarLiteState & succ_state = getOrCreateState(succ);
      double edge_cost = getEdgeCost(current, succ);
      if (edge_cost >= INF_COST) {
        continue;
      }
      double val = edge_cost + succ_state.g;
      if (val < best_val) {
        best_val = val;
        best = succ;
      }
    }

    if (best_val >= INF_COST) {
      path.clear();
      return INF_COST;
    }

    total_cost += getEdgeCost(current, best);
    current = best;
  }

  return total_cost;
}

std::vector<CellIndex> DStarLite::getChangedCells()
{
  std::vector<CellIndex> changed;

  if (prev_costmap_.empty()) {
    changed.reserve(static_cast<std::size_t>(size_x_ * size_y_));
    for (int y = 0; y < size_y_; y++) {
      for (int x = 0; x < size_x_; x++) {
        changed.push_back({x, y});
      }
    }
    return changed;
  }

  for (int y = 0; y < size_y_; y++) {
    for (int x = 0; x < size_x_; x++) {
      unsigned int ux = static_cast<unsigned int>(x);
      unsigned int uy = static_cast<unsigned int>(y);
      unsigned char current = costmap_->getCost(ux, uy);
      std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(size_x_) +
                        static_cast<std::size_t>(x);
      if (idx >= prev_costmap_.size() || prev_costmap_[idx] != current) {
        changed.push_back({x, y});
      }
    }
  }

  return changed;
}

void DStarLite::snapshotCostmap()
{
  std::size_t cells = static_cast<std::size_t>(size_x_) *
                      static_cast<std::size_t>(size_y_);
  prev_costmap_.resize(cells);
  for (int y = 0; y < size_y_; y++) {
    for (int x = 0; x < size_x_; x++) {
      std::size_t idx = static_cast<std::size_t>(y) *
                        static_cast<std::size_t>(size_x_) +
                        static_cast<std::size_t>(x);
      prev_costmap_[idx] = costmap_->getCost(
        static_cast<unsigned int>(x),
        static_cast<unsigned int>(y));
    }
  }
}

DStarLiteState & DStarLite::getOrCreateState(const CellIndex & s)
{
  auto it = state_lookup_.find(s);
  if (it != state_lookup_.end()) {
    return state_pool_[it->second];
  }

  int id = next_state_id_++;
  if (static_cast<int>(state_pool_.size()) <= id) {
    state_pool_.resize(static_cast<std::size_t>(id) + 1);
  }
  state_pool_[id] = DStarLiteState();
  state_lookup_[s] = id;
  return state_pool_[id];
}

void DStarLite::resetState()
{
  state_pool_.clear();
  state_lookup_.clear();
  clearQueue();
  next_state_id_ = 0;
  prev_costmap_.clear();
}

void DStarLite::clearQueue()
{
  queue_ = std::priority_queue<
    PriorityQueueEntry,
    std::vector<PriorityQueueEntry>,
    PriorityQueueCompare>();
}

void DStarLite::clearStart()
{
  unsigned int mx_start = static_cast<unsigned int>(src_.x);
  unsigned int my_start = static_cast<unsigned int>(src_.y);
  costmap_->setCost(mx_start, my_start, nav2_costmap_2d::FREE_SPACE);
}

}  // namespace nav2_dstar_lite_planner
