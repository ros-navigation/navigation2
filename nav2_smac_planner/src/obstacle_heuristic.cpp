// Copyright (c) 2026, Open Navigation LLC
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

#include <algorithm>
#include <cmath>
#include <limits>

#include "nav2_smac_planner/obstacle_heuristic.hpp"

namespace nav2_smac_planner
{

namespace
{
constexpr float INC_INF = std::numeric_limits<float>::infinity();
const float INC_SQRT2 = std::sqrt(2.0f);
// 8-neighborhood: first 4 cardinal, last 4 diagonal
const int INC_DX[8] = {1, -1, 0, 0, 1, 1, -1, -1};
const int INC_DY[8] = {0, 0, 1, -1, 1, -1, 1, -1};
const bool INC_DIAG[8] = {false, false, false, false, true, true, true, true};
}  // namespace

void ObstacleHeuristic::resetObstacleHeuristic(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_i,
  const unsigned int & start_x, const unsigned int & start_y,
  const unsigned int & goal_x, const unsigned int & goal_y,
  const bool downsample_obstacle_heuristic)
{
  // Downsample costmap 2x to compute a sparse obstacle heuristic. This speeds up
  // the planner considerably to search through 75% less cells with no detectable
  // erosion of path quality after even modest smoothing. The error would be no more
  // than 0.05 * normalized cost. Since this is just a search prior, there's no loss in generality
  // Lazy path selected; getObstacleHeuristic() must not read the incremental field.
  inc_mode_ = false;
  costmap_ros = costmap_ros_i;
  auto costmap = costmap_ros->getCostmap();

  // Clear lookup table
  unsigned int size = 0u;
  unsigned int size_x = 0u;
  if (downsample_obstacle_heuristic) {
    size_x = ceil(static_cast<float>(costmap->getSizeInCellsX()) / 2.0f);
    size = size_x *
      ceil(static_cast<float>(costmap->getSizeInCellsY()) / 2.0f);
  } else {
    size_x = costmap->getSizeInCellsX();
    size = size_x * costmap->getSizeInCellsY();
  }

  if (obstacle_heuristic_lookup_table_.size() == size) {
    // must reset all values
    std::fill(
      obstacle_heuristic_lookup_table_.begin(),
      obstacle_heuristic_lookup_table_.end(), 0.0f);
  } else {
    unsigned int obstacle_size = obstacle_heuristic_lookup_table_.size();
    obstacle_heuristic_lookup_table_.resize(size, 0.0f);
    // must reset values for non-constructed indices
    std::fill_n(
      obstacle_heuristic_lookup_table_.begin(), obstacle_size, 0.0f);
  }

  obstacle_heuristic_queue_.clear();
  obstacle_heuristic_queue_.reserve(size);

  // Set initial goal point to queue from. Divided by 2 due to downsampled costmap.
  unsigned int goal_index;
  if (downsample_obstacle_heuristic) {
    goal_index = floor(goal_y / 2.0f) * size_x + floor(goal_x / 2.0f);
  } else {
    goal_index = floor(goal_y) * size_x + floor(goal_x);
  }

  obstacle_heuristic_queue_.emplace_back(
    distanceHeuristic2D(goal_index, size_x, start_x, start_y), goal_index);

  // initialize goal cell with a very small value to differentiate it from 0.0 (~uninitialized)
  // the negative value means the cell is in the open set
  obstacle_heuristic_lookup_table_[goal_index] = -0.00001f;
}

float ObstacleHeuristic::getObstacleHeuristic(
  const Coordinates & node_coords,
  const float & cost_penalty,
  const bool use_quadratic_cost_penalty,
  const bool downsample_obstacle_heuristic)
{
  // Incremental (LPA*) mode maintains the full goal-rooted field between planning
  // requests and repairs it locally; read straight from it. Unreachable cells are
  // +inf in the raw field -> return 0.0f so the node falls back to its distance
  // heuristic, exactly as the lazy path does for cells it never expands to. The
  // incremental field is always full-resolution, so the downsample flag is unused.
  if (inc_mode_) {
    const float h = getIncrementalObstacleHeuristic(node_coords);
    return std::isinf(h) ? 0.0f : h;
  }

  // If already expanded, return the cost
  auto costmap = costmap_ros->getCostmap();
  unsigned int size_x = 0u;
  unsigned int size_y = 0u;
  if (downsample_obstacle_heuristic) {
    size_x = ceil(static_cast<float>(costmap->getSizeInCellsX()) / 2.0f);
    size_y = ceil(static_cast<float>(costmap->getSizeInCellsY()) / 2.0f);
  } else {
    size_x = costmap->getSizeInCellsX();
    size_y = costmap->getSizeInCellsY();
  }

  // Divided by 2 due to downsampled costmap.
  unsigned int start_y, start_x;
  if (downsample_obstacle_heuristic) {
    start_y = floor(node_coords.y / 2.0f);
    start_x = floor(node_coords.x / 2.0f);
  } else {
    start_y = floor(node_coords.y);
    start_x = floor(node_coords.x);
  }

  const unsigned int start_index = start_y * size_x + start_x;
  const float & requested_node_cost = obstacle_heuristic_lookup_table_[start_index];
  if (requested_node_cost > 0.0f) {
    // costs are doubled due to downsampling
    return downsample_obstacle_heuristic ? 2.0f * requested_node_cost : requested_node_cost;
  }

  // If not, expand until it is included. This dynamic programming ensures that
  // we only expand the MINIMUM spanning set of the costmap per planning request.
  // Rather than naively expanding the entire (potentially massive) map for a limited
  // path, we only expand to the extent required for the furthest expansion in the
  // search-planning request that dynamically updates during search as needed.

  // start_x and start_y have changed since last call
  // we need to recompute 2D distance heuristic and reprioritize queue
  for (auto & n : obstacle_heuristic_queue_) {
    n.first = -obstacle_heuristic_lookup_table_[n.second] +
      distanceHeuristic2D(n.second, size_x, start_x, start_y);
  }
  std::make_heap(
    obstacle_heuristic_queue_.begin(), obstacle_heuristic_queue_.end(),
    ObstacleHeuristicComparator{});

  const int size_x_int = static_cast<int>(size_x);
  const float sqrt2 = sqrtf(2.0f);
  float c_cost, cost, travel_cost, new_cost, existing_cost;
  unsigned int mx, my;
  unsigned int idx, new_idx = 0;

  const std::vector<int> neighborhood = {1, -1,  // left right
    size_x_int, -size_x_int,  // up down
    size_x_int + 1, size_x_int - 1,  // upper diagonals
    -size_x_int + 1, -size_x_int - 1};  // lower diagonals

  while (!obstacle_heuristic_queue_.empty()) {
    idx = obstacle_heuristic_queue_.front().second;
    std::pop_heap(
      obstacle_heuristic_queue_.begin(), obstacle_heuristic_queue_.end(),
      ObstacleHeuristicComparator{});
    obstacle_heuristic_queue_.pop_back();
    c_cost = obstacle_heuristic_lookup_table_[idx];
    if (c_cost > 0.0f) {
      // cell has been processed and closed, no further cost improvements
      // are mathematically possible thanks to euclidean distance heuristic consistency
      continue;
    }
    c_cost = -c_cost;
    obstacle_heuristic_lookup_table_[idx] = c_cost;  // set a positive value to close the cell

    // find neighbors
    for (unsigned int i = 0; i != neighborhood.size(); i++) {
      new_idx = static_cast<unsigned int>(static_cast<int>(idx) + neighborhood[i]);

      // if neighbor path is better and non-lethal, set new cost and add to queue
      if (new_idx < size_x * size_y) {
        if (downsample_obstacle_heuristic) {
          // Get costmap values as if downsampled
          unsigned int y_offset = (new_idx / size_x) * 2;
          unsigned int x_offset = (new_idx - ((new_idx / size_x) * size_x)) * 2;
          cost = costmap->getCost(x_offset, y_offset);
          for (unsigned int k = 0; k < 2u; ++k) {
            unsigned int mxd = x_offset + k;
            if (mxd >= costmap->getSizeInCellsX()) {
              continue;
            }
            for (unsigned int j = 0; j < 2u; ++j) {
              unsigned int myd = y_offset + j;
              if (myd >= costmap->getSizeInCellsY()) {
                continue;
              }
              if (k == 0 && j == 0) {
                continue;
              }
              cost = std::min(cost, static_cast<float>(costmap->getCost(mxd, myd)));
            }
          }
        } else {
          cost = static_cast<float>(costmap->getCost(new_idx));
        }

        if (cost >= INSCRIBED_COST) {
          continue;
        }

        my = new_idx / size_x;
        mx = new_idx - (my * size_x);

        if (mx >= size_x - 3 || mx <= 3) {
          continue;
        }
        if (my >= size_y - 3 || my <= 3) {
          continue;
        }

        existing_cost = obstacle_heuristic_lookup_table_[new_idx];
        if (existing_cost <= 0.0f) {
          if (use_quadratic_cost_penalty) {
            travel_cost =
              (i <= 3 ? 1.0f : sqrt2) * (1.0f + (cost_penalty * cost * cost / 63504.0f));  // 252^2
          } else {
            travel_cost =
              ((i <= 3) ? 1.0f : sqrt2) * (1.0f + (cost_penalty * cost / 252.0f));
          }

          new_cost = c_cost + travel_cost;
          if (existing_cost == 0.0f || -existing_cost > new_cost) {
            // the negative value means the cell is in the open set
            obstacle_heuristic_lookup_table_[new_idx] = -new_cost;
            obstacle_heuristic_queue_.emplace_back(
              new_cost + distanceHeuristic2D(new_idx, size_x, start_x, start_y), new_idx);
            std::push_heap(
              obstacle_heuristic_queue_.begin(), obstacle_heuristic_queue_.end(),
              ObstacleHeuristicComparator{});
          }
        }
      }
    }

    if (idx == start_index) {
      break;
    }
  }
  return downsample_obstacle_heuristic ? 2.0f * requested_node_cost : requested_node_cost;
}

// ---------------- Incremental (LPA*) obstacle heuristic ----------------

float ObstacleHeuristic::incrementalEnterCost(
  const unsigned int idx, const bool diagonal,
  const float & cost_penalty, const bool use_quadratic_cost_penalty) const
{
  const float cost = static_cast<float>(costmap_ros->getCostmap()->getCost(idx));
  if (cost >= INSCRIBED_COST) {
    return INC_INF;
  }
  const float penalty_term = use_quadratic_cost_penalty ?
    (cost_penalty * cost * cost / 63504.0f) :   // 252^2
    (cost_penalty * cost / 252.0f);
  return (diagonal ? INC_SQRT2 : 1.0f) * (1.0f + penalty_term);
}

void ObstacleHeuristic::incrementalUpdateVertex(
  const unsigned int idx, const float & cost_penalty,
  const bool use_quadratic_cost_penalty)
{
  if (idx != inc_goal_index_) {
    float best = INC_INF;
    const int x = static_cast<int>(idx % inc_size_x_);
    const int y = static_cast<int>(idx / inc_size_x_);
    for (unsigned int i = 0; i < 8u; ++i) {
      const int nx = x + INC_DX[i];
      const int ny = y + INC_DY[i];
      if (nx < 0 || ny < 0 ||
        nx >= static_cast<int>(inc_size_x_) || ny >= static_cast<int>(inc_size_y_))
      {
        continue;
      }
      const unsigned int n = static_cast<unsigned int>(ny) * inc_size_x_ +
        static_cast<unsigned int>(nx);
      // cost to enter `idx` moving from neighbor `n`
      const float ec = incrementalEnterCost(
        idx, INC_DIAG[i], cost_penalty, use_quadratic_cost_penalty);
      if (ec == INC_INF || inc_g_[n] == INC_INF) {
        continue;
      }
      best = std::min(best, inc_g_[n] + ec);
    }
    inc_rhs_[idx] = best;
  }
  if (inc_g_[idx] != inc_rhs_[idx]) {
    inc_queue_.emplace_back(std::min(inc_g_[idx], inc_rhs_[idx]), idx);
    std::push_heap(inc_queue_.begin(), inc_queue_.end(), ObstacleHeuristicComparator{});
  }
}

void ObstacleHeuristic::incrementalComputeShortestPath(
  const float & cost_penalty, const bool use_quadratic_cost_penalty)
{
  while (!inc_queue_.empty()) {
    const float k = inc_queue_.front().first;
    const unsigned int u = static_cast<unsigned int>(inc_queue_.front().second);
    std::pop_heap(inc_queue_.begin(), inc_queue_.end(), ObstacleHeuristicComparator{});
    inc_queue_.pop_back();

    const float ku = std::min(inc_g_[u], inc_rhs_[u]);
    if (k > ku + 1e-6f) {
      continue;                          // stale queue entry (lazy deletion)
    }
    if (inc_g_[u] == inc_rhs_[u]) {
      continue;                          // already locally consistent
    }
    if (inc_g_[u] > inc_rhs_[u]) {
      inc_g_[u] = inc_rhs_[u];           // overconsistent -> relax
    } else {
      inc_g_[u] = INC_INF;               // underconsistent -> raise
      incrementalUpdateVertex(u, cost_penalty, use_quadratic_cost_penalty);
    }
    const int x = static_cast<int>(u % inc_size_x_);
    const int y = static_cast<int>(u / inc_size_x_);
    for (unsigned int i = 0; i < 8u; ++i) {
      const int nx = x + INC_DX[i];
      const int ny = y + INC_DY[i];
      if (nx < 0 || ny < 0 ||
        nx >= static_cast<int>(inc_size_x_) || ny >= static_cast<int>(inc_size_y_))
      {
        continue;
      }
      incrementalUpdateVertex(
        static_cast<unsigned int>(ny) * inc_size_x_ + static_cast<unsigned int>(nx),
        cost_penalty, use_quadratic_cost_penalty);
    }
  }
}

void ObstacleHeuristic::resetIncrementalObstacleHeuristic(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_i,
  const unsigned int & goal_x, const unsigned int & goal_y,
  const float & cost_penalty, const bool use_quadratic_cost_penalty)
{
  // Incremental path selected; getObstacleHeuristic() reads the incremental field.
  inc_mode_ = true;
  costmap_ros = costmap_ros_i;
  auto costmap = costmap_ros->getCostmap();
  inc_size_x_ = costmap->getSizeInCellsX();
  inc_size_y_ = costmap->getSizeInCellsY();
  const unsigned int size = inc_size_x_ * inc_size_y_;

  inc_g_.assign(size, INC_INF);
  inc_rhs_.assign(size, INC_INF);
  inc_queue_.clear();
  inc_queue_.reserve(size);

  inc_goal_index_ = goal_y * inc_size_x_ + goal_x;
  inc_rhs_[inc_goal_index_] = 0.0f;
  inc_queue_.emplace_back(0.0f, inc_goal_index_);
  std::push_heap(inc_queue_.begin(), inc_queue_.end(), ObstacleHeuristicComparator{});

  incrementalComputeShortestPath(cost_penalty, use_quadratic_cost_penalty);

  // snapshot the costmap so updates can diff against it
  inc_prev_cost_.resize(size);
  for (unsigned int i = 0; i < size; ++i) {
    inc_prev_cost_[i] = costmap->getCost(i);
  }
}

unsigned int ObstacleHeuristic::updateIncrementalObstacleHeuristic(
  const float & cost_penalty, const bool use_quadratic_cost_penalty)
{
  auto costmap = costmap_ros->getCostmap();
  const unsigned int size = inc_size_x_ * inc_size_y_;
  if (inc_prev_cost_.size() != size) {
    // costmap was resized underneath us; caller must reset instead
    return size;
  }

  unsigned int changed = 0;
  for (unsigned int i = 0; i < size; ++i) {
    const unsigned char c = costmap->getCost(i);
    if (c != inc_prev_cost_[i]) {
      inc_prev_cost_[i] = c;
      ++changed;
      // edges entering cell i changed -> its rhs may change
      incrementalUpdateVertex(i, cost_penalty, use_quadratic_cost_penalty);
    }
  }
  if (changed > 0u) {
    incrementalComputeShortestPath(cost_penalty, use_quadratic_cost_penalty);
  }
  return changed;
}

float ObstacleHeuristic::getIncrementalObstacleHeuristic(const Coordinates & node_coords)
{
  const unsigned int mx = static_cast<unsigned int>(node_coords.x);
  const unsigned int my = static_cast<unsigned int>(node_coords.y);
  return inc_g_[my * inc_size_x_ + mx];
}

}  // namespace nav2_smac_planner
