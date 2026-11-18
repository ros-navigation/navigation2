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

#include "nav2_smac_planner/obstacle_heuristic.hpp"

namespace nav2_smac_planner
{

void ObstacleHeuristic::resetObstacleHeuristic(
  GridCollisionChecker * collision_checker,
  const unsigned int & start_x, const unsigned int & start_y,
  const unsigned int & goal_x, const unsigned int & goal_y,
  const bool downsample_obstacle_heuristic,
  const bool traverse_unknown)
{
  // Downsample costmap 2x to compute a sparse obstacle heuristic. This speeds up
  // the planner considerably to search through 75% less cells with no detectable
  // erosion of path quality after even modest smoothing. The error would be no more
  // than 0.05 * normalized cost. Since this is just a search prior, there's no loss in generality
  collision_checker_ = collision_checker;
  traverse_unknown_ = traverse_unknown;
  auto costmap = collision_checker_->getCostmap();

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
  const float scale = downsample_obstacle_heuristic ? 2.0f : 1.0f;

  // If already expanded, return the cost
  auto costmap = collision_checker_->getCostmap();
  unsigned int size_x = 0u;
  unsigned int size_y = 0u;
  size_x = ceil(static_cast<float>(costmap->getSizeInCellsX()) / scale);
  size_y = ceil(static_cast<float>(costmap->getSizeInCellsY()) / scale);

  // Divided by 2 due to downsampled costmap.
  unsigned int start_y, start_x;
  start_y = floor(node_coords.y / scale);
  start_x = floor(node_coords.x / scale);

  const unsigned int start_index = start_y * size_x + start_x;
  const float & requested_node_cost = obstacle_heuristic_lookup_table_[start_index];
  if (requested_node_cost > 0.0f) {
    // costs are doubled due to downsampling
    return scale * requested_node_cost;
  }

  // Return early rather than expanding the full map for a non-traversable cell (unvalidated start)
  const unsigned int scale_cells = downsample_obstacle_heuristic ? 2u : 1u;
  float cost;
  if (!isCellOpen(start_index, size_x, scale_cells, cost)) {
    return 0.0f;
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
  float c_cost, travel_cost, new_cost, existing_cost;
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
        if (!isCellOpen(new_idx, size_x, scale_cells, cost)) {
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
  return scale * requested_node_cost;
}

bool ObstacleHeuristic::isCellOpen(
  const unsigned int idx, const unsigned int size_x, const unsigned int scale,
  float & cost) const
{
  // Open if any of its costmap cells pass the collision checker's center cost validity
  const auto costmap = collision_checker_->getCostmap();
  const unsigned int costmap_size_x = costmap->getSizeInCellsX();
  const unsigned int costmap_size_y = costmap->getSizeInCellsY();
  const unsigned char * char_map = costmap->getCharMap();
  const unsigned int my = idx / size_x;
  const unsigned int x0 = (idx - my * size_x) * scale;
  const unsigned int y0 = my * scale;
  auto is_valid = [&](const float cell_cost, const unsigned int cx, const unsigned int cy) {
      return cell_cost == UNKNOWN_COST ?
             traverse_unknown_ :
             !collision_checker_->exceedsMaxCost(cell_cost, cx + 0.5f, cy + 0.5f);
    };

  // Without downsampling, the heuristic cells are the costmap cells
  if (scale == 1u) {
    if (collision_checker_->outsideRange(costmap_size_x, x0) ||
      collision_checker_->outsideRange(costmap_size_y, y0))
    {
      return false;
    }
    cost = static_cast<float>(char_map[idx]);
    return is_valid(cost, x0, y0);
  }

  // With downsampling, check if the heuristic cell's window is interior to bypass further checks
  const bool interior =
    !collision_checker_->outsideRange(costmap_size_x, x0) &&
    !collision_checker_->outsideRange(costmap_size_x, x0 + scale - 1) &&
    !collision_checker_->outsideRange(costmap_size_y, y0) &&
    !collision_checker_->outsideRange(costmap_size_y, y0 + scale - 1);
  bool is_open = false;
  cost = UNKNOWN_COST;
  for (unsigned int cy = y0; cy < y0 + scale; ++cy) {
    for (unsigned int cx = x0; cx < x0 + scale; ++cx) {
      if (!interior &&  // Skips interior checks if fully internal
        (collision_checker_->outsideRange(costmap_size_x, cx) ||
        collision_checker_->outsideRange(costmap_size_y, cy)))
      {
        continue;
      }
      const float cell_cost = static_cast<float>(char_map[cy * costmap_size_x + cx]);
      if (is_valid(cell_cost, cx, cy)) {
        is_open = true;
        cost = std::min(cost, cell_cost);
      }
    }
  }
  return is_open;
}

}  // namespace nav2_smac_planner
