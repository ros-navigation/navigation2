//  Copyright 2020 Anshumaan Singh
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//  http:// www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include "lazy_theta_star_p_planner/lazy_theta_star_p.hpp"
#include <vector>

namespace lazyThetaStarP
{

LazyThetaStarP::LazyThetaStarP()
{
  costmap_tolerance_ = 1.75;
  euc_tolerance_2_ = 1.0;
  euc_tolerance_ = 1.0;
  size_x = 0;
  size_y = 0;
  index_generated = 0;
}

bool LazyThetaStarP::generatePath(std::vector<coordsW> & raw_path)
{
  setContainers();
  RCLCPP_INFO(node_->get_logger(), "Path Planning Begins... ");

  index_generated = 0;
  id curr_id = index_generated;
  addToNodesData(index_generated);
  nodes_data[curr_id] = {src.x, src.y, getCost(src.x, src.y), dist(src.x, src.y, dst.x, dst.y),
    curr_id, true, dist(src.x, src.y, dst.x, dst.y) + (getCost(src.x, src.y))};
  queue_.push({curr_id, (nodes_data[curr_id].f)});
  addIndex(nodes_data[curr_id].x, nodes_data[curr_id].y, index_generated);
  index_generated++;

  int nodes_opened = 0;
  while (!queue_.empty()) {
    nodes_opened++;

    if (isGoal(nodes_data[curr_id].x, nodes_data[curr_id].y)) {
      break;
    }
    tree_node & curr_data = nodes_data[curr_id];
    resetParent(curr_data);
    setNeighbors(curr_data, curr_id);

    curr_id = queue_.top().pos_id;
    queue_.pop();
  }

  if (queue_.empty()) {
    RCLCPP_INFO(node_->get_logger(), "No Path Found !!!!!!!!!!!");
    raw_path.clear();
    return false;
  }

  std::cout << "the number of executions are: " << nodes_opened << '\n';
  RCLCPP_INFO(node_->get_logger(),
    "REACHED DEST  %i,   %i --- %f",
    nodes_data[curr_id].x,
    nodes_data[curr_id].y,
    nodes_data[curr_id].g);
  backtrace(raw_path, curr_id);
  clearQueue();
  return true;
}

void LazyThetaStarP::resetParent(tree_node & curr_data)
{
  cost g_cost, cal_cost, los_cost = 0;
  curr_data.is_in_queue = false;
  tree_node & curr_par = nodes_data[curr_data.parent_id];
  tree_node & maybe_par = nodes_data[curr_par.parent_id];

  if (losCheck(curr_data.x, curr_data.y, maybe_par.x, maybe_par.y, los_cost)) {
    g_cost = maybe_par.g +
      dist(curr_data.x, curr_data.y, maybe_par.x, maybe_par.y) * euc_tolerance_ +
      los_cost;
    cal_cost = g_cost + curr_data.h;
    if (cal_cost < curr_data.f) {
      curr_data.parent_id = curr_par.parent_id;
      curr_data.g = g_cost;
      curr_data.f = cal_cost;
    }
  }
}

void LazyThetaStarP::setNeighbors(const tree_node & curr_data, const id & curr_id)
{
  map_pts mx, my;
  id m_id, m_par;
  cost g_cost, h_cost, cal_cost;
  tree_node * curr_node;

  const tree_node & curr_par = nodes_data[curr_data.parent_id];
  for (auto & move : moves) {
    mx = curr_data.x + move.x;
    my = curr_data.y + move.y;

    if (mx == curr_par.x && my == curr_par.y) {
      continue;
    }

    if (withinLimits(mx, my)) {
      if (!isSafe(mx, my)) {
        continue;
      }
    } else {
      continue;
    }

    m_par = curr_id;
    g_cost = curr_data.g + dist(curr_data.x, curr_data.y, mx, my) * euc_tolerance_ +
      getCost(mx, my);

    getIndex(mx, my, m_id);

    if (m_id == -1) {
      addToNodesData(index_generated);
      m_id = index_generated;
      addIndex(mx, my, m_id);
      index_generated++;
    }

    curr_node = &nodes_data[m_id];

    h_cost = dist(mx, my, dst.x, dst.y) * euc_tolerance_2_;
    cal_cost = g_cost + h_cost;

    if (curr_node->f > cal_cost) {
      curr_node->g = g_cost;
      curr_node->h = h_cost;
      curr_node->f = cal_cost;
      curr_node->parent_id = m_par;
      if (!curr_node->is_in_queue) {
        curr_node->x = mx;
        curr_node->y = my;
        curr_node->is_in_queue = true;
        queue_.push({m_id, (curr_node->f)});
      }
    }
  }
}

void LazyThetaStarP::backtrace(std::vector<coordsW> & raw_points, id curr_id)
{
  std::vector<coordsW> path_rev;
  coordsW world{};
  double pusher_ = 0.5 * costmap_->getResolution();
  do {
    costmap_->mapToWorld(nodes_data[curr_id].x, nodes_data[curr_id].y, world.x, world.y);
    path_rev.push_back({world.x + pusher_, world.y + pusher_});
    curr_id = nodes_data[curr_id].parent_id;
  } while (curr_id != 0);
  costmap_->mapToWorld(nodes_data[curr_id].x, nodes_data[curr_id].y, world.x, world.y);
  path_rev.push_back({world.x + pusher_, world.y + pusher_});

  raw_points.reserve(path_rev.size());
  for (int i = static_cast<int>(path_rev.size()) - 1; i >= 0; i--) {
    raw_points.push_back(path_rev[i]);
  }
}

bool LazyThetaStarP::losCheck(
  const map_pts & x0,
  const map_pts & y0,
  const map_pts & x1,
  const map_pts & y1,
  cost & sl_cost) const
{
  sl_cost = 0;
  const int dy = abs(y1 - y0);
  const int dx = abs(x1 - x0);
  const int sx = x1 > x0 ? 1 : -1;
  const int sy = y1 > y0 ? 1 : -1;
  const int u_x = (sx - 1) / 2;
  const int u_y = (sy - 1) / 2;
  int cx, cy, f;
  cx = x0;
  cy = y0;
  f = 0;

  if (dx >= dy) {
    if (dy != 0) {
      for (; cx != x1; cx += sx) {
        f += dy;
        if (f >= dx) {
          if (!isSafe(cx + u_x, cy + u_y)) {
            return false;
          }
          sl_cost += getCost(cx + u_x, cy + u_y);
          cy += sy;
          f -= dx;
        }
		if (f != 0 && !isSafe(cx + u_x, cy + u_y)) {
		  return false;
		}
		sl_cost += getCost(cx + u_x, cy + u_y);
      }
    } else {
      for (cx = x0; cx < x1; cx += sx) {
        if (!isSafe(cx + u_x, cy) || !isSafe(cx + u_x, cy - 1)) {
          return false;
        }
        sl_cost += maxCost(getCost(cx + u_x, cy), getCost(cx + u_x, cy - 1));
      }
    }
  } else {
    if (dx != 0) {
      for (; cy != y1; cy += sy) {
        f = f + dx;
        if (f >= dy) {
          if (!isSafe(cx + u_x, cy + u_y)) {
            return false;
          }
          sl_cost += getCost(cx + u_x, cy + u_y);
          cx += sx;
          f -= dy;
        }
        if (f != 0 && !isSafe(cx + u_x, cy + u_y)) {
		  return false;
		}
        sl_cost += getCost(cx + u_x, cy + u_y);
      }
    } else {
      for (; cy != y1; cy += sy) {
        if (!isSafe(cx, cy + u_y) || !isSafe(cx - 1, cy + u_y)) {
          return false;
        }
        sl_cost += maxCost(getCost(cx, cy + u_y), getCost(cx - 1, cy + u_y));
      }
    }
  }
  return true;
}

void LazyThetaStarP::setContainers()
{
  index_generated = 0;
  int last_size_x = size_x;
  int last_size_y = size_y;
  int curr_size_x = static_cast<int>(costmap_->getSizeInCellsX());
  int curr_size_y = static_cast<int>(costmap_->getSizeInCellsY());
  if (last_size_x != curr_size_x || last_size_y != curr_size_y) {
    initializePosn(curr_size_y * curr_size_x - last_size_y * last_size_x);
    nodes_data.reserve(curr_size_x * curr_size_y);
  } else {
    initializePosn();
  }
  size_x = curr_size_x;
  size_y = curr_size_y;
}
}  //  namespace lazyThetaStarP
