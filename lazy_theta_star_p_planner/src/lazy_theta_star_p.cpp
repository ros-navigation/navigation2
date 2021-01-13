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

LazyThetaStarP::LazyThetaStarP() : costmap_tolerance_(4.0),
  euc_tolerance_(6.5),
  how_many_corners_(8),
  size_x_(0),
  size_y_(0),
  index_generated(0) {}

bool LazyThetaStarP::generatePath(std::vector<coordsW> & raw_path)
{
  setContainers();
  int curr_id = index_generated;
  addToNodesData(index_generated);
  nodes_data[curr_id] = {src.x, src.y, getTraversalCost(src.x, src.y), dist(src.x, src.y, dst.x,
      dst.y),
    curr_id, true, dist(src.x, src.y, dst.x, dst.y) + (getTraversalCost(src.x, src.y))};
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
    raw_path.clear();
    return false;
  }

  backtrace(raw_path, curr_id);
  clearQueue();

  return true;
}

void LazyThetaStarP::resetParent(tree_node & curr_data)
{
  double g_cost, los_cost = 0;
  curr_data.is_in_queue = false;
  tree_node & curr_par = nodes_data[curr_data.parent_id];
  tree_node & maybe_par = nodes_data[curr_par.parent_id];

  if (losCheck(curr_data.x, curr_data.y, maybe_par.x, maybe_par.y, los_cost)) {
    g_cost = maybe_par.g +
      getEucledianCost(curr_data.x, curr_data.y, maybe_par.x, maybe_par.y) +
      los_cost;

    if (g_cost < curr_data.g) {
      curr_data.parent_id = curr_par.parent_id;
      curr_data.g = g_cost;
      curr_data.f = g_cost + curr_data.h;
    }
  }
}

void LazyThetaStarP::setNeighbors(const tree_node & curr_data, const int & curr_id)
{
  int mx, my;
  int m_id, m_par;
  double g_cost, h_cost, cal_cost;

  for (int i = 0; i < how_many_corners_; i++) {
    mx = curr_data.x + moves[i].x;
    my = curr_data.y + moves[i].y;

    if (withinLimits(mx, my)) {
      if (!isSafe(mx, my)) {
        continue;
      }
    } else {
      continue;
    }

    m_par = curr_id;
    g_cost = curr_data.g + getEucledianCost(curr_data.x, curr_data.y, mx, my) +
      getTraversalCost(mx, my);

    getIndex(mx, my, m_id);

    if (m_id == -1) {
      addToNodesData(index_generated);
      m_id = index_generated;
      addIndex(mx, my, m_id);
      index_generated++;
    }

    curr_node = &nodes_data[m_id];

    h_cost = getEucledianCost(mx, my, dst.x, dst.y);
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

void LazyThetaStarP::backtrace(std::vector<coordsW> & raw_points, int curr_id)
{
  std::vector<coordsW> path_rev;
  coordsW world{};
  double pusher_ = 0.5 * costmap_->getResolution();
  do {
    costmap_->mapToWorld(nodes_data[curr_id].x, nodes_data[curr_id].y, world.x, world.y);
    path_rev.push_back({world.x + pusher_, world.y + pusher_});
    if (path_rev.size() > 1) {
      curr_id = nodes_data[curr_id].parent_id;
    }
  } while (curr_id != 0);
  costmap_->mapToWorld(nodes_data[curr_id].x, nodes_data[curr_id].y, world.x, world.y);
  path_rev.push_back({world.x + pusher_, world.y + pusher_});

  raw_points.reserve(path_rev.size());
  for (int i = static_cast<int>(path_rev.size()) - 1; i >= 0; i--) {
    raw_points.push_back(path_rev[i]);
  }
}

bool LazyThetaStarP::losCheck(
  const int & x0,
  const int & y0,
  const int & x1,
  const int & y1,
  double & sl_cost)
{
  sl_cost = 0;

  int cx, cy;
  int dy = abs(y1 - y0), dx = abs(x1 - x0), f = 0;
  int sx, sy;
  int lx, ly;  // this variable is reserved for future use
  sy = y1 > y0 ? 1 : -1;
  sx = x1 > x0 ? 1 : -1;

  int u_x = (sx - 1) / 2;
  int u_y = (sy - 1) / 2;
  lx = x1;
  ly = y1;
  cx = x0;
  cy = y0;

  if (dx >= dy) {
    while (cx != lx) {
      f += dy;
      if (f >= dx) {
        if (!isSafe(cx + u_x, cy + u_y, sl_cost)) {
          return false;
        }
        cy += sy;
        f -= dx;
      }
      if (f != 0 && !isSafe(cx + u_x, cy + u_y, sl_cost)) {
        return false;
      }
      if (dy == 0 && !isSafe(cx + u_x, cy, sl_cost) && !isSafe(cx + u_x, cy - 1, sl_cost)) {
        return false;
      }
      cx += sx;
    }
  } else {
    while (cy != ly) {
      f = f + dx;
      if (f >= dy) {
        if (!isSafe(cx + u_x, cy + u_y, sl_cost)) {
          return false;
        }
        cx += sx;
        f -= dy;
      }
      if (f != 0 && !isSafe(cx + u_x, cy + u_y, sl_cost)) {
        return false;
      }
      if (dx == 0 && isSafe(cx, cy + u_y, sl_cost) && !isSafe(cx - 1, cy + u_y, sl_cost)) {
        return false;
      }
      cy += sy;
    }
  }
  return true;
}

void LazyThetaStarP::setContainers()
{
  index_generated = 0;
  int last_size_x = size_x_;
  int last_size_y = size_y_;
  int curr_size_x = static_cast<int>(costmap_->getSizeInCellsX());
  int curr_size_y = static_cast<int>(costmap_->getSizeInCellsY());
  if (last_size_x != curr_size_x || last_size_y != curr_size_y) {
    initializePosn(curr_size_y * curr_size_x - last_size_y * last_size_x);
    nodes_data.reserve(curr_size_x * curr_size_y);
  } else {
    initializePosn();
  }
  size_x_ = curr_size_x;
  size_y_ = curr_size_y;
}

void LazyThetaStarP::initializePosn(int size_inc)
{
  int i = 0;

  if (!node_position.empty()) {
    for (; i < size_x_ * size_y_; i++) {
      node_position[i] = -1;
    }
  }

  for (; i < size_inc; i++) {
    node_position.emplace_back(-1);
  }
}

}  //  namespace lazyThetaStarP
