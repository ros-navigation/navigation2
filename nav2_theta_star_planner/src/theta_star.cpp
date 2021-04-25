//  Copyright 2020 Anshumaan Singh
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include <vector>
#include "nav2_theta_star_planner/theta_star.hpp"

namespace theta_star
{

ThetaStar::ThetaStar()
: w_traversal_cost_(1.0),
  w_euc_cost_(5.0),
  w_heuristic_cost_(1.0),
  how_many_corners_(8),
  size_x_(0),
  size_y_(0),
  index_generated_(0)
{
  curr_node = new tree_node;
}

void ThetaStar::setStartAndGoal(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  unsigned int s[2], d[2];
  costmap_->worldToMap(start.pose.position.x, start.pose.position.y, s[0], s[1]);
  costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, d[0], d[1]);

  src_ = {static_cast<int>(s[0]), static_cast<int>(s[1])};
  dst_ = {static_cast<int>(d[0]), static_cast<int>(d[1])};
}

bool ThetaStar::generatePath(std::vector<coordsW> & raw_path)
{
  resetContainers();
  int curr_id = index_generated_;
  addToNodesData(index_generated_);
  double src_g_cost = getTraversalCost(src_.x, src_.y), src_h_cost = getHCost(src_.x, src_.y);
  nodes_data_[curr_id] =
  {src_.x, src_.y, src_g_cost, src_h_cost, curr_id, true, src_g_cost + src_h_cost};
  queue_.push({curr_id, (nodes_data_[curr_id].f)});
  addIndex(nodes_data_[curr_id].x, nodes_data_[curr_id].y, index_generated_);
  index_generated_++;

  int nodes_opened = 0;

  while (!queue_.empty()) {
    nodes_opened++;

    tree_node & curr_data = nodes_data_[curr_id];
    if (isGoal(curr_data)) {
      break;
    }

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

void ThetaStar::resetParent(tree_node & curr_data)
{
  double g_cost, los_cost = 0;
  curr_data.is_in_queue = false;
  tree_node & curr_par = nodes_data_[curr_data.parent_id];
  tree_node & maybe_par = nodes_data_[curr_par.parent_id];

  if (losCheck(curr_data.x, curr_data.y, maybe_par.x, maybe_par.y, los_cost)) {
    g_cost = maybe_par.g +
      getEuclideanCost(curr_data.x, curr_data.y, maybe_par.x, maybe_par.y) + los_cost;

    if (g_cost < curr_data.g) {
      curr_data.parent_id = curr_par.parent_id;
      curr_data.g = g_cost;
      curr_data.f = g_cost + curr_data.h;
    }
  }
}

void ThetaStar::setNeighbors(const tree_node & curr_data, const int & curr_id)
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
    g_cost = curr_data.g + getEuclideanCost(curr_data.x, curr_data.y, mx, my) +
      getTraversalCost(mx, my);

    getIndex(mx, my, m_id);

    if (m_id == -1) {
      addToNodesData(index_generated_);
      m_id = index_generated_;
      addIndex(mx, my, m_id);
      index_generated_++;
    }

    curr_node = &nodes_data_[m_id];

    h_cost = getHCost(mx, my);
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

void ThetaStar::backtrace(std::vector<coordsW> & raw_points, int curr_id)
{
  std::vector<coordsW> path_rev;
  coordsW world{};
  do {
    costmap_->mapToWorld(nodes_data_[curr_id].x, nodes_data_[curr_id].y, world.x, world.y);
    path_rev.push_back(world);
    if (path_rev.size() > 1) {
      curr_id = nodes_data_[curr_id].parent_id;
    }
  } while (curr_id != 0);
  costmap_->mapToWorld(nodes_data_[curr_id].x, nodes_data_[curr_id].y, world.x, world.y);
  path_rev.push_back(world);

  raw_points.reserve(path_rev.size());
  for (int i = static_cast<int>(path_rev.size()) - 1; i >= 0; i--) {
    raw_points.push_back(path_rev[i]);
  }
}

bool ThetaStar::losCheck(
  const int & x0, const int & y0, const int & x1, const int & y1,
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

void ThetaStar::resetContainers()
{
  index_generated_ = 0;
  int last_size_x = size_x_;
  int last_size_y = size_y_;
  int curr_size_x = static_cast<int>(costmap_->getSizeInCellsX());
  int curr_size_y = static_cast<int>(costmap_->getSizeInCellsY());
  if (((last_size_x != curr_size_x) || (last_size_y != curr_size_y)) &&
    static_cast<int>(node_position_.size()) < (curr_size_x * curr_size_y))
  {
    initializePosn(curr_size_y * curr_size_x - last_size_y * last_size_x);
    nodes_data_.reserve(curr_size_x * curr_size_y);
  } else {
    initializePosn();
  }
  size_x_ = curr_size_x;
  size_y_ = curr_size_y;
}

void ThetaStar::initializePosn(int size_inc)
{
  int i = 0;

  if (!node_position_.empty()) {
    for (; i < size_x_ * size_y_; i++) {
      node_position_[i] = -1;
    }
  }

  for (; i < size_inc; i++) {
    node_position_.emplace_back(-1);
  }
}
}  //  namespace theta_star
