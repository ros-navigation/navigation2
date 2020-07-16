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

#include "lazy_theta_star_planner/lazy_theta_star2.h"
#include <vector>

namespace lazyThetaStar
{

LazyThetaStar::LazyThetaStar()
{
  sizeX = 0;
  sizeY = 0;
}

bool LazyThetaStar::getPath(std::vector<coords<world_pts>> & raw_path)
{
  RCLCPP_INFO(node_->get_logger(), "Path Planning Begins....");

  initializeStuff();

  std::cout << src.x << '\t' << src.y << '\t' << dst.x << '\t' << dst.y << '\n';

  data.push_back({src.x, src.y, 0, dist(src.x, src.y, dst.x, dst.y),
      0, 1, dist(src.x, src.y, dst.x, dst.y)});
  addIndex(src.x, src.y, id_gen);
  pq.push({id_gen, &(data[id_gen].f)});

  id_gen++;
  id curr_id = 0;

  if (!losCheck(src.x, src.y, dst.x, dst.y)) {
    while (!pq.empty()) {

      tree_node curr_data = data[curr_id];
      curr_data.closed = -1;
      id curr_par = curr_data.parent_id;

      if (isGoal(curr_data.x, curr_data.y)) {
        if (!(losCheck(curr_data.x, curr_data.y, data[curr_par].x, data[curr_par].y))) {
          resetParent(curr_data);
        }
        break;
      }

      if (!(losCheck(curr_data.x, curr_data.y, data[curr_par].x, data[curr_par].y))) {
        resetParent(curr_data);
      }

      setNeighbors(curr_data);

      curr_id = pq.top().pos_id;
      pq.pop();
    }
    if (pq.empty()) {
      RCLCPP_INFO(node_->get_logger(), "No Path Found !!!!!!!!!!!");
      raw_path.clear();
      return false;
    }

  } else {
    RCLCPP_INFO(node_->get_logger(), "Straight Line Path Found!");
    data.push_back({dst.x, dst.y, dist(src.x, src.y, dst.x, dst.y), 0, 0, 1,
        dist(src.x, src.y, dst.x, dst.y)});
    curr_id = id_gen;
  }

  RCLCPP_INFO(node_->get_logger(), "REACHED DEST  %i,   %i", data[curr_id].x, data[curr_id].y);

  backtrace(raw_path, curr_id);

  return true;
}

void LazyThetaStar::initializePosn()
{
  posn.clear();
  for (int i = 0; i < sizeX * sizeY; i++) {
    posn.push_back(-1);
  }
}

bool LazyThetaStar::losCheck(map_pts & x0, map_pts & y0, map_pts & x1, map_pts & y1) const
{
  int dy = abs(y1 - y0), dx = abs(x1 - x0), f = 0;
  int sx = x1 > x0 ? 1 : -1, sy = y1 > y0 ? 1 : -1;
  int cx = x0, cy = y0;
  int ux = (sx - 1) / 2, uy = (sy - 1) / 2;

  if (dx >= dy) {
    while (cx != x1) {
      f += dy;
      if (f >= dx) {
        if (!isSafe(cx + ux, cy + uy)) {
          return false;
        }
        cy += sy;
        f -= dx;
      }
      if (f != 0 && !isSafe(cx + ux, cy + uy)) {
        return false;
      }
      if (dy == 0 && !isSafe(cx + ux, cy) && !isSafe(cx + ux, cy - 1)) {
        return false;
      }
      cx += sx;
    }
  } else {
    while (cy != y1) {
      f = f + dx;
      if (f >= dy) {
        if (!isSafe(cx + ux, cy + uy)) {
          return false;
        }
        cx += sx;
        f -= dy;
      }
      if (f != 0 && !isSafe(cx + ux, cy + uy)) {
        return false;
      }
      if (dx == 0 && !isSafe(cx, cy + uy) && !isSafe(cx - 1, cy + uy)) {
        return false;
      }
      cy += sy;
    }
  }
  return true;
}

void LazyThetaStar::backtrace(std::vector<coords<world_pts>> & raw_points, id & curr_id)
{
  std::vector<coords<world_pts>> path_rev;
  do {
    coords<world_pts> world;
    costmap_->mapToWorld(data[curr_id].x, data[curr_id].y, world.x, world.y);
    path_rev.push_back({world.x, world.y});
    curr_id = data[curr_id].parent_id;
  } while (curr_id != 0);
  coords<world_pts> w;
  costmap_->mapToWorld(data[curr_id].x, data[curr_id].y, w.x, w.y);
  path_rev.push_back({w.x, w.y});

  for (int i = path_rev.size() - 1; i >= 0; i--) {
    raw_points.push_back(path_rev[i]);
  }
}

void LazyThetaStar::resetParent(tree_node & curr_data)
{
  map_pts mx, my;
  id m_id;
  cost min_dist = INF_COST;
  id min_dist_id;

  for (int i = 0; i < how_many_corners; i++) {
    mx = curr_data.x + moves[i][0];
    my = curr_data.y + moves[i][1];

    if (withinLimits(mx, my)) {
      getIndex(mx, my, m_id);
      if (m_id != -1) {
        if (data[m_id].f < min_dist) {
          min_dist = data[m_id].f;
          min_dist_id = m_id;
        }
      }
    }
  }
  curr_data.parent_id = min_dist_id;
  curr_data.g = data[min_dist_id].g +
    dist(curr_data.x, curr_data.y, data[min_dist_id].x, data[min_dist_id].y);
  curr_data.f = curr_data.g + curr_data.h;
}

bool LazyThetaStar::isGoal(map_pts & cx, map_pts & cy)
{
  return cx == dst.x && cy == dst.y;
}
void LazyThetaStar::setNeighbors(tree_node & curr_data)
{

  tree_node & curr_par = data[curr_data.parent_id];

  map_pts mx, my;
  id m_id;
  for (int i = 0; i < how_many_corners; i++) {
    mx = curr_data.x + moves[i][0];
    my = curr_data.y + moves[i][1];

    if (mx == curr_par.x && my == curr_par.y) {
      continue;
    }

    if (withinLimits(mx, my)) {
      cost g_cost = curr_par.g + dist(mx, my, curr_par.x, curr_par.y);
      cost h_cost, cal_cost;
      getIndex(mx, my, m_id);

      if (isSafe(mx, my) && m_id == -1) {
        h_cost = dist(mx, my, dst.x, dst.y);
        cal_cost = g_cost + h_cost;
        data.push_back({mx, my, g_cost, h_cost, curr_data.parent_id, -1, cal_cost});
        pq.push({id_gen, &(data[id_gen].f)});
        addIndex(mx, my, id_gen);
        id_gen++;
        continue;
      } else if (m_id != -1) {
        tree_node & curr_node = data[m_id];
        h_cost = data[m_id].h;
        cal_cost = g_cost + h_cost;
        if (curr_node.f > cal_cost) {
          curr_node.g = g_cost;
          curr_node.f = cal_cost;
          curr_node.parent_id = curr_data.parent_id;
          if (curr_node.closed == 1) {
            curr_node.closed = -1;
            pq.push({m_id, &(data[m_id].f)});
          }
        }
        continue;
      }
    }
  }
}

void LazyThetaStar::initializeStuff()
{
  id_gen = 0;
  if (costmap_->getCost(src.x,
    src.y) > lethal_cost && costmap_->getCost(src.x, src.y) < LETHAL_COST)
  {
    lethal_cost = costmap_->getCost(src.x, src.y);
  }
  sizeX = costmap_->getSizeInCellsX();
  sizeY = costmap_->getSizeInCellsY();
  initializePosn();
  data.reserve(static_cast<int>(sizeX * sizeY * 0.05));
}

void LazyThetaStar::clearStuff()
{
  data.clear();
  pq = std::priority_queue<pos, std::vector<pos>, comp>();
  posn.clear();
}
} //  namespace lazyThetaStar
