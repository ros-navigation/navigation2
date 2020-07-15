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
  pushToPq(id_gen);
  pushToPq(id_gen);
  // extra one added due to binaryHeapDelMin starting from index 1

  id_gen++;
  id curr_id = 0;

  if (!losCheck(src.x, src.y, dst.x, dst.y)) {

    while (pq.size() > 0) {

      map_pts cx = data[curr_id].x;
      map_pts cy = data[curr_id].y;
      id curr_par = data[curr_id].parent_id;

      if (isGoal(cx, cy)) {
        if (!(losCheck(cx, cy, data[curr_par].x, data[curr_par].y))) {
          resetParent(cx, cy, curr_id);
        }
        break;
      }

      if (!(losCheck(cx, cy, data[curr_par].x, data[curr_par].y))) {
        resetParent(cx, cy, curr_id);
      }

      setNeighbors(cx, cy, curr_id);

      getNextNode(curr_id);
    }

    if (pq.size() < 1) {
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
  int dy = y1 - y0, dx = x1 - x0, f = 0;
  int sx, sy;
  int cx = x0, cy = y0;

  if (dy < 0) {
    dy = -dy;
    sy = -1;
  } else {
    sy = 1;
  }

  if (dx < 0) {
    dx = -dx;
    sx = -1;
  } else {
    sx = 1;
  }

  if (dx >= dy) {
    while (cx != x1) {
      f += dy;
      if (f >= dx) {
        if (!isSafe(cx + (sx - 1) / 2, cy + (sy - 1) / 2)) {
          return false;
        }
        cy += sy;
        f -= dx;
      }
      if (f != 0 && !isSafe(cx + (sx - 1) / 2, cy + (sy - 1) / 2)) {
        return false;
      }
      if (dy == 0 && !isSafe(cx + (sx - 1) / 2, cy) && !isSafe(cx + (sx - 1) / 2, cy - 1)) {
        return false;
      }
      cx += sx;
    }
  } else {
    while (cy != y1) {
      f = f + dx;
      if (f >= dy) {
        if (!isSafe(cx + (sx - 1) / 2, cy + (sy - 1) / 2)) {
          return false;
        }
        cx += sx;
        f -= dy;
      }
      if (f != 0 && !isSafe(cx + (sx - 1) / 2, cy + (sy - 1) / 2)) {
        return false;
      }
      if (dx == 0 && !isSafe(cx, cy + (sy - 1) / 2) && !isSafe(cx - 1, cy + (sy - 1) / 2)) {
        return false;
      }
      cy += sy;
    }
  }
  return true;
}

void LazyThetaStar::binaryHeapDelMin()
{
//   pop_heap(pq.begin() + 1, pq.end(), comp);
//   pq.pop_back();
 int hole = 1;
 int succ = 2, size = pq.size() - 1, sz = size;
 // the main bottom up part where it compares and assigns the smallest value to the hole
 // remember that for this part the heap starts at index 1

 while (succ < sz) {
   double k1 = *(pq[succ].f);
   double k2 = *(pq[succ + 1].f);
   // std::cout<<k1<<'\t'<<k2<<'\n';
   if (k1 > k2) {
     // std::cout<<"came 1"<<'\n';
     succ++;
     // data[hole].key = k2;
     pq[hole] = pq[succ];
   } else {
     // std::cout<<"came 2"<<'\n';
     //  data[hole].key = k1;
     pq[hole] = pq[succ];
   }
   hole = succ;
   succ <<= 1;
 }

 // this part checks if the value to be used to fill the last row's hole is small or not
 // if not small then it slides the small value up till it reaches the apt position
 double bubble = *(pq[sz].f);
 int pred = hole >> 1;
 //  std::cout<<"HERE"<<'\t'<<data[pred].f<<'\t'<<bubble<<'\n';
 while (*(pq[pred].f) > bubble) {
   //    std::cout<<"HERE"<<'\t'<<hole<<'\n';
   pq[hole] = pq[pred];
   hole = pred;
   pred >>= 1;
 }
 // std::cout<<"HERE AT:"<<'\t'<<hole<<'\n';

 // this part simply assigns the last value in the haep to the hole after checking it
 // for maintaining the heap data structure
 pq[hole] = pq[sz];
 pq.pop_back();
}

void LazyThetaStar::pushToPq(id id_this)
{
  pq.push_back({id_this, &(data[id_this].f)});
  push_heap(pq.begin() + 1, pq.end(), comp);
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

void LazyThetaStar::resetParent(map_pts & cx, map_pts & cy, id & curr_id)
{
  map_pts mx, my;
  id m_id;
  cost min_dist = INF_COST;
  id min_dist_id;

  for (int i = 0; i < how_many_corners; i++) {
    mx = cx + moves[i][0];
    my = cy + moves[i][1];
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
  data[curr_id].parent_id = min_dist_id;
  data[curr_id].g = data[min_dist_id].g + dist(cx, cy, data[min_dist_id].x, data[min_dist_id].y);
  data[curr_id].f = data[curr_id].g + data[curr_id].h;
}

bool LazyThetaStar::isGoal(map_pts & cx, map_pts & cy)
{
  return cx == dst.x && cy == dst.y;
}

void LazyThetaStar::setNeighbors(map_pts & cx, map_pts & cy, id & curr_id)
{
  id curr_par = data[curr_id].parent_id;
  map_pts px = data[curr_par].x, py = data[curr_par].y;
  cost g_cost_par = data[curr_par].g;

  map_pts mx, my;
  id m_id;
  for (int i = 0; i < 8; i++) {
    mx = cx + moves[i][0];
    my = cy + moves[i][1];
    if (mx == px && my == py) {
      continue;
    }

    if (withinLimits(mx, my)) {
      cost g_cost = g_cost_par + dist(mx, my, px, py);
      cost h_cost, cal_cost;
      getIndex(mx, my, m_id);

      if (isSafe(mx, my) && m_id == -1) {
        h_cost = dist(mx, my, dst.x, dst.y);
        cal_cost = g_cost + h_cost;
        data.push_back({mx, my, g_cost, h_cost, curr_par, -1, cal_cost});
        pushToPq(id_gen);
        addIndex(mx, my, id_gen);
        id_gen++;
        continue;
      } else if (m_id != -1) {
        tree_node & curr_node = data[m_id];
        h_cost = curr_node.h;
        cal_cost = g_cost + h_cost;
        if (curr_node.f > cal_cost) {
          curr_node.g = g_cost;
          curr_node.f = cal_cost;
          curr_node.parent_id = curr_par;
          if (curr_node.closed == 1) {
            curr_node.closed = -1;
            pushToPq(m_id);     
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
  sizeX = costmap_->getSizeInCellsX();
  sizeY = costmap_->getSizeInCellsY(); 
  initializePosn();
  data.reserve(static_cast<int>(sizeX * sizeY * 0.05));
}

void LazyThetaStar::clearStuff()
{
  data.clear();
  pq.clear();
  posn.clear();
}

void LazyThetaStar::getNextNode(id & curr_id)
{
  curr_id = pq[1].pos_id;
  data[curr_id].closed = -1;
  binaryHeapDelMin();
}


//void LazyThetaStar::tfMapToWorld(
//  map_pts & mx,
//  map_pts & my,
//  world_pts & wx,
//  world_pts & wy);
//{
//wx = costmap_->getOriginX() + (mx) * costmap_->getResolution();
//wy = costmap_->getOriginY() + (my) * costmap_->getResolution();
//}

} //  namespace lazyThetaStar
