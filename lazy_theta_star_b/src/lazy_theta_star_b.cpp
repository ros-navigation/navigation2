// Copyright 2020 Anshumaan Singh
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <vector>
#include "lazy_theta_star_b/lazy_theta_star_b.h"

namespace lazyThetaStarB
{

void LazyThetaStarB::makePlan(std::vector<float> & x_path, std::vector<float> & y_path)
{
  RCLCPP_INFO(
    node_->get_logger(), "Path Search Begins");

  x_path.clear();
  y_path.clear();
  id id_gen = 0;
  sizeX = costmap_->getSizeInCellsX();
  sizeY = costmap_->getSizeInCellsY();
  posn.clear();
  initializePosn();

  std::cout << posn[0] << '\n';
  // has been added so that if the robot is stuck at a place with costmap cost higher than 127
  // then only for that run, the planner be able to give a path
  if (costmap_->getCost(src.x, src.y) >= lethal_cost && costmap_->getCost(src.x, src.y) <= 253) {
    lethal_cost = costmap_->getCost(src.x, src.y);
  }

  data.reserve(static_cast<int>( (sizeX) * (sizeY) * 0.01 ) );
  clearRobotCell(src.x, src.y);

  RCLCPP_INFO(
    node_->get_logger(), "GOT THE SOURCE AND DESTINATION ------ %i, %i && %i, %i",
    src.x, src.y, dst.x, dst.y);

  if (!isSafe(src.x, src.y) || !isSafe(dst.x, dst.y) || !withinLimits(src.x, src.y) ||
    !withinLimits(dst.x, dst.y))
  {
    RCLCPP_INFO(node_->get_logger(), "NO PATH POSSIBLE!!!!!!");
  }

  data.push_back({(src.x), (src.y), 0, dist(src.x, src.y, dst.x, dst.y), 0, -1,
      dist(src.x, src.y, dst.x, dst.y)});

  addIndex(src.x, src.y, id_gen);

  pq.push_back({0, &(data[0].g), &(data[0].f)});
  pq.push_back({0, &(data[0].g), &(data[0].f)});
  // added as a buffer, since my binaryHeapDelMin requires for the start index to 1 instead of 0

  // The algorithm begins here
  id curr_id = 0;
  id_gen++;

  const int moves[8][2] = {
          {0, +1},
          {0, -1},
          {-1, 0},
          {+1, 0},
          {+1, -1},
          {-1, -1},
          {-1, +1},
          {+1, +1},
  };

  int cy;
  int cx;
  cx = (data[curr_id].x);
  cy = (data[curr_id].y);

  if (!losCheck(src.x, src.y, dst.x, dst.y)) {

    while (pq.size() > 1) {
      int mx, my;
      id m_id;
      id curr_par = (data[curr_id].parentId);
      std::cout << "THE CURRENT POINT IS : " << cx << '\t' << cy << '\n';

      //          The condition if current point is the destination
      if (cx == dst.x && cy == dst.y) {
          std::cout<<"Got IT!!!!!!!!!"<<'\n';
          if (!(losCheck(cx, cy, data[curr_par].x, data[curr_par].y))) {
          float min_dist = INF_COST;
          int min_dist_id = curr_par;
              std::cout<<"NEED IT!!!!!!!!!"<<'\n';

          for (int i = 0; i < how_many_corners; i++) {
              std::cout<<"IT!!!!!!!!!"<<'\n';
              mx = cx + moves[i][0];
            my = cy + moves[i][1];
            if (withinLimits(mx, my)) {
              m_id = getIndex(mx, my);
              if (m_id != 0) {
                if (data[m_id].f < min_dist) {
                  min_dist = data[m_id].f;
                  min_dist_id = m_id;
                }
              }
            }
            data[curr_id].parentId = min_dist_id;
            data[curr_id].g = data[min_dist_id].g +
              dist(cx, cy, data[min_dist_id].x, data[min_dist_id].y);
            data[curr_id].f = data[curr_id].g + data[curr_id].h;
          }
        }
        break;
      }

      if (!(losCheck(cx, cy, data[curr_par].x, data[curr_par].y))) {
        float min_dist = INF_COST;
        int min_dist_id = curr_par;

        for (int i = 0; i < how_many_corners; i++) {
          mx = cx + moves[i][0];
          my = cy + moves[i][1];
          if (withinLimits(mx, my)) {
            m_id = getIndex(mx, my);
            if (m_id != 0) {
              if (data[m_id].f < min_dist) {
                min_dist = data[m_id].f;
                min_dist_id = m_id;
              }
            }
          }
          data[curr_id].parentId = min_dist_id;
          data[curr_id].g = data[min_dist_id].g + dist(cx, cy, data[min_dist_id].x,
              data[min_dist_id].y);
          data[curr_id].f = data[curr_id].g + data[curr_id].h;
        }
      }
      curr_par = data[curr_id].parentId;

      for (int i = 0; i < how_many_corners; i++) {
        mx = cx + moves[i][0];
        my = cy + moves[i][1];

        if (mx == data[curr_par].x && my == data[curr_par].y) {
          continue;
        }

        if (withinLimits(mx, my)) {
          float g_cost = data[curr_par].g + dist(mx, my, data[curr_par].x, data[curr_par].y);
          float h_cost, cal_cost;
          m_id = getIndex(mx, my);
          if (m_id == 0 && isSafe(mx, my)) {
            h_cost = dist(mx, my, dst.x, dst.y);
            cal_cost = g_cost + h_cost;
            data.push_back({mx, my, g_cost, h_cost, curr_par, -1, cal_cost});
            addIndex(mx, my, id_gen);
            pushToPq(id_gen);
            id_gen++;
            continue;
          } else if (m_id != 0) {
            h_cost = data[m_id].h;
            cal_cost = g_cost + h_cost;
            if (data[m_id].f > cal_cost) {
              data[m_id].g = g_cost;
              data[m_id].f = cal_cost;
              data[m_id].parentId = curr_par;
              if (data[m_id].closed == 1) {
                data[m_id].closed = -1;
                pushToPq(m_id);
              }
            }
            continue;
          }
        }
      }

      do {
        curr_id = pq[1].pos_id;
        cx = data[curr_id].x;
        cy = data[curr_id].y;
      } while (!isSafe(cx, cy));
      binaryHeapDelMin();
      data[curr_id].closed = -1;
    }

  } else {
    RCLCPP_INFO(node_->get_logger(), "Straight Path");
    data.push_back({(dst.x), (dst.y), dist(cx, cy, src.x, src.y), 0, 0});
    curr_id = 1;
  }

  RCLCPP_INFO(node_->get_logger(), "REACHED DEST  %i,   %i", dst.x, dst.y);

  backtrace(&path, curr_id);

  for (auto & p : path) {
    pts<double> map_coords;
    costmap_->mapToWorld(data[p].x, data[p].y, map_coords.x, map_coords.y);
    RCLCPP_INFO(node_->get_logger(), "%f, %f", map_coords.x, map_coords.y);
    x_path.push_back(map_coords.x);
    y_path.push_back(map_coords.y);
  }
  std::cout << data.size() << '\n';

  path.clear();
  data.clear();

  pq.clear();
  lethal_cost = LETHAL_COST;
  std::cout << "Publishing the Path" << '\n';
}

void LazyThetaStarB::binaryHeapDelMin()
{
  int succ = 2;
  int size = pq.size() - 1;
  int sz = size;
  int hole = 1;
  // the main bottom up part where it compares and assigns the smallest value to the hole
  // remember that for this part the heap starts at index 1
  while (succ < sz) {
    double k1 = *(pq[succ].f);
    double k2 = *(pq[succ + 1].f);
    if (k1 > k2) {
      succ++;
      pq[hole] = pq[succ];
    } else {
      pq[hole] = pq[succ];
    }
    hole = succ;
    succ <<= 1;
  }
  // this part checks if the value to be used to fill the last row's hole is small or not
  // if not small then it slides the small value up till it reaches the apt position
  double bubble = *(pq[sz].f);
  int pred = hole >> 1;
  while (*(pq[pred].f) > bubble) {
    pq[hole] = pq[pred];
    hole = pred;
    pred >>= 1;
  }
  // this part simply assigns the last value in the heap to the hole after checking it
  // for maintaining the heap data structure
  pq[hole] = pq[sz];
  pq.pop_back();
}

// TODO(Anshhu-man567): FIND A WAY TO PASS THE VALUES BY REFERENCE WHEN CHECKING
bool LazyThetaStarB::losCheck(int x0, int y0, int x1, int y1)
{
  int dy = y1 - y0, dx = x1 - x0, f = 0;
  int sx, sy;

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
    while (x0 != x1) {
      f += dy;
      if (f >= dx) {
        if (!isSafe2(x0 + (sx - 1) / 2, y0 + (sy - 1) / 2)) {
          return false;
        }
          y0 += sy;
        f -= dx;
      }
      if (f != 0 && !isSafe2(x0 + (sx - 1) / 2, y0 + (sy - 1) / 2)) {
        return false;
      }
      if (dy == 0 && !isSafe2(x0 + (sx - 1) / 2, y0) && !isSafe2(x0 + (sx - 1) / 2, y0 - 1)) {
        return false;
      }
        x0 += sx;
    }
  } else {
    while (y0 != y1) {
      f = f + dx;
      if (f >= dy) {
        if (!isSafe2(x0 + (sx - 1) / 2, y0 + (sy - 1) / 2)) {
          return false;
        }
          x0 += sx;
        f -= dy;
      }
      if (f != 0 && !isSafe2(x0 + (sx - 1) / 2, y0 + (sy - 1) / 2)) {
        return false;
      }
      if (dx == 0 && !isSafe2(x0, y0 + (sy - 1) / 2) && !isSafe2(x0 - 1, y0 + (sy - 1) / 2)) {
        return false;
      }
        y0 += sy;
    }
  }
  return true;
}

void LazyThetaStarB::initializePosn()
{
  for (int i = 0; i < sizeY * sizeX; i++) {
    posn.push_back(0);
  }
}

void LazyThetaStarB::addIndex(const int & cx, const int & cy, const id & index)
{
  posn[sizeX * cy + cx] = index;
}

id LazyThetaStarB::getIndex(const int & cx, const int & cy)
{
  return posn[sizeX * cy + cx];
}

void LazyThetaStarB::backtrace(std::vector<id> * waypt, id curr_id)
{
  std::vector<id> wayptRev;
  do {
    wayptRev.push_back(curr_id);
      curr_id = data[curr_id].parentId;
    if (data[curr_id].got_here == 1) {
          std::cout
                  << "------------------------------------------------------>  got the same error   <-------------------------------------------------------"
                  << '\n';
          break;
    }
    data[curr_id].got_here = 1;
  } while (curr_id != 0);
  wayptRev.push_back(0);

  for (int i = wayptRev.size() - 1; i >= 0; i--) {
    waypt->push_back(wayptRev[i]);
  }
}

void LazyThetaStarB::pushToPq(id & idThis)
{
  pq.push_back({idThis, &(data[idThis].g), &(data[idThis].f)});
  push_heap(pq.begin() + 1, pq.end(), comp);
}

bool LazyThetaStarB::isSafe(const int & cx, const int & cy)
{
  return costmap_->getCost(cx, cy) < lethal_cost;
}

bool LazyThetaStarB::isSafe2(int cx, int cy)
{
  return costmap_->getCost(cx, cy) < lethal_cost;
}

bool LazyThetaStarB::withinLimits(const int & x, const int & y)
{
  return x > 0 && x <= sizeX && y > 0 && y <= sizeY;
}

void LazyThetaStarB::clearRobotCell(int mx, int my)
{
  costmap_->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
}

}  // namespace lazyThetaStarB
