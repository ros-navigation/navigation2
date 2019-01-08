/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#include "costmap_queue/costmap_queue.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

using std::hypot;

namespace costmap_queue
{

CostmapQueue::CostmapQueue(nav2_costmap_2d::Costmap2D & costmap, bool manhattan)
: MapBasedQueue(), costmap_(costmap), max_distance_(-1), manhattan_(manhattan),
  cached_max_distance_(-1)
{
  reset();
}

void CostmapQueue::reset()
{
  unsigned int size_x = costmap_.getSizeInCellsX(), size_y = costmap_.getSizeInCellsY();
  if (seen_.size() != size_x * size_y) {
    seen_.resize(size_x * size_y);
  }
  std::fill(seen_.begin(), seen_.end(), false);
  computeCache();
  MapBasedQueue::reset();
}

void CostmapQueue::enqueueCell(unsigned int x, unsigned int y)
{
  unsigned int index = costmap_.getIndex(x, y);
  enqueueCell(index, x, y, x, y);
}

void CostmapQueue::enqueueCell(
  unsigned int index, unsigned int cur_x, unsigned int cur_y,
  unsigned int src_x, unsigned int src_y)
{
  if (seen_[index]) {return;}

  // we compute our distance table one cell further than the inflation radius
  // dictates so we can make the check below
  double distance = distanceLookup(cur_x, cur_y, src_x, src_y);
  CellData data(distance, index, cur_x, cur_y, src_x, src_y);
  if (validCellToQueue(data)) {
    seen_[index] = true;
    enqueue(distance, data);
  }
}

CellData CostmapQueue::getNextCell()
{
  // get the highest priority cell and pop it off the priority queue
  CellData current_cell = front();
  pop();

  unsigned int index = current_cell.index_;
  unsigned int mx = current_cell.x_;
  unsigned int my = current_cell.y_;
  unsigned int sx = current_cell.src_x_;
  unsigned int sy = current_cell.src_y_;

  // attempt to put the neighbors of the current cell onto the queue
  unsigned int size_x = costmap_.getSizeInCellsX();
  if (mx > 0) {
    enqueueCell(index - 1, mx - 1, my, sx, sy);
  }
  if (my > 0) {
    enqueueCell(index - size_x, mx, my - 1, sx, sy);
  }
  if (mx < size_x - 1) {
    enqueueCell(index + 1, mx + 1, my, sx, sy);
  }
  if (my < costmap_.getSizeInCellsY() - 1) {
    enqueueCell(index + size_x, mx, my + 1, sx, sy);
  }

  return current_cell;
}

void CostmapQueue::computeCache()
{
  if (max_distance_ == -1) {
    max_distance_ = std::max(costmap_.getSizeInCellsX(), costmap_.getSizeInCellsY());
  }
  if (max_distance_ == cached_max_distance_) {return;}
  cached_distances_.clear();

  cached_distances_.resize(max_distance_ + 2);

  for (unsigned int i = 0; i < cached_distances_.size(); ++i) {
    cached_distances_[i].resize(max_distance_ + 2);
    for (unsigned int j = 0; j < cached_distances_[i].size(); ++j) {
      if (manhattan_) {
        cached_distances_[i][j] = i + j;
      } else {
        cached_distances_[i][j] = hypot(i, j);
      }
    }
  }
  cached_max_distance_ = max_distance_;
}

}  // namespace costmap_queue
